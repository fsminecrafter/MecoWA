/*
 * debugui.cpp  –  MecoWA Dear ImGui debug overlay
 *
 * Panels
 * ──────
 *   Scene Objects   – lists every entry in sceneModels with pos/rot/scale sliders
 *   Physics         – shows every gPhysicsLinks entry with live body state
 *                     *** pos/rot edits go through BodyInterface so they stick ***
 *   Engine Settings – exposes mecowa.h globals (camera speed, sensitivity, gravity…)
 *   Jolt Info       – body count, active bodies, jolt version string
 *   Renderer        – wireframe toggle, light direction, brightness, lightStrength
 *   Performance     – frame time, FPS, a rolling frame-time graph
 *
 * Toggle: Alt + D
 *
 * Dependencies (add to your vcxproj / CMake):
 *   imgui.cpp  imgui_draw.cpp  imgui_tables.cpp  imgui_widgets.cpp
 *   imgui_impl_glfw.cpp  imgui_impl_opengl3.cpp
 * All are in  <imgui_dir>/  and  <imgui_dir>/backends/
 *
 * FIX LOG
 * -------
 *  - PanelPhysics: position/rotation drag edits now call
 *      BodyInterface::SetPositionAndRotation + ActivateBody
 *    so they are not overwritten by Physics_SyncToEngine() each frame.
 *  - PanelSceneObjects: scale edits still go directly to the instance
 *    (scale is not a Jolt concept for box/convex bodies here).
 *  - Physics_SyncToEngine (engine.cpp): converts JPH::Quat->glm::quat
 *    with the correct (w,x,y,z) constructor order and writes Euler degrees.
 */

#include "debugui.h"

 // ── ImGui core ────────────────────────────────────────────────────────────────
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

// ── Project headers ───────────────────────────────────────────────────────────
#include "engine.h"        // sceneModels, ModelInstance
#include "mecowa.h"        // windowWidth/Height, gravityG, camera* globals
#include "jolt_init.h"     // gPhysics, gJobs
#include "jolt_bridge.h"   // gPhysicsLinks, PhysicsLink
#include "jolt_layers.h"

// ── Jolt ──────────────────────────────────────────────────────────────────────
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/Body.h>

// ── GLM ───────────────────────────────────────────────────────────────────────
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/quaternion.hpp>

// ── GLFW / OpenGL ─────────────────────────────────────────────────────────────
#include <glfw/include/GLFW/glfw3.h>
#include <glad/include/glad/glad.h>

// ── Standard ──────────────────────────────────────────────────────────────────
#include <string>
#include <deque>
#include <algorithm>
#include <cstring>
#include <cstdio>


// ─────────────────────────────────────────────────────────────────────────────
//  Internal state
// ─────────────────────────────────────────────────────────────────────────────
namespace {

    bool  g_open = false;
    float g_prevTime = 0.0f;
    float g_deltaTime = 0.0f;

    std::deque<float> g_frameHistory;
    constexpr int     kHistorySize = 120;

    bool  g_wireframe = false;
    float g_lightDir[3] = { -0.5f, -1.0f, -0.3f };
    float g_brightness = 1.0f;
    float g_lightStrength = 1.0f;

    // ─────────────────────────────────────────────────────────────────────────────
    //  Style
    // ─────────────────────────────────────────────────────────────────────────────
    void ApplyMecoStyle()
    {
        ImGuiStyle& s = ImGui::GetStyle();
        s.WindowRounding = 6.0f;
        s.FrameRounding = 4.0f;
        s.ScrollbarRounding = 4.0f;
        s.GrabRounding = 4.0f;
        s.FramePadding = { 6, 4 };
        s.ItemSpacing = { 8, 5 };

        ImVec4* c = s.Colors;
        c[ImGuiCol_WindowBg] = { 0.08f, 0.09f, 0.12f, 0.92f };
        c[ImGuiCol_TitleBg] = { 0.10f, 0.14f, 0.22f, 1.00f };
        c[ImGuiCol_TitleBgActive] = { 0.14f, 0.20f, 0.35f, 1.00f };
        c[ImGuiCol_Header] = { 0.18f, 0.28f, 0.48f, 0.80f };
        c[ImGuiCol_HeaderHovered] = { 0.22f, 0.36f, 0.60f, 0.90f };
        c[ImGuiCol_HeaderActive] = { 0.26f, 0.44f, 0.72f, 1.00f };
        c[ImGuiCol_FrameBg] = { 0.14f, 0.16f, 0.22f, 1.00f };
        c[ImGuiCol_FrameBgHovered] = { 0.20f, 0.24f, 0.34f, 1.00f };
        c[ImGuiCol_FrameBgActive] = { 0.26f, 0.32f, 0.46f, 1.00f };
        c[ImGuiCol_SliderGrab] = { 0.34f, 0.56f, 0.90f, 1.00f };
        c[ImGuiCol_SliderGrabActive] = { 0.44f, 0.70f, 1.00f, 1.00f };
        c[ImGuiCol_Button] = { 0.20f, 0.30f, 0.50f, 0.80f };
        c[ImGuiCol_ButtonHovered] = { 0.26f, 0.40f, 0.66f, 1.00f };
        c[ImGuiCol_ButtonActive] = { 0.32f, 0.50f, 0.82f, 1.00f };
        c[ImGuiCol_CheckMark] = { 0.50f, 0.80f, 1.00f, 1.00f };
        c[ImGuiCol_Text] = { 0.88f, 0.90f, 0.95f, 1.00f };
        c[ImGuiCol_TextDisabled] = { 0.40f, 0.44f, 0.52f, 1.00f };
        c[ImGuiCol_Separator] = { 0.22f, 0.28f, 0.40f, 1.00f };
        c[ImGuiCol_Tab] = { 0.12f, 0.18f, 0.28f, 1.00f };
        c[ImGuiCol_TabHovered] = { 0.24f, 0.36f, 0.58f, 1.00f };
        c[ImGuiCol_TabActive] = { 0.20f, 0.30f, 0.50f, 1.00f };
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Table helpers
    // ─────────────────────────────────────────────────────────────────────────────

    bool BeginTwoColTable(const char* id)
    {
        return ImGui::BeginTable(id, 2,
            ImGuiTableFlags_BordersInnerV |
            ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingStretchProp);
    }

    void DrawVec3Row(const char* label, glm::vec3& v,
        float speed = 0.01f, float lo = -1000.f, float hi = 1000.f)
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        ImGui::PushID(label);
        ImGui::SetNextItemWidth(-1);
        ImGui::DragFloat3("##v", &v.x, speed, lo, hi, "%.3f");
        ImGui::PopID();
    }

    void DrawFloatRow(const char* label, float& v,
        float speed = 0.01f, float lo = 0.f, float hi = 100.f)
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        ImGui::PushID(label);
        ImGui::SetNextItemWidth(-1);
        ImGui::DragFloat("##f", &v, speed, lo, hi, "%.4f");
        ImGui::PopID();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Helper: find the PhysicsLink whose model pointer matches a ModelInstance*
    //  Returns nullptr if the object has no physics body.
    // ─────────────────────────────────────────────────────────────────────────────
    PhysicsLink* FindLinkForInstance(ModelInstance* inst)
    {
        for (auto& link : gPhysicsLinks)
            if (link.model == inst)
                return &link;
        return nullptr;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Helper: push a new position + rotation into Jolt so it survives the sync.
    //
    //  • pos   – desired world-space render position (mesh origin)
    //  • euler – desired Euler angles in degrees (same convention as ModelInstance)
    //
    //  The function adds back the renderOffset so Jolt's body centre ends up in
    //  the right place, then converts degrees->quat for Jolt.
    // ─────────────────────────────────────────────────────────────────────────────
    void PushTransformToJolt(PhysicsLink& link, const glm::vec3& pos, const glm::vec3& eulerDeg)
    {
        if (!gPhysics) return;
        JPH::BodyInterface& bi = gPhysics->GetBodyInterface();

        // Position: add render offset (e.g. half-box-height so COM is centred)
        glm::vec3 simPos = pos + link.renderOffset;
        JPH::RVec3 jPos(simPos.x, simPos.y, simPos.z);

        // Rotation: degrees -> radians -> glm::quat -> JPH::Quat
        glm::quat gq = glm::quat(glm::radians(eulerDeg));          // (w,x,y,z) internally
        JPH::Quat jRot(gq.x, gq.y, gq.z, gq.w);                   // JPH stores (x,y,z,w)

        // SetPositionAndRotation wakes the body automatically when activated.
        bi.SetPositionAndRotation(link.body, jPos, jRot,
            JPH::EActivation::Activate);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Panel: Scene Objects  (non-physics properties: scale, read-only info)
    //  Position and rotation for physics bodies are shown here too, but edits
    //  are routed through PushTransformToJolt.
    // ─────────────────────────────────────────────────────────────────────────────
    void PanelSceneObjects()
    {
        if (sceneModels.empty()) {
            ImGui::TextDisabled("No scene objects.");
            return;
        }

        int idx = 0;
        for (auto& obj : sceneModels)
        {
            ModelInstance& inst = obj.instance;
            PhysicsLink* link = FindLinkForInstance(&inst);

            char nodeLabel[128];
            snprintf(nodeLabel, sizeof(nodeLabel), "[%d] %s%s",
                idx++, obj.name.c_str(),
                link ? "  [physics]" : "");

            if (ImGui::TreeNodeEx(nodeLabel, ImGuiTreeNodeFlags_SpanFullWidth))
            {
                if (BeginTwoColTable("##scene_tbl"))
                {
                    ImGui::TableSetupColumn("Property", ImGuiTableColumnFlags_WidthFixed, 90.f);
                    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);

                    // ── Position ──────────────────────────────────────────────────
                    {
                        glm::vec3 editPos = inst.position;
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Position");
                        ImGui::TableSetColumnIndex(1);
                        ImGui::PushID("pos");
                        ImGui::SetNextItemWidth(-1);
                        if (ImGui::DragFloat3("##pos", &editPos.x, 0.01f, -1000.f, 1000.f, "%.3f"))
                        {
                            if (link)
                                PushTransformToJolt(*link, editPos, inst.rotation);
                            else
                                inst.position = editPos;
                        }
                        ImGui::PopID();
                    }

                    // ── Rotation ──────────────────────────────────────────────────
                    {
                        glm::vec3 editRot = inst.rotation;
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Rotation");
                        ImGui::TableSetColumnIndex(1);
                        ImGui::PushID("rot");
                        ImGui::SetNextItemWidth(-1);
                        if (ImGui::DragFloat3("##rot", &editRot.x, 0.1f, -360.f, 360.f, "%.2f°"))
                        {
                            if (link)
                                PushTransformToJolt(*link, inst.position, editRot);
                            else
                                inst.rotation = editRot;
                        }
                        ImGui::PopID();
                    }

                    // ── Scale (pure render, no Jolt equivalent) ───────────────────
                    DrawVec3Row("Scale", inst.scale, 0.005f, 0.001f, 100.f);

                    // ── Info ──────────────────────────────────────────────────────
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("Vertices");
                    ImGui::TableSetColumnIndex(1); ImGui::Text("%d", inst.vertexCount);

                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("Triangles");
                    ImGui::TableSetColumnIndex(1); ImGui::Text("%d", inst.indiciesCount);

                    ImGui::EndTable();
                }

                if (link)
                {
                    ImGui::TextDisabled("Physics body ID: %u", link->body.GetIndex());
                }

                ImGui::TreePop();
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Panel: Physics  (live Jolt body state + editable pos/rot via BodyInterface)
    // ─────────────────────────────────────────────────────────────────────────────
    void PanelPhysics()
    {
        if (!gPhysics) {
            ImGui::TextColored({ 1,0.4f,0.4f,1 }, "Jolt not initialised.");
            return;
        }

        unsigned activeBodies = gPhysics->GetNumActiveBodies(JPH::EBodyType::RigidBody);
        unsigned totalBodies = gPhysics->GetNumBodies();
        ImGui::Text("Bodies: %u total  |  %u active", totalBodies, activeBodies);
        ImGui::Separator();

        if (gPhysicsLinks.empty()) {
            ImGui::TextDisabled("No physics links.");
            return;
        }

        JPH::BodyInterface& bi = gPhysics->GetBodyInterface();

        int idx = 0;
        for (auto& link : gPhysicsLinks)
        {
            // Find a display name from sceneModels
            const char* name = "body";
            for (auto& obj : sceneModels)
                if (&obj.instance == link.model) { name = obj.name.c_str(); break; }

            char nodeLabel[128];
            snprintf(nodeLabel, sizeof(nodeLabel), "[%d] %s  (ID %u)",
                idx++, name, link.body.GetIndex());

            if (!ImGui::TreeNodeEx(nodeLabel, ImGuiTreeNodeFlags_SpanFullWidth))
                continue;

            bool isActive = bi.IsActive(link.body);
            ImGui::PushStyleColor(ImGuiCol_Text,
                isActive ? ImVec4{ 0.4f,1.f,0.5f,1.f } : ImVec4{ 0.5f,0.5f,0.55f,1.f });
            ImGui::Text(isActive ? "● Active" : "○ Sleeping");
            ImGui::PopStyleColor();

            // ── Read live state from Jolt ──────────────────────────────────────────
            JPH::Vec3 jPos = bi.GetPosition(link.body);
            JPH::Quat jRot = bi.GetRotation(link.body);
            JPH::Vec3 linVel = bi.GetLinearVelocity(link.body);
            JPH::Vec3 angVel = bi.GetAngularVelocity(link.body);

            // Convert Jolt rotation to Euler degrees for display / editing
            glm::quat gRot(jRot.GetW(), jRot.GetX(), jRot.GetY(), jRot.GetZ());
            glm::vec3 eulerDeg = glm::degrees(glm::eulerAngles(gRot));

            // Render position (subtract offset so the displayed value matches the mesh)
            glm::vec3 simPos(jPos.GetX(), jPos.GetY(), jPos.GetZ());
            glm::vec3 renderPos = simPos - link.renderOffset;

            if (BeginTwoColTable("##phy_tbl"))
            {
                ImGui::TableSetupColumn("Property", ImGuiTableColumnFlags_WidthFixed, 90.f);
                ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);

                // ── Editable Position ──────────────────────────────────────────────
                {
                    glm::vec3 editPos = renderPos;
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Position");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::PushID("jpos");
                    ImGui::SetNextItemWidth(-1);
                    if (ImGui::DragFloat3("##jpos", &editPos.x, 0.01f, -1000.f, 1000.f, "%.3f"))
                        PushTransformToJolt(link, editPos, eulerDeg);
                    ImGui::PopID();
                }

                // ── Editable Rotation (Euler degrees) ─────────────────────────────
                {
                    glm::vec3 editRot = eulerDeg;
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Rotation");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::PushID("jrot");
                    ImGui::SetNextItemWidth(-1);
                    if (ImGui::DragFloat3("##jrot", &editRot.x, 0.1f, -360.f, 360.f, "%.2f°"))
                        PushTransformToJolt(link, renderPos, editRot);
                    ImGui::PopID();
                }

                // ── Read-only velocity info ────────────────────────────────────────
                auto Row3 = [&](const char* lbl, float x, float y, float z) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("%s", lbl);
                    ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f  %.3f  %.3f", x, y, z);
                    };

                Row3("Lin vel", linVel.GetX(), linVel.GetY(), linVel.GetZ());
                Row3("Ang vel", angVel.GetX(), angVel.GetY(), angVel.GetZ());

                float speed = std::sqrt(
                    linVel.GetX() * linVel.GetX() +
                    linVel.GetY() * linVel.GetY() +
                    linVel.GetZ() * linVel.GetZ());
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("Speed");
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f m/s", speed);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("Render offset");
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3f  %.3f  %.3f",
                    link.renderOffset.x, link.renderOffset.y, link.renderOffset.z);

                ImGui::EndTable();
            }

            // ── Velocity / sleep controls ──────────────────────────────────────────
            if (ImGui::SmallButton("Wake"))
                bi.ActivateBody(link.body);
            ImGui::SameLine();
            if (ImGui::SmallButton("Freeze"))
                bi.DeactivateBody(link.body);
            ImGui::SameLine();
            if (ImGui::SmallButton("Zero vel")) {
                bi.SetLinearVelocity(link.body, JPH::Vec3::sZero());
                bi.SetAngularVelocity(link.body, JPH::Vec3::sZero());
            }

            ImGui::TreePop();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Panel: Engine / Camera Settings
    // ─────────────────────────────────────────────────────────────────────────────
    void PanelEngineSettings()
    {
        ImGui::SeparatorText("Camera");
        ImGui::SliderFloat("Speed", &cameraSpeed, 0.1f, 50.f);
        ImGui::SliderFloat("Sensitivity", &cameraSensitivity, 0.01f, 1.f);
        ImGui::SliderFloat("Pitch clamp", &cameraPitchClamp, 10.f, 89.f);
        ImGui::Checkbox("Invert Mouse Y", &invertMouseY);
        ImGui::Checkbox("Invert Mouse X", &invertMouseX);

        ImGui::Spacing();
        ImGui::SeparatorText("Simulation");

        if (ImGui::DragFloat("Gravity G", &gravityG, 0.01f, -50.f, 50.f, "%.3f"))
        {
            if (gPhysics)
                gPhysics->SetGravity(JPH::Vec3(0.f, gravityG, 0.f));
        }

        ImGui::DragFloat("Air density", &airDensity, 0.001f, 0.f, 10.f, "%.4f kg/m³");

        ImGui::Spacing();
        ImGui::SeparatorText("Window");
        ImGui::Text("Resolution: %d × %d", windowWidth, windowHeight);
        ImGui::Text("Version: %s", version.c_str());
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Panel: Renderer
    // ─────────────────────────────────────────────────────────────────────────────
    void PanelRenderer()
    {
        ImGui::SeparatorText("Geometry");
        if (ImGui::Checkbox("Wireframe", &g_wireframe))
            glPolygonMode(GL_FRONT_AND_BACK, g_wireframe ? GL_LINE : GL_FILL);

        ImGui::Spacing();
        ImGui::SeparatorText("Lighting");
        ImGui::SliderFloat3("Light direction", g_lightDir, -1.f, 1.f);
        ImGui::SliderFloat("Brightness", &g_brightness, 0.f, 5.f);
        ImGui::SliderFloat("Light strength", &g_lightStrength, 0.f, 5.f);

        ImGui::Spacing();
        ImGui::TextDisabled("Wire these uniforms in your render loop:");
        ImGui::TextDisabled("  shader.setVec3(\"lightDir\", {%.2f,%.2f,%.2f})",
            g_lightDir[0], g_lightDir[1], g_lightDir[2]);
        ImGui::TextDisabled("  shader.setFloat(\"brightness\", %.3f)", g_brightness);
        ImGui::TextDisabled("  shader.setFloat(\"lightStrength\", %.3f)", g_lightStrength);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Panel: Performance
    // ─────────────────────────────────────────────────────────────────────────────
    void PanelPerformance()
    {
        float fps = (g_deltaTime > 0.f) ? (1.f / g_deltaTime) : 0.f;
        ImGui::Text("FPS:        %.1f", fps);
        ImGui::Text("Frame time: %.2f ms", g_deltaTime * 1000.f);

        g_frameHistory.push_back(g_deltaTime * 1000.f);
        while ((int)g_frameHistory.size() > kHistorySize)
            g_frameHistory.pop_front();

        static float buf[kHistorySize];
        int n = (int)g_frameHistory.size();
        for (int i = 0; i < n; ++i) buf[i] = g_frameHistory[i];

        float maxMs = *std::max_element(buf, buf + n);
        maxMs = std::max(maxMs, 33.3f);

        char overlay[32];
        snprintf(overlay, sizeof(overlay), "%.1f ms", g_deltaTime * 1000.f);
        ImGui::PlotLines("##ft", buf, n, 0, overlay, 0.f, maxMs, { 0.f, 60.f });
        ImGui::TextDisabled("Last %d frames  (max %.1f ms)", n, maxMs);

        ImGui::Spacing();
        ImGui::SeparatorText("Jolt");
        if (gPhysics) {
            ImGui::Text("Active bodies:  %u", gPhysics->GetNumActiveBodies(JPH::EBodyType::RigidBody));
            ImGui::Text("Total bodies:   %u", gPhysics->GetNumBodies());
        }
        else {
            ImGui::TextDisabled("Jolt not running.");
        }
    }

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────

void DebugUI_Init(GLFWwindow* window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;

    ApplyMecoStyle();

    ImGui_ImplGlfw_InitForOpenGL(window, /*install_callbacks=*/true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void DebugUI_Render()
{
    float now = (float)glfwGetTime();
    g_deltaTime = now - g_prevTime;
    g_prevTime = now;

    if (!g_open) return;

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize({ 540, 640 }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos({ 20,  20 }, ImGuiCond_FirstUseEver);

    if (ImGui::Begin("MecoWA  —  Debug  (Alt+D to close)", &g_open,
        ImGuiWindowFlags_NoCollapse))
    {
        if (ImGui::BeginTabBar("##tabs"))
        {
            if (ImGui::BeginTabItem("Scene")) { PanelSceneObjects();   ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Physics")) { PanelPhysics();        ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Engine")) { PanelEngineSettings(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Render")) { PanelRenderer();       ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Perf")) { PanelPerformance();    ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
    }
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool DebugUI_HandleKey(int key, int action, int mods)
{
    if (key == GLFW_KEY_D && action == GLFW_PRESS && (mods & GLFW_MOD_ALT))
    {
        g_open = !g_open;
        return true;
    }
    return false;
}

void DebugUI_Shutdown()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

const float* DebugUI_GetLightDir() { return g_lightDir; }
float        DebugUI_GetBrightness() { return g_brightness; }
float        DebugUI_GetLightStrength() { return g_lightStrength; }