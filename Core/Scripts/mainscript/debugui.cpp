/*
 * debugui.cpp  –  MecoWA Dear ImGui debug overlay
 *
 * Tabs
 * ────
 *   Scene      – scene objects, pos/rot routed through BodyInterface
 *   Physics    – per-link live state, editable pos/rot/vel + impulse applicator
 *   Bodies     – full Jolt body table: motion type, layer, friction, restitution,
 *                global wake/freeze/zero-vel, global friction/restitution override
 *   Collision  – Jolt DrawBodies flag toggles, shape colour mode selector,
 *                collision-layer matrix, per-body AABB table, active-body bar
 *   Simulation – pause / single-step / resume, sim-speed slider, gravity presets,
 *                gravity direction drag, solver stats, scene reset
 *   Engine     – camera speed/sensitivity/pitch/invert, air density, window info
 *   Render     – wireframe toggle, light direction, brightness, light strength
 *   Perf       – colour-coded FPS, frame-time line graph, FPS histogram
 *   Log        – in-app console (intercepts std::cout/cerr, filterable, clearable)
 *
 * Toggle: Alt+D
 * Single-step while paused (menu closed): Space
 *
 * Integration changes needed in window.cpp
 * ─────────────────────────────────────────
 *  1. Replace the `drawSettings` local variable with the exported reference:
 *       const JPH::BodyManager::DrawSettings& drawSettings = DebugUI_GetDrawSettings();
 *     (remove the old local JPH::BodyManager::DrawSettings drawSettings block)
 *
 *  2. Gate Physics_Update on pause/step/speed:
 *       if (!DebugUI_GetSimPaused())
 *           Physics_Update(deltaTime * DebugUI_GetSimSpeed());
 *       else if (DebugUI_ConsumeStep())
 *           Physics_Update(1.0f / 60.0f);
 *
 * Integration changes needed in debugui.h
 * ────────────────────────────────────────
 *  Add these declarations (below the existing ones):
 *
 *   #include <Jolt/Physics/Body/BodyManager.h>
 *   bool        DebugUI_GetSimPaused();
 *   float       DebugUI_GetSimSpeed();
 *   bool        DebugUI_ConsumeStep();
 *   const JPH::BodyManager::DrawSettings& DebugUI_GetDrawSettings();
 */

#include "debugui.h"

// ── ImGui ─────────────────────────────────────────────────────────────────────
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

// ── Project ───────────────────────────────────────────────────────────────────
#include "engine.h"
#include "mecowa.h"
#include "jolt_init.h"
#include "jolt_bridge.h"
#include "jolt_layers.h"

// ── Jolt ──────────────────────────────────────────────────────────────────────
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyManager.h>

// ── GLM ───────────────────────────────────────────────────────────────────────
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/quaternion.hpp>

// ── OpenGL / GLFW ─────────────────────────────────────────────────────────────
#include <glfw/include/GLFW/glfw3.h>
#include <glad/include/glad/glad.h>

// ── Standard ──────────────────────────────────────────────────────────────────
#include <string>
#include <sstream>
#include <streambuf>
#include <deque>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <mutex>


// ═════════════════════════════════════════════════════════════════════════════
//  In-app log: redirects std::cout (white) and std::cerr (red) into the UI
// ═════════════════════════════════════════════════════════════════════════════
namespace AppLog {

struct Entry { std::string text; ImVec4 col; };

static std::deque<Entry>   lines;
static std::mutex          mtx;
bool                       autoScroll = true;
static constexpr int       kMaxLines  = 512;

static void Push(const std::string& s, ImVec4 col)
{
    std::lock_guard<std::mutex> lk(mtx);
    std::istringstream ss(s);
    std::string ln;
    while (std::getline(ss, ln)) {
        if (ln.empty()) continue;
        lines.push_back({ln, col});
        if ((int)lines.size() > kMaxLines) lines.pop_front();
    }
}

void Clear() { std::lock_guard<std::mutex> lk(mtx); lines.clear(); }

// Stream buffer that funnels a stream into Push()
class LogBuf : public std::streambuf {
    std::string acc_;
    ImVec4      col_;
public:
    explicit LogBuf(ImVec4 c) : col_(c) {}
protected:
    int overflow(int c) override {
        if (c != EOF) {
            acc_ += (char)c;
            if (c == '\n') { Push(acc_, col_); acc_.clear(); }
        }
        return c;
    }
};

static LogBuf*         coutBuf = nullptr;
static LogBuf*         cerrBuf = nullptr;
static std::streambuf* oldCout = nullptr;
static std::streambuf* oldCerr = nullptr;

void Install()
{
    coutBuf = new LogBuf({0.88f,0.90f,0.95f,1.f});   // white-ish
    cerrBuf = new LogBuf({1.00f,0.40f,0.40f,1.f});   // red
    oldCout = std::cout.rdbuf(coutBuf);
    oldCerr = std::cerr.rdbuf(cerrBuf);
}

void Uninstall()
{
    if (oldCout) std::cout.rdbuf(oldCout);
    if (oldCerr) std::cerr.rdbuf(oldCerr);
    delete coutBuf; coutBuf = nullptr;
    delete cerrBuf; cerrBuf = nullptr;
}

} // namespace AppLog


// ═════════════════════════════════════════════════════════════════════════════
//  Module state
// ═════════════════════════════════════════════════════════════════════════════
namespace {

bool  g_open      = false;
float g_prevTime  = 0.0f;
float g_deltaTime = 0.0f;

std::deque<float> g_frameHistory;
constexpr int     kHistorySize = 120;

// Renderer
bool  g_wireframe     = false;
float g_lightDir[3]   = { -0.5f, -1.0f, -0.3f };
float g_brightness    = 1.0f;
float g_lightStrength = 1.0f;

// Collision draw settings – exported to window.cpp
JPH::BodyManager::DrawSettings g_drawSettings;

// Simulation controls
bool  g_simPaused  = false;
bool  g_stepQueued = false;
float g_simSpeed   = 1.0f;
float g_gravVec[3] = { 0.f, -9.81f, 0.f };


// ═════════════════════════════════════════════════════════════════════════════
//  Style
// ═════════════════════════════════════════════════════════════════════════════
void ApplyMecoStyle()
{
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding    = 6.f; s.FrameRounding  = 4.f;
    s.ScrollbarRounding = 4.f; s.GrabRounding   = 4.f;
    s.FramePadding  = {6,4};   s.ItemSpacing    = {8,5};

    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg]         = {0.08f,0.09f,0.12f,0.92f};
    c[ImGuiCol_TitleBg]          = {0.10f,0.14f,0.22f,1.00f};
    c[ImGuiCol_TitleBgActive]    = {0.14f,0.20f,0.35f,1.00f};
    c[ImGuiCol_Header]           = {0.18f,0.28f,0.48f,0.80f};
    c[ImGuiCol_HeaderHovered]    = {0.22f,0.36f,0.60f,0.90f};
    c[ImGuiCol_HeaderActive]     = {0.26f,0.44f,0.72f,1.00f};
    c[ImGuiCol_FrameBg]          = {0.14f,0.16f,0.22f,1.00f};
    c[ImGuiCol_FrameBgHovered]   = {0.20f,0.24f,0.34f,1.00f};
    c[ImGuiCol_FrameBgActive]    = {0.26f,0.32f,0.46f,1.00f};
    c[ImGuiCol_SliderGrab]       = {0.34f,0.56f,0.90f,1.00f};
    c[ImGuiCol_SliderGrabActive] = {0.44f,0.70f,1.00f,1.00f};
    c[ImGuiCol_Button]           = {0.20f,0.30f,0.50f,0.80f};
    c[ImGuiCol_ButtonHovered]    = {0.26f,0.40f,0.66f,1.00f};
    c[ImGuiCol_ButtonActive]     = {0.32f,0.50f,0.82f,1.00f};
    c[ImGuiCol_CheckMark]        = {0.50f,0.80f,1.00f,1.00f};
    c[ImGuiCol_Text]             = {0.88f,0.90f,0.95f,1.00f};
    c[ImGuiCol_TextDisabled]     = {0.40f,0.44f,0.52f,1.00f};
    c[ImGuiCol_Separator]        = {0.22f,0.28f,0.40f,1.00f};
    c[ImGuiCol_Tab]              = {0.12f,0.18f,0.28f,1.00f};
    c[ImGuiCol_TabHovered]       = {0.24f,0.36f,0.58f,1.00f};
    c[ImGuiCol_TabActive]        = {0.20f,0.30f,0.50f,1.00f};
}


// ═════════════════════════════════════════════════════════════════════════════
//  Shared helpers
// ═════════════════════════════════════════════════════════════════════════════
bool BeginTwoColTable(const char* id, float w0 = 110.f)
{
    bool ok = ImGui::BeginTable(id, 2,
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_SizingStretchProp);
    if (ok) {
        ImGui::TableSetupColumn("Property", ImGuiTableColumnFlags_WidthFixed, w0);
        ImGui::TableSetupColumn("Value",    ImGuiTableColumnFlags_WidthStretch);
    }
    return ok;
}

PhysicsLink* FindLink(ModelInstance* inst)
{
    for (auto& lk : gPhysicsLinks)
        if (lk.model == inst) return &lk;
    return nullptr;
}

void PushToJolt(PhysicsLink& lk, const glm::vec3& pos, const glm::vec3& eulerDeg)
{
    if (!gPhysics) return;
    glm::vec3  simPos = pos + lk.renderOffset;
    glm::quat  gq     = glm::quat(glm::radians(eulerDeg));
    gPhysics->GetBodyInterface().SetPositionAndRotation(
        lk.body,
        JPH::RVec3(simPos.x, simPos.y, simPos.z),
        JPH::Quat(gq.x, gq.y, gq.z, gq.w),
        JPH::EActivation::Activate);
}

const char* MotionName(JPH::EMotionType t)
{
    switch(t){
        case JPH::EMotionType::Static:    return "Static";
        case JPH::EMotionType::Kinematic: return "Kinematic";
        case JPH::EMotionType::Dynamic:   return "Dynamic";
        default: return "?";
    }
}

ImVec4 MotionColor(JPH::EMotionType t)
{
    switch(t){
        case JPH::EMotionType::Static:    return {0.50f,0.50f,0.55f,1.f};
        case JPH::EMotionType::Kinematic: return {0.30f,0.70f,1.00f,1.f};
        case JPH::EMotionType::Dynamic:   return {0.40f,1.00f,0.50f,1.f};
        default: return {1.f,1.f,1.f,1.f};
    }
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Scene Objects
// ═════════════════════════════════════════════════════════════════════════════
void PanelSceneObjects()
{
    if (sceneModels.empty()) { ImGui::TextDisabled("No scene objects."); return; }

    int idx = 0;
    for (auto& obj : sceneModels)
    {
        ModelInstance& inst = obj.instance;
        PhysicsLink*   lk   = FindLink(&inst);

        char lbl[128];
        snprintf(lbl, sizeof(lbl), "[%d] %s%s",
                 idx++, obj.name.c_str(), lk ? "  [phys]" : "");

        if (!ImGui::TreeNodeEx(lbl, ImGuiTreeNodeFlags_SpanFullWidth)) continue;

        if (BeginTwoColTable("##sc"))
        {
            // Position
            {
                glm::vec3 ep = inst.position;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Position");
                ImGui::TableSetColumnIndex(1);
                ImGui::PushID("sp"); ImGui::SetNextItemWidth(-1);
                if (ImGui::DragFloat3("##p",&ep.x,0.01f,-1e4f,1e4f,"%.3f"))
                { if (lk) PushToJolt(*lk,ep,inst.rotation); else inst.position=ep; }
                ImGui::PopID();
            }
            // Rotation
            {
                glm::vec3 er = inst.rotation;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Rotation");
                ImGui::TableSetColumnIndex(1);
                ImGui::PushID("sr"); ImGui::SetNextItemWidth(-1);
                if (ImGui::DragFloat3("##r",&er.x,0.1f,-360.f,360.f,"%.2f"))
                { if (lk) PushToJolt(*lk,inst.position,er); else inst.rotation=er; }
                ImGui::PopID();
            }
            // Scale
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Scale");
                ImGui::TableSetColumnIndex(1);
                ImGui::PushID("ss"); ImGui::SetNextItemWidth(-1);
                ImGui::DragFloat3("##s",&inst.scale.x,0.005f,0.001f,100.f,"%.3f");
                ImGui::PopID();
            }
            // Info
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("Vertices");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%d", inst.vertexCount);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("Triangles");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%d", inst.indiciesCount);
            ImGui::EndTable();
        }
        if (lk) ImGui::TextDisabled("Body ID: %u", lk->body.GetIndex());
        ImGui::TreePop();
    }
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Physics  (per-link live data + impulse)
// ═════════════════════════════════════════════════════════════════════════════
void PanelPhysics()
{
    if (!gPhysics) { ImGui::TextColored({1,.4f,.4f,1},"Jolt not initialised."); return; }

    ImGui::Text("Bodies: %u total | %u active",
        gPhysics->GetNumBodies(),
        gPhysics->GetNumActiveBodies(JPH::EBodyType::RigidBody));
    ImGui::Separator();

    if (gPhysicsLinks.empty()) { ImGui::TextDisabled("No physics links."); return; }

    JPH::BodyInterface& bi = gPhysics->GetBodyInterface();
    int idx = 0;
    for (auto& lk : gPhysicsLinks)
    {
        const char* name = "body";
        for (auto& obj : sceneModels)
            if (&obj.instance == lk.model) { name = obj.name.c_str(); break; }

        char lbl[128];
        snprintf(lbl,sizeof(lbl),"[%d] %s  (ID %u)",idx++,name,lk.body.GetIndex());
        if (!ImGui::TreeNodeEx(lbl,ImGuiTreeNodeFlags_SpanFullWidth)) continue;

        bool active = bi.IsActive(lk.body);
        ImGui::PushStyleColor(ImGuiCol_Text,
            active ? ImVec4{.4f,1.f,.5f,1.f} : ImVec4{.5f,.5f,.55f,1.f});
        ImGui::Text(active ? "● Active" : "○ Sleeping");
        ImGui::PopStyleColor();

        JPH::Vec3 jP = bi.GetPosition(lk.body);
        JPH::Quat jR = bi.GetRotation(lk.body);
        JPH::Vec3 lv = bi.GetLinearVelocity(lk.body);
        JPH::Vec3 av = bi.GetAngularVelocity(lk.body);

        glm::vec3 euler = glm::degrees(glm::eulerAngles(
            glm::quat(jR.GetW(),jR.GetX(),jR.GetY(),jR.GetZ())));
        glm::vec3 rPos  = glm::vec3(jP.GetX(),jP.GetY(),jP.GetZ()) - lk.renderOffset;

        if (BeginTwoColTable("##ph"))
        {
            // Editable position
            {
                glm::vec3 ep = rPos;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Position");
                ImGui::TableSetColumnIndex(1);
                ImGui::PushID("jp"); ImGui::SetNextItemWidth(-1);
                if (ImGui::DragFloat3("##jp",&ep.x,0.01f,-1e4f,1e4f,"%.3f"))
                    PushToJolt(lk,ep,euler);
                ImGui::PopID();
            }
            // Editable rotation
            {
                glm::vec3 er = euler;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Rotation");
                ImGui::TableSetColumnIndex(1);
                ImGui::PushID("jr"); ImGui::SetNextItemWidth(-1);
                if (ImGui::DragFloat3("##jr",&er.x,0.1f,-360.f,360.f,"%.2f"))
                    PushToJolt(lk,rPos,er);
                ImGui::PopID();
            }
            // Editable linear velocity
            {
                float v3[3]={lv.GetX(),lv.GetY(),lv.GetZ()};
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Lin vel");
                ImGui::TableSetColumnIndex(1);
                ImGui::PushID("lv"); ImGui::SetNextItemWidth(-1);
                if (ImGui::DragFloat3("##lv",v3,0.1f,-500.f,500.f,"%.2f"))
                    bi.SetLinearVelocity(lk.body,JPH::Vec3(v3[0],v3[1],v3[2]));
                ImGui::PopID();
            }
            // Editable angular velocity
            {
                float v3[3]={av.GetX(),av.GetY(),av.GetZ()};
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Ang vel");
                ImGui::TableSetColumnIndex(1);
                ImGui::PushID("av"); ImGui::SetNextItemWidth(-1);
                if (ImGui::DragFloat3("##av",v3,0.1f,-500.f,500.f,"%.2f"))
                    bi.SetAngularVelocity(lk.body,JPH::Vec3(v3[0],v3[1],v3[2]));
                ImGui::PopID();
            }
            float spd = std::sqrt(lv.GetX()*lv.GetX()+lv.GetY()*lv.GetY()+lv.GetZ()*lv.GetZ());
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("Speed");
            ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f m/s", spd);
            ImGui::EndTable();
        }

        // Impulse applicator
        if (ImGui::TreeNode("Apply Impulse##imp"))
        {
            static float imp[3] = {0.f,5.f,0.f};
            ImGui::SetNextItemWidth(220);
            ImGui::DragFloat3("N·s", imp, 0.1f, -1000.f, 1000.f, "%.2f");
            if (ImGui::Button("Apply at COM"))
            { bi.AddImpulse(lk.body,JPH::Vec3(imp[0],imp[1],imp[2])); bi.ActivateBody(lk.body); }
            ImGui::SameLine();
            if (ImGui::Button("Launch up"))
            { bi.AddImpulse(lk.body,JPH::Vec3(0.f,std::abs(imp[1]),0.f)); bi.ActivateBody(lk.body); }
            ImGui::TreePop();
        }

        if (ImGui::SmallButton("Wake"))   bi.ActivateBody(lk.body);
        ImGui::SameLine();
        if (ImGui::SmallButton("Freeze")) bi.DeactivateBody(lk.body);
        ImGui::SameLine();
        if (ImGui::SmallButton("Zero vel"))
        { bi.SetLinearVelocity(lk.body,JPH::Vec3::sZero());
          bi.SetAngularVelocity(lk.body,JPH::Vec3::sZero()); }

        ImGui::TreePop();
    }
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Bodies – full table + bulk actions
// ═════════════════════════════════════════════════════════════════════════════
void PanelBodies()
{
    if (!gPhysics) { ImGui::TextColored({1,.4f,.4f,1},"Jolt not initialised."); return; }

    JPH::BodyInterface& bi = gPhysics->GetBodyInterface();

    ImGui::TextDisabled("%zu registered bodies", gPhysicsLinks.size());

    static char filter[64] = {};
    ImGui::SameLine(0,16); ImGui::SetNextItemWidth(160);
    ImGui::InputText("Filter##bf", filter, sizeof(filter));
    ImGui::Separator();

    if (ImGui::BeginTable("##bt", 6,
        ImGuiTableFlags_BordersOuter|ImGuiTableFlags_BordersInnerV|
        ImGuiTableFlags_RowBg|ImGuiTableFlags_ScrollY|ImGuiTableFlags_Resizable,
        ImVec2(0,250)))
    {
        ImGui::TableSetupScrollFreeze(0,1);
        ImGui::TableSetupColumn("ID",      ImGuiTableColumnFlags_WidthFixed, 34.f);
        ImGui::TableSetupColumn("Name",    ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Type",    ImGuiTableColumnFlags_WidthFixed, 74.f);
        ImGui::TableSetupColumn("Layer",   ImGuiTableColumnFlags_WidthFixed, 40.f);
        ImGui::TableSetupColumn("Frict.",  ImGuiTableColumnFlags_WidthFixed, 54.f);
        ImGui::TableSetupColumn("Rest.",   ImGuiTableColumnFlags_WidthFixed, 54.f);
        ImGui::TableHeadersRow();

        for (auto& lk : gPhysicsLinks)
        {
            const char* name = "body";
            for (auto& obj : sceneModels)
                if (&obj.instance == lk.model) { name=obj.name.c_str(); break; }

            if (filter[0] && std::string(name).find(filter)==std::string::npos) continue;

            JPH::EMotionType mt = bi.GetMotionType(lk.body);
            JPH::ObjectLayer ol = bi.GetObjectLayer(lk.body);
            float fr=0.f, re=0.f;
            {
                JPH::BodyLockRead lock(gPhysics->GetBodyLockInterface(), lk.body);
                if (lock.Succeeded())
                { fr=lock.GetBody().GetFriction(); re=lock.GetBody().GetRestitution(); }
            }

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::Text("%u",lk.body.GetIndex());
            ImGui::TableSetColumnIndex(1);
            ImGui::PushStyleColor(ImGuiCol_Text,MotionColor(mt));
            ImGui::TextUnformatted(name);
            ImGui::PopStyleColor();
            ImGui::TableSetColumnIndex(2);
            ImGui::PushStyleColor(ImGuiCol_Text,MotionColor(mt));
            ImGui::TextUnformatted(MotionName(mt));
            ImGui::PopStyleColor();
            ImGui::TableSetColumnIndex(3); ImGui::Text("%u",(unsigned)ol);
            ImGui::TableSetColumnIndex(4); ImGui::Text("%.2f",fr);
            ImGui::TableSetColumnIndex(5); ImGui::Text("%.2f",re);
        }
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Bulk Actions");
    if (ImGui::Button("Wake All"))
        for (auto& lk:gPhysicsLinks) bi.ActivateBody(lk.body);
    ImGui::SameLine();
    if (ImGui::Button("Freeze All"))
        for (auto& lk:gPhysicsLinks) bi.DeactivateBody(lk.body);
    ImGui::SameLine();
    if (ImGui::Button("Zero All Vel"))
        for (auto& lk:gPhysicsLinks)
        { bi.SetLinearVelocity(lk.body,JPH::Vec3::sZero());
          bi.SetAngularVelocity(lk.body,JPH::Vec3::sZero()); }

    ImGui::Spacing();
    ImGui::SeparatorText("Global Property Override");
    static float gFr=0.5f, gRe=0.1f;
    bool ch = ImGui::SliderFloat("Friction##g",&gFr,0.f,1.f);
    ch |=      ImGui::SliderFloat("Restitution##g",&gRe,0.f,1.f);
    if (ch)
        for (auto& lk:gPhysicsLinks)
        {
            JPH::BodyLockWrite lock(gPhysics->GetBodyLockInterface(),lk.body);
            if (lock.Succeeded())
            { lock.GetBody().SetFriction(gFr); lock.GetBody().SetRestitution(gRe); }
        }
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Collision
// ═════════════════════════════════════════════════════════════════════════════
void PanelCollision()
{
    if (!gPhysics) { ImGui::TextColored({1,.4f,.4f,1},"Jolt not initialised."); return; }

    // ── Live active-body bar ───────────────────────────────────────────────────
    unsigned active = gPhysics->GetNumActiveBodies(JPH::EBodyType::RigidBody);
    unsigned total  = gPhysics->GetNumBodies();

    ImGui::SeparatorText("Live Stats");
    {
        float frac = total ? (float)active/(float)total : 0.f;
        char ovl[48]; snprintf(ovl,sizeof(ovl),"%u / %u bodies active",active,total);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram,ImVec4{.3f,.7f,.4f,1.f});
        ImGui::ProgressBar(frac,ImVec2(-1,0),ovl);
        ImGui::PopStyleColor();
    }

    // ── DrawBodies toggles ─────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::SeparatorText("Debug Draw Flags  (F3 = toggle overlay)");
    ImGui::Columns(2,"##df",false);

    ImGui::Checkbox("Shape",              &g_drawSettings.mDrawShape);
    ImGui::Checkbox("Shape wireframe",    &g_drawSettings.mDrawShapeWireframe);
    ImGui::Checkbox("Bounding box",       &g_drawSettings.mDrawBoundingBox);
    ImGui::Checkbox("Centre of mass",     &g_drawSettings.mDrawCenterOfMassTransform);
    ImGui::Checkbox("World transform",    &g_drawSettings.mDrawWorldTransform);

    ImGui::NextColumn();

    ImGui::Checkbox("Velocity",           &g_drawSettings.mDrawVelocity);
    ImGui::Checkbox("Mass & inertia",     &g_drawSettings.mDrawMassAndInertia);
    ImGui::Checkbox("Sleep stats",        &g_drawSettings.mDrawSleepStats);
    ImGui::Checkbox("Shape colour",       &g_drawSettings.mDrawShapeColor);

    ImGui::Columns(1);

    // Shape colour mode combo
    ImGui::Spacing();
    static const char* colorModes[] = {
        "Instance","Shape type","Motion type","Sleep","Island","Material"
    };
    int cm = (int)g_drawSettings.mDrawShapeColor;
    if (ImGui::BeginCombo("Shape colour mode", colorModes[cm]))
    {
        for (int i=0;i<6;++i)
            if (ImGui::Selectable(colorModes[i],cm==i))
                g_drawSettings.mDrawShapeColor=(JPH::BodyManager::EShapeColor)i;
        ImGui::EndCombo();
    }

    // ── Layer matrix ───────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::SeparatorText("Collision Layer Matrix");
    ImGui::TextDisabled("NON_MOVING=0   MOVING=1   (defined in jolt_init.cpp)");

    if (ImGui::BeginTable("##lm",3,
        ImGuiTableFlags_BordersOuter|ImGuiTableFlags_BordersInner|ImGuiTableFlags_RowBg,
        ImVec2(260,0)))
    {
        ImGui::TableSetupColumn("",       ImGuiTableColumnFlags_WidthFixed,80.f);
        ImGui::TableSetupColumn("Lyr 0",  ImGuiTableColumnFlags_WidthFixed,72.f);
        ImGui::TableSetupColumn("Lyr 1",  ImGuiTableColumnFlags_WidthFixed,72.f);
        ImGui::TableHeadersRow();

        auto Cell=[](bool collide){
            collide ? ImGui::TextColored({.3f,1.f,.4f,1.f},"  YES")
                    : ImGui::TextColored({.4f,.4f,.5f,1.f},"  —");
        };

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Layer 0");
        ImGui::TableSetColumnIndex(1); Cell(false);   // 0 vs 0
        ImGui::TableSetColumnIndex(2); Cell(true);    // 0 vs 1

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Layer 1");
        ImGui::TableSetColumnIndex(1); Cell(true);    // 1 vs 0
        ImGui::TableSetColumnIndex(2); Cell(true);    // 1 vs 1

        ImGui::EndTable();
    }

    // ── Per-body AABB ──────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::SeparatorText("Per-body World AABB");

    if (ImGui::BeginTable("##aabb",3,
        ImGuiTableFlags_BordersOuter|ImGuiTableFlags_BordersInnerV|
        ImGuiTableFlags_RowBg|ImGuiTableFlags_ScrollY, ImVec2(0,130)))
    {
        ImGui::TableSetupScrollFreeze(0,1);
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Min",  ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Max",  ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        for (auto& lk:gPhysicsLinks)
        {
            const char* nm="body";
            for (auto& obj:sceneModels) if(&obj.instance==lk.model){nm=obj.name.c_str();break;}

            JPH::AABox bb;
            {
                JPH::BodyLockRead lock(gPhysics->GetBodyLockInterface(),lk.body);
                if (!lock.Succeeded()) continue;
                bb = lock.GetBody().GetWorldSpaceBounds();
            }
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(nm);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.1f %.1f %.1f",bb.mMin.GetX(),bb.mMin.GetY(),bb.mMin.GetZ());
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.1f %.1f %.1f",bb.mMax.GetX(),bb.mMax.GetY(),bb.mMax.GetZ());
        }
        ImGui::EndTable();
    }
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Simulation
// ═════════════════════════════════════════════════════════════════════════════
void PanelSimulation()
{
    // ── Playback ──────────────────────────────────────────────────────────────
    ImGui::SeparatorText("Playback");

    if (g_simPaused)
    {
        ImGui::PushStyleColor(ImGuiCol_Button,{.5f,.2f,.2f,1.f});
        if (ImGui::Button("▶  Resume")) g_simPaused=false;
        ImGui::PopStyleColor();
        ImGui::SameLine();
        if (ImGui::Button("⏭  Step once")) g_stepQueued=true;
        ImGui::SameLine();
        ImGui::TextDisabled("(Space also steps)");
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button,{.2f,.45f,.2f,1.f});
        if (ImGui::Button("⏸  Pause")) g_simPaused=true;
        ImGui::PopStyleColor();
    }

    ImGui::Spacing();
    ImGui::SliderFloat("Sim speed", &g_simSpeed, 0.01f, 4.f, "%.2f×");
    if (ImGui::Button("1×")) g_simSpeed=1.f;
    ImGui::SameLine(); if (ImGui::Button("½×")) g_simSpeed=0.5f;
    ImGui::SameLine(); if (ImGui::Button("2×")) g_simSpeed=2.f;

    // ── Gravity ───────────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::SeparatorText("Gravity Vector");

    bool gc = ImGui::DragFloat3("X / Y / Z", g_gravVec, 0.05f, -50.f, 50.f, "%.3f");

    if (ImGui::SmallButton("Earth"))
        { g_gravVec[0]=0;g_gravVec[1]=-9.81f;g_gravVec[2]=0; gc=true; }
    ImGui::SameLine();
    if (ImGui::SmallButton("Moon"))
        { g_gravVec[0]=0;g_gravVec[1]=-1.62f;g_gravVec[2]=0; gc=true; }
    ImGui::SameLine();
    if (ImGui::SmallButton("Mars"))
        { g_gravVec[0]=0;g_gravVec[1]=-3.72f;g_gravVec[2]=0; gc=true; }
    ImGui::SameLine();
    if (ImGui::SmallButton("Zero-G"))
        { g_gravVec[0]=0;g_gravVec[1]=0;g_gravVec[2]=0; gc=true; }
    ImGui::SameLine();
    if (ImGui::SmallButton("Flip Y"))
        { g_gravVec[1]=-g_gravVec[1]; gc=true; }

    if (gc && gPhysics)
        gPhysics->SetGravity(JPH::Vec3(g_gravVec[0],g_gravVec[1],g_gravVec[2]));

    // ── Solver info ────────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::SeparatorText("Solver  (read-only)");
    if (gPhysics)
    {
        const JPH::PhysicsSettings& ps = gPhysics->GetPhysicsSettings();
        ImGui::Text("Velocity steps : %u", ps.mNumVelocitySteps);
        ImGui::Text("Position steps : %u", ps.mNumPositionSteps);
        ImGui::TextDisabled("Edit via gPhysics->SetPhysicsSettings()");
    }

    // ── Scene reset ────────────────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::SeparatorText("Quick Reset");
    ImGui::PushStyleColor(ImGuiCol_Button,{.5f,.1f,.1f,1.f});
    if (ImGui::Button("Zero all velocities") && gPhysics)
    {
        JPH::BodyInterface& bi = gPhysics->GetBodyInterface();
        for (auto& lk:gPhysicsLinks)
        { bi.SetLinearVelocity(lk.body,JPH::Vec3::sZero());
          bi.SetAngularVelocity(lk.body,JPH::Vec3::sZero()); }
    }
    ImGui::PopStyleColor();
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Engine / Camera
// ═════════════════════════════════════════════════════════════════════════════
void PanelEngineSettings()
{
    ImGui::SeparatorText("Camera");
    ImGui::SliderFloat("Speed",       &cameraSpeed,       0.1f, 50.f);
    ImGui::SliderFloat("Sensitivity", &cameraSensitivity, 0.01f, 1.f);
    ImGui::SliderFloat("Pitch clamp", &cameraPitchClamp,  10.f, 89.f);
    ImGui::Checkbox("Invert Mouse Y", &invertMouseY);
    ImGui::Checkbox("Invert Mouse X", &invertMouseX);

    ImGui::Spacing();
    ImGui::SeparatorText("Environment");
    ImGui::DragFloat("Air density",&airDensity,0.001f,0.f,10.f,"%.4f kg/m³");

    ImGui::Spacing();
    ImGui::SeparatorText("Window");
    ImGui::Text("Resolution : %d × %d", windowWidth, windowHeight);
    ImGui::Text("Version    : %s",       version.c_str());
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Renderer
// ═════════════════════════════════════════════════════════════════════════════
void PanelRenderer()
{
    ImGui::SeparatorText("Geometry");
    if (ImGui::Checkbox("Wireframe",&g_wireframe))
        glPolygonMode(GL_FRONT_AND_BACK, g_wireframe ? GL_LINE : GL_FILL);

    ImGui::Spacing();
    ImGui::SeparatorText("Lighting");
    ImGui::SliderFloat3("Light dir",   g_lightDir,    -1.f, 1.f);
    ImGui::SliderFloat("Brightness",  &g_brightness,   0.f, 5.f);
    ImGui::SliderFloat("Light str.",  &g_lightStrength, 0.f, 5.f);

    ImGui::Spacing();
    ImGui::TextDisabled("Uniforms to set each frame:");
    ImGui::TextDisabled("  shader.setVec3(\"lightDir\", {%.2f,%.2f,%.2f})",
        g_lightDir[0],g_lightDir[1],g_lightDir[2]);
    ImGui::TextDisabled("  shader.setFloat(\"brightness\",    %.3f)", g_brightness);
    ImGui::TextDisabled("  shader.setFloat(\"lightStrength\", %.3f)", g_lightStrength);
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Performance
// ═════════════════════════════════════════════════════════════════════════════
void PanelPerformance()
{
    float fps = g_deltaTime>0.f ? 1.f/g_deltaTime : 0.f;

    ImVec4 fc = fps>55.f ? ImVec4{.3f,1.f,.4f,1.f}
              : fps>30.f ? ImVec4{1.f,.9f,.2f,1.f}
                         : ImVec4{1.f,.3f,.3f,1.f};
    ImGui::PushStyleColor(ImGuiCol_Text,fc);
    ImGui::Text("FPS:  %.1f", fps);
    ImGui::PopStyleColor();
    ImGui::Text("Frame time:  %.2f ms", g_deltaTime*1000.f);

    // Rolling history
    g_frameHistory.push_back(g_deltaTime*1000.f);
    while ((int)g_frameHistory.size()>kHistorySize) g_frameHistory.pop_front();

    static float ftBuf[kHistorySize]={};
    static float fpsBuf[kHistorySize]={};
    int n=(int)g_frameHistory.size();
    for(int i=0;i<n;++i){ ftBuf[i]=g_frameHistory[i]; fpsBuf[i]=(ftBuf[i]>0.f?1000.f/ftBuf[i]:0.f); }

    float mxFt=*std::max_element(ftBuf,ftBuf+std::max(n,1));
    mxFt=std::max(mxFt,33.3f);
    char ovl[32]; snprintf(ovl,sizeof(ovl),"%.1f ms",g_deltaTime*1000.f);

    ImGui::SeparatorText("Frame time (ms)");
    ImGui::PlotLines("##ft",ftBuf,n,0,ovl,0.f,mxFt,{0.f,55.f});

    ImGui::SeparatorText("FPS");
    ImGui::PlotHistogram("##fps",fpsBuf,n,0,nullptr,0.f,200.f,{0.f,40.f});

    ImGui::Spacing();
    ImGui::SeparatorText("Jolt");
    if(gPhysics){
        ImGui::Text("Active bodies : %u",
            gPhysics->GetNumActiveBodies(JPH::EBodyType::RigidBody));
        ImGui::Text("Total bodies  : %u", gPhysics->GetNumBodies());
    } else ImGui::TextDisabled("Jolt not running.");
}


// ═════════════════════════════════════════════════════════════════════════════
//  Tab: Log console
// ═════════════════════════════════════════════════════════════════════════════
void PanelLog()
{
    if (ImGui::SmallButton("Clear")) AppLog::Clear();
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll",&AppLog::autoScroll);

    static char logFilt[64]={};
    ImGui::SameLine(); ImGui::SetNextItemWidth(160);
    ImGui::InputText("Filter##lf",logFilt,sizeof(logFilt));
    ImGui::Separator();

    ImGui::BeginChild("##ls",ImVec2(0,0),false,ImGuiWindowFlags_HorizontalScrollbar);
    {
        std::lock_guard<std::mutex> guard(AppLog::mtx);
        std::string flt(logFilt);
        for(auto& e:AppLog::lines){
            if(!flt.empty()&&e.text.find(flt)==std::string::npos) continue;
            ImGui::PushStyleColor(ImGuiCol_Text,e.col);
            ImGui::TextUnformatted(e.text.c_str());
            ImGui::PopStyleColor();
        }
    }
    if(AppLog::autoScroll && ImGui::GetScrollY()>=ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.f);
    ImGui::EndChild();
}

} // anonymous namespace


// ═════════════════════════════════════════════════════════════════════════════
//  Public API
// ═════════════════════════════════════════════════════════════════════════════

void DebugUI_Init(GLFWwindow* window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename  = nullptr;
    ApplyMecoStyle();
    ImGui_ImplGlfw_InitForOpenGL(window,true);
    ImGui_ImplOpenGL3_Init("#version 330");

    AppLog::Install();   // redirect cout/cerr into the Log tab

    // Sensible draw-settings defaults
    g_drawSettings.mDrawShape          = true;
    g_drawSettings.mDrawShapeWireframe = true;
    g_drawSettings.mDrawBoundingBox    = false;
    g_drawSettings.mDrawVelocity       = false;
    g_drawSettings.mDrawMassAndInertia = false;
    g_drawSettings.mDrawSleepStats     = false;
    g_drawSettings.mDrawShapeColor     = JPH::BodyManager::EShapeColor::MotionTypeColor;

    // Sync gravity display
    g_gravVec[0]=0.f; g_gravVec[1]=gravityG; g_gravVec[2]=0.f;
}

void DebugUI_Render()
{
    float now=static_cast<float>(glfwGetTime());
    g_deltaTime=now-g_prevTime; g_prevTime=now;

    if (!g_open) return;

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSize({600,700},ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos ({20,20}, ImGuiCond_FirstUseEver);

    if (ImGui::Begin("MecoWA  —  Debug  (Alt+D)",&g_open,ImGuiWindowFlags_NoCollapse))
    {
        if (g_simPaused)
        {
            ImGui::PushStyleColor(ImGuiCol_Text,ImVec4{1.f,.8f,.2f,1.f});
            ImGui::TextUnformatted("  ⏸  SIMULATION PAUSED");
            ImGui::PopStyleColor();
            ImGui::Separator();
        }

        if (ImGui::BeginTabBar("##tabs"))
        {
            if (ImGui::BeginTabItem("Scene"))      { PanelSceneObjects();   ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Physics"))    { PanelPhysics();        ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Bodies"))     { PanelBodies();         ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Collision"))  { PanelCollision();      ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Simulation")) { PanelSimulation();     ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Engine"))     { PanelEngineSettings(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Render"))     { PanelRenderer();       ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Perf"))       { PanelPerformance();    ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Log"))        { PanelLog();            ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
    }
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool DebugUI_HandleKey(int key, int action, int mods)
{
    if (key==GLFW_KEY_D && action==GLFW_PRESS && (mods&GLFW_MOD_ALT))
        { g_open=!g_open; return true; }
    // Space = single step (menu may be closed)
    if (key==GLFW_KEY_SPACE && action==GLFW_PRESS && g_simPaused)
        { g_stepQueued=true; return true; }
    return false;
}

void DebugUI_Shutdown()
{
    AppLog::Uninstall();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

// Accessors
const float*                          DebugUI_GetLightDir()      { return g_lightDir; }
float                                 DebugUI_GetBrightness()    { return g_brightness; }
float                                 DebugUI_GetLightStrength() { return g_lightStrength; }
bool                                  DebugUI_GetSimPaused()     { return g_simPaused; }
float                                 DebugUI_GetSimSpeed()      { return g_simSpeed; }
bool                                  DebugUI_ConsumeStep()      { bool v=g_stepQueued; g_stepQueued=false; return v; }
const JPH::BodyManager::DrawSettings& DebugUI_GetDrawSettings()  { return g_drawSettings; }
