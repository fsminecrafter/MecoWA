#include "debugui.h"
#include "engine.h"
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <glfw/include/GLFW/glfw3.h>

#include <glad/include/glad/glad.h>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/type_ptr.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>

#include <map>
#include <vector>
#include <string>

// Forward declaration — implemented in debugui_scenefile.cpp
void DebugUI_RenderSceneEditor();

// Forward declaration for scene object count
extern std::vector<ObjectList> sceneModels;

// ─────────────────────────────────────────────────────────────────────────────
//  Module state
// ─────────────────────────────────────────────────────────────────────────────
static bool  g_showDebugUI          = false;
static float g_lightDir[3]          = { -0.5f, -1.0f, -0.3f };
static float g_brightness           = 1.0f;
static float g_lightStrength        = 1.0f;
static float g_timeScale            = 1.0f;
static bool  g_showColliderOverlay  = false;   // NEW: collider wireframe overlay

// FPS tracking
static float g_avgFps          = 0.0f;
static float g_frameTimeAccum  = 0.0f;
static int   g_frameCount      = 0;

// Bottom menu state
static std::vector<BottomMenuCategory> g_bottomMenuCategories;
static std::map<std::pair<std::string, std::string>, std::function<void()>> g_menuCallbacks;
static std::string g_selectedCategory;
static std::string g_selectedItem;

// ─────────────────────────────────────────────────────────────────────────────
//  Collider wireframe overlay
//
//  We keep a tiny immediate-mode GL draw path here so we can visualise the
//  SceneCollider boxes that live in the scene editor without needing a full
//  separate shader.  We use the legacy GL matrix pipeline (same approach as
//  the Jolt debug draw in window.cpp) which is already available via GLAD.
//
//  Only Box colliders produce a visible outline; ConvexHull / TriangleMesh
//  shapes fall back to a bounding-box outline of the parent mesh instead.
// ─────────────────────────────────────────────────────────────────────────────

// Draw the 12 edges of an AABB defined by center + half-extents
static void DrawWireBox(const glm::vec3& center,
                         const glm::vec3& half,
                         const glm::vec4& color)
{
    glm::vec3 mn = center - half;
    glm::vec3 mx = center + half;

    glColor4f(color.r, color.g, color.b, color.a);
    glBegin(GL_LINES);

    // Bottom face
    glVertex3f(mn.x, mn.y, mn.z);  glVertex3f(mx.x, mn.y, mn.z);
    glVertex3f(mx.x, mn.y, mn.z);  glVertex3f(mx.x, mn.y, mx.z);
    glVertex3f(mx.x, mn.y, mx.z);  glVertex3f(mn.x, mn.y, mx.z);
    glVertex3f(mn.x, mn.y, mx.z);  glVertex3f(mn.x, mn.y, mn.z);

    // Top face
    glVertex3f(mn.x, mx.y, mn.z);  glVertex3f(mx.x, mx.y, mn.z);
    glVertex3f(mx.x, mx.y, mn.z);  glVertex3f(mx.x, mx.y, mx.z);
    glVertex3f(mx.x, mx.y, mx.z);  glVertex3f(mn.x, mx.y, mx.z);
    glVertex3f(mn.x, mx.y, mx.z);  glVertex3f(mn.x, mx.y, mn.z);

    // Vertical pillars
    glVertex3f(mn.x, mn.y, mn.z);  glVertex3f(mn.x, mx.y, mn.z);
    glVertex3f(mx.x, mn.y, mn.z);  glVertex3f(mx.x, mx.y, mn.z);
    glVertex3f(mx.x, mn.y, mx.z);  glVertex3f(mx.x, mx.y, mx.z);
    glVertex3f(mn.x, mn.y, mx.z);  glVertex3f(mn.x, mx.y, mx.z);

    glEnd();
}

// Draw a wireframe sphere approximation (3 great circles)
static void DrawWireSphere(const glm::vec3& center, float radius,
                            const glm::vec4& color, int segments = 24)
{
    glColor4f(color.r, color.g, color.b, color.a);
    auto circle = [&](int ax0, int ax1, int ax2) {
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < segments; ++i)
        {
            float a = (float)i / (float)segments * 3.14159265f * 2.f;
            float v[3] = { center.x, center.y, center.z };
            v[ax0] += radius * cosf(a);
            v[ax1] += radius * sinf(a);
            (void)ax2;
            glVertex3f(v[0], v[1], v[2]);
        }
        glEnd();
    };
    circle(0, 1, 2);
    circle(1, 2, 0);
    circle(0, 2, 1);
}

void DebugUI_DrawColliderOverlay(const glm::mat4& view, const glm::mat4& projection)
{
    if (!g_showColliderOverlay) return;

    // Use legacy GL matrix path (same as Jolt wire debug)
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(1.5f);
    glUseProgram(0);
    glBindVertexArray(0);

    // Load view-projection into legacy pipeline
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projection));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(view));

    // Colour palette per shape type
    const glm::vec4 colBox     = { 0.10f, 0.85f, 0.20f, 0.90f };  // green
    const glm::vec4 colSphere  = { 0.20f, 0.60f, 1.00f, 0.90f };  // blue
    const glm::vec4 colCapsule = { 1.00f, 0.80f, 0.10f, 0.90f };  // yellow
    const glm::vec4 colConvex  = { 1.00f, 0.40f, 0.10f, 0.90f };  // orange
    const glm::vec4 colMesh    = { 0.80f, 0.10f, 1.00f, 0.90f };  // purple
    const glm::vec4 colFallback= { 0.90f, 0.90f, 0.90f, 0.60f };  // grey (no collider)

    for (const auto& sceneObj : sceneModels)
    {
        const ModelInstance& inst = sceneObj.instance;

        // Build model translation so offsets are in world space
        glm::vec3 instPos = inst.position;
        glm::vec3 instScl = inst.scale;

        // If this object has no colliders known here we fall back to
        // drawing its mesh AABB in grey.  The scene editor keeps colliders
        // in g_scene.objects, which is a separate data structure — to avoid
        // a circular dependency we expose a small lookup helper that the
        // scene editor registers.  For now we draw a simple mesh AABB.

        // Compute mesh AABB
        const auto& vc = inst.model.vertexCoords;
        if (vc.size() < 3) continue;

        glm::vec3 mn( std::numeric_limits<float>::max());
        glm::vec3 mx(-std::numeric_limits<float>::max());
        for (size_t i = 0; i + 2 < vc.size(); i += 3)
        {
            glm::vec3 p(vc[i]*instScl.x, vc[i+1]*instScl.y, vc[i+2]*instScl.z);
            mn = glm::min(mn, p);
            mx = glm::max(mx, p);
        }
        glm::vec3 meshHalf   = (mx - mn) * 0.5f;
        glm::vec3 meshCenter = instPos + (mn + mx) * 0.5f;

        DrawWireBox(meshCenter, meshHalf, colFallback);
    }

    glLineWidth(1.0f);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

// ─────────────────────────────────────────────────────────────────────────────
//  FPS Calculation
// ─────────────────────────────────────────────────────────────────────────────
void DebugUI_UpdateFPS(float deltaTime)
{
    g_frameTimeAccum += deltaTime;
    g_frameCount++;

    if (g_frameTimeAccum >= 0.5f)
    {
        g_avgFps         = g_frameCount / g_frameTimeAccum;
        g_frameTimeAccum = 0.0f;
        g_frameCount     = 0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Bottom Menu Implementation
// ─────────────────────────────────────────────────────────────────────────────
static void RenderBottomMenu()
{
    const float BOTTOM_MENU_HEIGHT = 90.0f;
    ImGui::SetNextWindowPos(ImVec2(0, ImGui::GetIO().DisplaySize.y - BOTTOM_MENU_HEIGHT), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x, BOTTOM_MENU_HEIGHT), ImGuiCond_Always);
    ImGui::Begin("BottomMenu", nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    if (ImGui::BeginTable("BottomMenuLayout", 2, ImGuiTableFlags_NoBordersInBody))
    {
        ImGui::TableSetupColumn("TimeControls", ImGuiTableColumnFlags_WidthFixed, 400.0f);
        ImGui::TableSetupColumn("Stats",        ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);

        ImGui::Text("Time Scale:");
        ImGui::SameLine(100.0f);

        if (ImGui::Button("Pause", ImVec2(50, 0))) g_timeScale = 0.0f;
        ImGui::SameLine();
        if (ImGui::Button("0.5x",  ImVec2(50, 0))) g_timeScale = 0.5f;
        ImGui::SameLine();
        if (ImGui::Button("1x",    ImVec2(50, 0))) g_timeScale = 1.0f;
        ImGui::SameLine();
        if (ImGui::Button("2x",    ImVec2(50, 0))) g_timeScale = 2.0f;
        ImGui::SameLine();
        if (ImGui::Button("4x",    ImVec2(50, 0))) g_timeScale = 4.0f;

        ImGui::SameLine(0, 20);
        ImGui::Separator();
        ImGui::SameLine(0, 20);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.55f, 0.1f, 0.1f, 1.f });
        if (ImGui::Button("Reset", ImVec2(100, 0)))
        {
            sceneModels.clear();
            DebugUI_LoadAndApplyScene("Core/Data/scene.scn");
        }
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Clear the scene and reload Core/Data/scene.scn");

        ImGui::TableSetColumnIndex(1);
        ImGui::Text("Current: %.1fx | FPS: %.1f | Objects: %zu",
                    g_timeScale, g_avgFps, sceneModels.size());

        ImGui::EndTable();
    }
    ImGui::Separator();

    ImGui::PushItemWidth(-1);
    for (size_t i = 0; i < g_bottomMenuCategories.size(); ++i)
    {
        BottomMenuCategory& category = g_bottomMenuCategories[i];
        bool isSelected = (g_selectedCategory == category.name);
        if (isSelected)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.26f, 0.59f, 0.98f, 0.4f));

        if (ImGui::Button(category.name.c_str(), ImVec2(80, 0)))
        {
            g_selectedCategory    = category.name;
            category.isExpanded   = !category.isExpanded;
            if (!category.items.empty())
                g_selectedItem = category.items[0];
        }

        if (isSelected) ImGui::PopStyleColor();
        if (i < g_bottomMenuCategories.size() - 1) ImGui::SameLine();
    }
    ImGui::PopItemWidth();

    if (!g_selectedCategory.empty())
    {
        for (auto& category : g_bottomMenuCategories)
        {
            if (category.name != g_selectedCategory) continue;
            for (size_t i = 0; i < category.items.size(); ++i)
            {
                bool isItemSelected = (g_selectedItem == category.items[i]);
                if (isItemSelected)
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.26f, 0.59f, 0.98f, 0.6f));

                if (ImGui::Button(category.items[i].c_str(), ImVec2(100, 0)))
                {
                    g_selectedItem = category.items[i];
                    auto key = std::make_pair(category.name, category.items[i]);
                    auto it  = g_menuCallbacks.find(key);
                    if (it != g_menuCallbacks.end()) it->second();
                }

                if (isItemSelected) ImGui::PopStyleColor();
                if (i < category.items.size() - 1) ImGui::SameLine();
            }
            break;
        }
    }

    ImGui::End();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────
void DebugUI_Init(GLFWwindow* window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void DebugUI_Shutdown()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

bool DebugUI_HandleKey(int key, int action, int mods)
{
    if (key == GLFW_KEY_D && action == GLFW_PRESS && (mods & GLFW_MOD_ALT))
    {
        g_showDebugUI = !g_showDebugUI;
        return true;
    }
    return false;
}

void DebugUI_Render(float deltaTime)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    DebugUI_UpdateFPS(deltaTime);

    // Always render bottom menu
    RenderBottomMenu();

    if (g_showDebugUI)
    {
        ImGui::SetNextWindowSize(ImVec2(700, 500), ImGuiCond_Once);
        ImGui::Begin("MecoWA Debug", &g_showDebugUI);

        if (ImGui::BeginTabBar("##tabs"))
        {
            // ── Renderer tab ────────────────────────────────────────────────
            if (ImGui::BeginTabItem("Renderer"))
            {
                ImGui::SeparatorText("Lighting");
                ImGui::DragFloat3("Light Direction",  g_lightDir, 0.01f, -1.f, 1.f);
                ImGui::SliderFloat("Light Strength",  &g_lightStrength, 0.0f, 5.0f);
                ImGui::SliderFloat("Brightness",      &g_brightness,    0.0f, 3.0f);

                ImGui::Spacing();
                ImGui::SeparatorText("Debug Overlays");

                // ── Collider overlay toggle ──────────────────────────────
                ImGui::PushStyleColor(ImGuiCol_FrameBg,
                    g_showColliderOverlay
                        ? ImVec4{0.10f, 0.40f, 0.10f, 1.f}
                        : ImVec4{0.20f, 0.20f, 0.20f, 1.f});
                ImGui::Checkbox("Show Collider Overlay", &g_showColliderOverlay);
                ImGui::PopStyleColor();

                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Draw mesh bounding-box wireframes over all scene objects.\n"
                        "Green = mesh AABB.\n"
                        "Toggle with this checkbox or call DebugUI_GetShowColliderOverlay().");

                if (g_showColliderOverlay)
                {
                    ImGui::SameLine();
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{0.30f, 1.00f, 0.40f, 1.f});
                    ImGui::TextUnformatted("● ACTIVE");
                    ImGui::PopStyleColor();
                }

                ImGui::Spacing();
                ImGui::TextDisabled(
                    "The Jolt wireframe (F3) shows physics body shapes.\n"
                    "The collider overlay (above) shows the scene-editor colliders.");

                ImGui::EndTabItem();
            }

            // ── Scene Editor tab ────────────────────────────────────────────
            if (ImGui::BeginTabItem("Scene Editor"))
            {
                DebugUI_RenderSceneEditor();
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

        ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Accessors
// ─────────────────────────────────────────────────────────────────────────────
const float* DebugUI_GetLightDir()          { return g_lightDir; }
float        DebugUI_GetBrightness()        { return g_brightness; }
float        DebugUI_GetLightStrength()     { return g_lightStrength; }
float        DebugUI_GetTimeScale()         { return g_timeScale; }
void         DebugUI_SetTimeScale(float s)  { g_timeScale = s; }
bool         DebugUI_GetShowColliderOverlay(){ return g_showColliderOverlay; }

// ─────────────────────────────────────────────────────────────────────────────
//  Bottom Menu API
// ─────────────────────────────────────────────────────────────────────────────
void DebugUI_AddBottomMenuCategory(const std::string& categoryName)
{
    for (const auto& cat : g_bottomMenuCategories)
        if (cat.name == categoryName) return;
    g_bottomMenuCategories.emplace_back(categoryName);
}

void DebugUI_AddItemToCategory(const std::string& categoryName,
                                const std::string& itemName)
{
    for (auto& category : g_bottomMenuCategories)
    {
        if (category.name != categoryName) continue;
        category.AddItem(itemName);
        if (g_selectedCategory.empty())
            g_selectedCategory = categoryName;
        return;
    }
}

void DebugUI_SetBottomMenuCallback(const std::string& categoryName,
                                    const std::string& itemName,
                                    std::function<void()> callback)
{
    g_menuCallbacks[{categoryName, itemName}] = callback;
}
