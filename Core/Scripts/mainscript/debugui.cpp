#include "debugui.h"
#include "engine.h"
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <glfw/include/GLFW/glfw3.h>
#include <map>

// Forward declaration — implemented in debugui_scenefile.cpp
void DebugUI_RenderSceneEditor();

// Forward declaration for scene object count
extern std::vector<ObjectList> sceneModels;

// ─────────────────────────────────────────────────────────────────────────────
//  Module state
// ─────────────────────────────────────────────────────────────────────────────
static bool  g_showDebugUI = false;
static float g_lightDir[3] = { -0.5f, -1.0f, -0.3f };
static float g_brightness = 1.0f;
static float g_lightStrength = 1.0f;
static float g_timeScale = 1.0f;

// FPS tracking
static float g_avgFps = 0.0f;
static float g_frameTimeAccum = 0.0f;
static int   g_frameCount = 0;

// Bottom menu state
static std::vector<BottomMenuCategory> g_bottomMenuCategories;
static std::map<std::pair<std::string, std::string>, std::function<void()>> g_menuCallbacks;
static std::string g_selectedCategory;
static std::string g_selectedItem;

// ─────────────────────────────────────────────────────────────────────────────
//  FPS Calculation
// ─────────────────────────────────────────────────────────────────────────────
void DebugUI_UpdateFPS(float deltaTime)
{
    g_frameTimeAccum += deltaTime;
    g_frameCount++;

    // Update FPS every 0.5 seconds
    if (g_frameTimeAccum >= 0.5f)
    {
        g_avgFps = g_frameCount / g_frameTimeAccum;
        g_frameTimeAccum = 0.0f;
        g_frameCount = 0;
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
    ImGui::Begin("BottomMenu", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    // Top section: Time scale controls and stats
    if (ImGui::BeginTable("BottomMenuLayout", 2, ImGuiTableFlags_NoBordersInBody))
    {
        ImGui::TableSetupColumn("TimeControls", ImGuiTableColumnFlags_WidthFixed, 400.0f);
        ImGui::TableSetupColumn("Stats", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);

        // Time scale controls
        ImGui::Text("Time Scale:");
        ImGui::SameLine(100.0f);

        if (ImGui::Button("Pause", ImVec2(50, 0)))
            g_timeScale = 0.0f;
        ImGui::SameLine();

        if (ImGui::Button("0.5x", ImVec2(50, 0)))
            g_timeScale = 0.5f;
        ImGui::SameLine();

        if (ImGui::Button("1x", ImVec2(50, 0)))
            g_timeScale = 1.0f;
        ImGui::SameLine();

        if (ImGui::Button("2x", ImVec2(50, 0)))
            g_timeScale = 2.0f;
        ImGui::SameLine();

        if (ImGui::Button("4x", ImVec2(50, 0)))
            g_timeScale = 4.0f;

        ImGui::SameLine(0, 20);
        ImGui::Separator();
        ImGui::SameLine(0, 20);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.55f, 0.1f, 0.1f, 1.f });
        if (ImGui::Button("Reset", ImVec2(100, 0)))
        {
            // Clear and reload
            sceneModels.clear();
            DebugUI_LoadAndApplyScene("Core/Data/scene.scn");
        }
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Clear the scene and reload Core/Data/scene.scn");

        // Stats column
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("Current: %.1fx | FPS: %.1f | Objects: %zu", g_timeScale, g_avgFps, sceneModels.size());

        ImGui::SameLine(0, 20);

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.55f, 0.1f, 0.1f, 1.f });
        if (ImGui::Button("Reset Scene", ImVec2(100, 0)))
        {
            sceneModels.clear();
            DebugUI_LoadAndApplyScene("Core/Data/scene.scn");
        }
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Clear the scene and reload Core/Data/scene.scn");

        ImGui::EndTable();
    }
    ImGui::Separator();

    // Category tabs
    ImGui::PushItemWidth(-1);
    for (size_t i = 0; i < g_bottomMenuCategories.size(); ++i)
    {
        BottomMenuCategory& category = g_bottomMenuCategories[i];

        bool isSelected = (g_selectedCategory == category.name);
        if (isSelected)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.26f, 0.59f, 0.98f, 0.4f));

        if (ImGui::Button(category.name.c_str(), ImVec2(80, 0)))
        {
            g_selectedCategory = category.name;
            category.isExpanded = !category.isExpanded;
            if (!category.items.empty())
                g_selectedItem = category.items[0];
        }

        if (isSelected)
            ImGui::PopStyleColor();

        if (i < g_bottomMenuCategories.size() - 1)
            ImGui::SameLine();
    }
    ImGui::PopItemWidth();

    // Bottom row: display selected items or expanded category
    if (!g_selectedCategory.empty())
    {
        for (auto& category : g_bottomMenuCategories)
        {
            if (category.name == g_selectedCategory)
            {
                for (size_t i = 0; i < category.items.size(); ++i)
                {
                    bool isItemSelected = (g_selectedItem == category.items[i]);
                    if (isItemSelected)
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.26f, 0.59f, 0.98f, 0.6f));

                    if (ImGui::Button(category.items[i].c_str(), ImVec2(100, 0)))
                    {
                        g_selectedItem = category.items[i];

                        auto key = std::make_pair(category.name, category.items[i]);
                        if (g_menuCallbacks.find(key) != g_menuCallbacks.end())
                        {
                            g_menuCallbacks[key]();
                        }
                    }

                    if (isItemSelected)
                        ImGui::PopStyleColor();

                    if (i < category.items.size() - 1)
                        ImGui::SameLine();
                }
                break;
            }
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

    // Update FPS calculation
    DebugUI_UpdateFPS(deltaTime);

    // Always render bottom menu
    RenderBottomMenu();

    if (g_showDebugUI)
    {
        ImGui::SetNextWindowSize(ImVec2(700, 500), ImGuiCond_Once);
        ImGui::Begin("MecoWA Debug", &g_showDebugUI);

        if (ImGui::BeginTabBar("##tabs"))
        {
            if (ImGui::BeginTabItem("Renderer"))
            {
                ImGui::SeparatorText("Lighting");
                ImGui::DragFloat3("Light Direction", g_lightDir, 0.01f, -1.f, 1.f);
                ImGui::SliderFloat("Light Strength", &g_lightStrength, 0.0f, 5.0f);
                ImGui::SliderFloat("Brightness", &g_brightness, 0.0f, 3.0f);
                ImGui::EndTabItem();
            }

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

const float* DebugUI_GetLightDir() { return g_lightDir; }
float        DebugUI_GetBrightness() { return g_brightness; }
float        DebugUI_GetLightStrength() { return g_lightStrength; }
float        DebugUI_GetTimeScale() { return g_timeScale; }
void         DebugUI_SetTimeScale(float scale) { g_timeScale = scale; }

// ─────────────────────────────────────────────────────────────────────────────
//  Bottom Menu API
// ─────────────────────────────────────────────────────────────────────────────
void DebugUI_AddBottomMenuCategory(const std::string& categoryName)
{
    for (const auto& cat : g_bottomMenuCategories)
    {
        if (cat.name == categoryName)
            return; // Category already exists
    }
    g_bottomMenuCategories.emplace_back(categoryName);
}

void DebugUI_AddItemToCategory(const std::string& categoryName, const std::string& itemName)
{
    for (auto& category : g_bottomMenuCategories)
    {
        if (category.name == categoryName)
        {
            category.AddItem(itemName);
            if (g_selectedCategory.empty())
                g_selectedCategory = categoryName;
            return;
        }
    }
}

void DebugUI_SetBottomMenuCallback(const std::string& categoryName, const std::string& itemName,
                                    std::function<void()> callback)
{
    auto key = std::make_pair(categoryName, itemName);
    g_menuCallbacks[key] = callback;
}
