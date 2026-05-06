#include "debugui.h"

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <glfw/include/GLFW/glfw3.h>

// Forward declaration — implemented in debugui_scenefile.cpp
void DebugUI_RenderSceneEditor();

// ─────────────────────────────────────────────────────────────────────────────
//  Module state
// ─────────────────────────────────────────────────────────────────────────────
static bool  g_showDebugUI = false;
static float g_lightDir[3] = { -0.5f, -1.0f, -0.3f };
static float g_brightness = 1.0f;
static float g_lightStrength = 1.0f;

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

void DebugUI_Render()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

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