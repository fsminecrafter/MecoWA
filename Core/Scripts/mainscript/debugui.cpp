#include "debugui.h"

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <glfw/include/GLFW/glfw3.h>

void DebugUI_RenderSceneEditor();

// ─────────────────────────────────────────────────────────────────────────────
//  Module state
// ─────────────────────────────────────────────────────────────────────────────
static bool  g_showDebugUI      = false;
static float g_lightDir[3]      = { -0.5f, -1.0f, -0.3f };
static float g_brightness       = 1.0f;
static float g_lightStrength    = 1.0f;

// Collider debug overlay settings
static bool  g_showColliders    = false;
static float g_colliderColor[4] = { 0.0f, 1.0f, 0.2f, 0.6f }; // RGBA
static bool  g_colliderWireframe= true;
static bool  g_showAABB         = false;
static float g_aabbColor[4]     = { 0.9f, 0.6f, 0.0f, 0.4f };

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

                ImGui::Spacing();
                ImGui::SeparatorText("Collider Debug Overlay");

                ImGui::Checkbox("Show Physics Colliders", &g_showColliders);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Draws Jolt physics body shapes as a wireframe/solid overlay on top of meshes.\nToggle with this checkbox or press F3 for the full Jolt draw.");

                if (g_showColliders)
                {
                    ImGui::Indent(16.f);

                    ImGui::Checkbox("Wireframe only##cw", &g_colliderWireframe);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Wireframe = outlines only.  Unchecked = semi-transparent solid fill.");

                    ImGui::ColorEdit4("Collider color##cc", g_colliderColor,
                        ImGuiColorEditFlags_AlphaBar | ImGuiColorEditFlags_AlphaPreview);

                    ImGui::Spacing();
                    ImGui::Checkbox("Show AABB##ca", &g_showAABB);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Also draw each body's axis-aligned bounding box.");

                    if (g_showAABB)
                    {
                        ImGui::ColorEdit4("AABB color##ac", g_aabbColor,
                            ImGuiColorEditFlags_AlphaBar | ImGuiColorEditFlags_AlphaPreview);
                    }

                    ImGui::Unindent(16.f);
                }

                ImGui::Spacing();
                ImGui::SeparatorText("Tips");
                ImGui::TextDisabled("F3          - toggle full Jolt debug draw");
                ImGui::TextDisabled("Alt+D       - toggle this overlay");

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

const float* DebugUI_GetLightDir()         { return g_lightDir; }
float        DebugUI_GetBrightness()       { return g_brightness; }
float        DebugUI_GetLightStrength()    { return g_lightStrength; }

bool         DebugUI_ShowColliders()       { return g_showColliders; }
bool         DebugUI_CollidersWireframe()  { return g_colliderWireframe; }
const float* DebugUI_GetColliderColor()    { return g_colliderColor; }
bool         DebugUI_ShowAABB()            { return g_showAABB; }
const float* DebugUI_GetAABBColor()        { return g_aabbColor; }
