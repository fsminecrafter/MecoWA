#pragma once

#include <string>
#include <vector>
#include <functional>

// Forward declarations to avoid heavy includes in header
struct GLFWwindow;

// ─────────────────────────────────────────────────────────────────────────────
//  Bottom Menu Item - Modular UI Component
// ─────────────────────────────────────────────────────────────────────────────
struct BottomMenuCategory
{
    std::string name;
    std::vector<std::string> items;
    bool isExpanded = false;

    BottomMenuCategory(const std::string& categoryName)
        : name(categoryName) {}

    void AddItem(const std::string& itemName)
    {
        items.push_back(itemName);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────
void DebugUI_Init(GLFWwindow* window);
void DebugUI_Render(float deltaTime);
bool DebugUI_HandleKey(int key, int action, int mods);
void DebugUI_Shutdown();

const float* DebugUI_GetLightDir();
float DebugUI_GetBrightness();
float DebugUI_GetLightStrength();
float DebugUI_GetTimeScale();
void  DebugUI_SetTimeScale(float scale);
void  DebugUI_LoadAndApplyScene(const char* path);

// ── Collider debug overlay ────────────────────────────────────────────────────
// Returns true when the collider wireframe overlay should be drawn this frame.
bool DebugUI_GetShowColliderOverlay();

// ─────────────────────────────────────────────────────────────────────────────
//  Bottom Menu API - Modular interface
// ─────────────────────────────────────────────────────────────────────────────
void DebugUI_AddBottomMenuCategory(const std::string& categoryName);
void DebugUI_AddItemToCategory(const std::string& categoryName, const std::string& itemName);
void DebugUI_SetBottomMenuCallback(const std::string& categoryName, const std::string& itemName,
                                    std::function<void()> callback);
