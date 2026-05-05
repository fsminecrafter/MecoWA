#pragma once

// Forward declarations to avoid heavy includes in header
struct GLFWwindow;

// Call once after OpenGL + GLFW are initialized
void DebugUI_Init(GLFWwindow* window);

// Call once per frame (before glfwSwapBuffers)
void DebugUI_Render();

// Call in GLFW key callback – handles Alt+D toggle
// Returns true if ImGui consumed the event (caller should skip own handling)
bool DebugUI_HandleKey(int key, int action, int mods);

// Clean up ImGui context (call before glfwTerminate)
void DebugUI_Shutdown();
