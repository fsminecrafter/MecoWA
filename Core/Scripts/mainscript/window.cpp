#include <glad/include/glad/glad.h>
#include <glfw/include/GLFW/glfw3.h>
#include "debugui.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_internal.h"
#include "imgui/imconfig.h"
#include <iostream>
#include <windows.h>
#include <cstdio>
#include <cmath>

#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include <glm/glm/gtc/type_ptr.hpp>

#include "shaderloader.h"
#include "objloader.h"
#include "engine.h"
#include "mecowa.h"
#include "physics_tick.h"
#include "engine_camera.h"
#include "material_registry.h"
#include "jolt_init.h"
#include "jolt_world.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Renderer/DebugRenderer.h>

#include <sstream>
#include <string.h>

#ifdef _DEBUG
#include "debugrender.h"
#endif

bool debugmode = true;

extern JPH::PhysicsSystem* gPhysics;
using namespace std;

// Global gravity (in G's, 1G = 9.81 m/s²)
float gravityG = -1.0f;
float airDensity = 1.225f;     // kg/m3 (Earth, sea level)

int windowWidth = 640;
int windowHeight = 480;
std::string version = "0.073";
bool joltinitsuccess = true;

void errorpopup(int code)
{
    char buffer[256];
    sprintf_s(buffer, sizeof(buffer), "An error has occurred: %d\n", code);
    MessageBox(NULL, buffer, "Error", MB_OK | MB_ICONERROR);
}

void glfwprinterror()
{
    const char* desc = nullptr;
    int error = glfwGetError(&desc);
    printf("GLFW Error Code: %d : %s\n", error, desc ? desc : "NULL");
    errorpopup(error);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    windowWidth = width;
    windowHeight = height;
    glViewport(0, 0, width, height);
}

// ── Key callback ─────────────────────────────────────────────────────────────
// Alt+D  → toggle ImGui debug overlay
// F3     → toggle Jolt wireframe debug draw (existing debugmode)
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // Let ImGui handle Alt+D first; if it consumed the event, stop here
    if (DebugUI_HandleKey(key, action, mods))
        return;

    if (key == GLFW_KEY_F3 && action == GLFW_PRESS)
        debugmode = !debugmode;

    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

int main(void)
{
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hConsole, 10);
    printf("Starting MecoWA. Mechanical simulator.\n");
    SetConsoleTextAttribute(hConsole, 15);
    printf("By Joel_minecrafter\n");
    printf("GLFW Version: %s\n", glfwGetVersionString());

    // GLFW setup
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_WIN32);
    glfwInitHint(GLFW_JOYSTICK_HAT_BUTTONS, GLFW_FALSE);

    Jolt_Init();

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        glfwprinterror();
        return -1;
    }

    if (joltinitsuccess == false) {
        std::cerr << "Failed to initialize Jolt Physics\n";
        return -1;
    }

    // Request an OpenGL 3.3 core context (required by ImGui's GLSL #version 330)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    std::string windowTitle = "MecoWA v" + version + " | By Joel_minecrafter | XLABS INC";
    GLFWwindow* window = glfwCreateWindow(windowWidth, windowHeight, windowTitle.c_str(), NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window\n";
        glfwprinterror();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, key_callback);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        glfwTerminate();
        return -1;
    }

    // ── Dear ImGui ────────────────────────────────────────────────────────────
    DebugUI_Init(window);

    // ── Jolt debug renderer ───────────────────────────────────────────────────
    bool g_DebugPhysics = true;

    OpenGLDebugRenderer* g_DebugRenderer = nullptr;
    if (g_DebugPhysics)
        g_DebugRenderer = new OpenGLDebugRenderer();

    JPH::BodyManager::DrawSettings drawSettings;
    drawSettings.mDrawShape          = true;
    drawSettings.mDrawBoundingBox    = false;
    drawSettings.mDrawWorldTransform = false;
    drawSettings.mDrawVelocity       = false;
    drawSettings.mDrawMassAndInertia = false;

    glEnable(GL_DEPTH_TEST);

    Shader shader(
        R"(Core\Resources\vertex.glsl)",
        R"(Core\Resources\fragment.glsl)"
    );

    Material Metal("Metal", 7800.0f, 0.3f, 0.2f, 0.8f);
    MaterialRegistry::Register(Metal);

    // Create camera
    static Camera camera = CreateCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f), 45.0f);
    CameraController camCtrl(camera);
    glfwSetWindowUserPointer(window, &camera);

    // ── Scene setup ───────────────────────────────────────────────────────────
    DebugUI_LoadAndApplyScene("Core/Data/scene.scn");

    // ── Main loop ─────────────────────────────────────────────────────────────
    float lastFrame = 0.0f;

    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = (float)glfwGetTime();
        float deltaTime    = currentFrame - lastFrame;
        lastFrame          = currentFrame;

        glfwPollEvents();

        PhysicsTick_Accumulate(deltaTime, DebugUI_GetTimeScale());


        // ── Render ────────────────────────────────────────────────────────────
        glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 view = GetViewMatrix(camera);
        glm::mat4 projection = glm::perspective(
            glm::radians(camera.fov),
            (float)windowWidth / (float)windowHeight,
            0.1f, 100.0f);

        // Feed light values from the ImGui Renderer tab back into the shader
        const float* ld = DebugUI_GetLightDir();
        shader.use();
        shader.setMat4 ("view",         view);
        shader.setMat4 ("projection",   projection);
        shader.setVec3 ("lightDir",     glm::normalize(glm::vec3(ld[0], ld[1], ld[2])));
        shader.setFloat("lightStrength",DebugUI_GetLightStrength());
        shader.setFloat("brightness",   DebugUI_GetBrightness());
        shader.setVec3 ("cameraPos",    camera.position);

        glEnable(GL_DEPTH_TEST);
        RenderModels(shader);

        // ── Jolt wireframe debug draw (F3) ────────────────────────────────────
        if (debugmode)
        {
            glDisable(GL_DEPTH_TEST);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

            glUseProgram(0);
            glBindVertexArray(0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            if (g_DebugPhysics && g_DebugRenderer && gPhysics)
            {
                g_DebugRenderer->SetCameraPosition(
                    camera.position.x, camera.position.y, camera.position.z);
                g_DebugRenderer->SetViewProjection(view, projection);
                gPhysics->DrawBodies(drawSettings, g_DebugRenderer, nullptr);
            }

            glBindVertexArray(0);
            glUseProgram(shader.ID);

            // Restore fill mode (ImGui needs it)
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_DEPTH_TEST);
        }

        // ── Dear ImGui overlay (Alt+D) ────────────────────────────────────────
        DebugUI_Render(deltaTime);

        glfwSwapBuffers(window);
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    DebugUI_Shutdown();

    delete g_DebugRenderer;
    g_DebugRenderer = nullptr;

    glfwTerminate();
    return 0;
}
