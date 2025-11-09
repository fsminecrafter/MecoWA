#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <windows.h>
#include <cstdio>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shaderloader.h"
#include "objloader.h"
#include "engine.h"
#include "mecowa.h"
#include "engine_camera.h"


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
    glViewport(0, 0, width, height);
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
    glfwInitHint(GLFW_ANGLE_PLATFORM_TYPE, GLFW_ANGLE_PLATFORM_TYPE_OPENGL);
    glfwInitHint(GLFW_JOYSTICK_HAT_BUTTONS, GLFW_FALSE);

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        glfwprinterror();
        return -1;
    }

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

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        glfwTerminate();
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    Shader shader(
        R"(Core\Resources\vertex.glsl)",
        R"(Core\Resources\fragment.glsl)"
    );

    // Create camera
    static Camera camera = CreateCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f), 45.0f);
    CameraController camCtrl(camera);

    // Create models using the new engine API
    OBJData monkeOBJ;
    CreateObject(R"(Core\Resources\3dmodels\monke.obj)", monkeOBJ,
        glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(1.0f));

    OBJData monke2OBJ;
    CreateObject(R"(Core\Resources\3dmodels\monke.obj)", monke2OBJ,
        glm::vec3(2.0f, 0.0f, 0.0f), glm::vec3(0.0f), glm::vec3(1.5f));

    float lastFrame = 0.0f;
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        float currentFrame = (float)glfwGetTime();
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        camCtrl.Update(window, deltaTime);
        // Animate the second model a bit
        sceneModels[1].rotation.y = (float)glfwGetTime() * -50.0f;

        glm::mat4 view = GetViewMatrix(camera);
        glm::mat4 projection = glm::perspective(glm::radians(camera.fov),
            (float)windowWidth / windowHeight,
            0.1f, 100.0f);

        shader.use();
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        shader.setVec3("lightDir", glm::normalize(glm::vec3(-0.5f, -1.0f, -0.3f)));
        shader.setFloat("lightStrength", 2.0f);
        shader.setFloat("brightness", 1.0f);
        shader.setVec3("cameraPos", camera.position);

        RenderModels(shader);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
