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

void errorpopup(int code)
{
    char buffer[256]; // Enough space for the message
    sprintf_s(buffer, sizeof(buffer), "An error has occurred: %d\n", code);

    MessageBox(
        NULL,           // No parent window
        buffer,         // Formatted message
        "Error",        // Window title
        MB_OK | MB_ICONERROR
    );
}

void glfwprinterror() {
    const char* desc = nullptr;
    int error = glfwGetError(&desc);
    printf("GLFW Error Code: %d : %s\n", error, desc ? desc : "NULL");
    errorpopup(error);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}


void testcolors(){
    HANDLE  hConsole;
    int k;

    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

    // you can loop k higher to see more color choices
    for (k = 1; k < 255; k++)
    {
        SetConsoleTextAttribute(hConsole, k);
        printf("Nice day for fishing, aint it! huhah");
    }
}

int main(void)
{
    HANDLE  hConsole;
    hConsole = GetStdHandle(STD_OUTPUT_HANDLE);


    SetConsoleTextAttribute(hConsole, 10);
    printf("Starting MecoWA. Mechanical simulator.\n");
    SetConsoleTextAttribute(hConsole, 15);
    printf("By Joel_minecrafter\n");
    printf("GLFW Version: ");
    printf("%s", glfwGetVersionString());
    printf("\n");

    // GLFW initialization
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_WIN32); // This init sets so that its running on windows
    glfwInitHint(GLFW_ANGLE_PLATFORM_TYPE, GLFW_ANGLE_PLATFORM_TYPE_OPENGL); // This init sets so its using opengl
    glfwInitHint(GLFW_JOYSTICK_HAT_BUTTONS, GLFW_FALSE); // and this one is saying NO CONTROLLERS!!!

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        glfwprinterror();
        return -1;
    }
    std::string windowTitle = "MecoWA v" + version + " | By Joel_minecrafter | XLABS INC";
    GLFWwindow* window = glfwCreateWindow(windowWidth, windowHeight, "MecoWA v0.02 | By Joel_minecrafter | XLABS INC", NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwprinterror();
        return -1;
    }

    glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback); //Set the viewport size callback (The function is called when the window is resized)

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return -1;
    }
    glEnable(GL_DEPTH_TEST); //enable test depth buffer

    OBJData model;
    if (!loadOBJ(R"(Core\Resources\3dmodels\monke.obj)", model)) {
        std::cerr << "Failed to load .obj model!" << std::endl;
        return -1;
    }
    ModelInstance monkey = CreateModelInstance(model, glm::vec3(0.0f), glm::vec3(0.0f));
    monkey.scale = glm::vec3(1.0f); // default scale


    float* vertexCoords = new float[model.vertexCoords.size()];
    float* vertexColors = new float[model.vertexColors.size()];
    int* elementArray = new int[model.elementArray.size()];

    std::vector<unsigned int> elementData; // indices

    size_t vertexCount = model.vertexCoords.size() / 3;
    std::vector<float> vertexData; // pos(3) + color(3) + normal(3)
    vertexData.reserve(model.vertexCoords.size() / 3 * 9); // 9 floats per vertex

    float angle = 0.0f;

    std::copy(model.vertexCoords.begin(), model.vertexCoords.end(), vertexCoords);
    std::copy(model.vertexColors.begin(), model.vertexColors.end(), vertexColors);
    std::copy(model.elementArray.begin(), model.elementArray.end(), elementArray);

    Shader shader(
        R"(Core\Resources\vertex.glsl)", //The end \r and \'s was creating problems loading the shaders
        R"(Core\Resources\fragment.glsl)"
    );


    // diagnostics
    void* proc = (void*)glfwGetProcAddress("glClear"); // fixed typo
    void* cur = (void*)glfwGetCurrentContext();
    const GLubyte* ver = glGetString(GL_VERSION);
    printf("glClear proc=%p current_ctx=%p glVersion=%s\n", proc, cur, ver ? (const char*)ver : "NULL");
    fflush(stdout);

    const char* desc = nullptr;
    int err = glfwGetError(&desc);
    printf("glfwGetError() = %d : %s\n", err, desc ? desc : "NULL");
    if (~err == NULL) {
        glfwTerminate();
        printf("An error has occoured while initilization.");
    }

    //Scary OHHHHH

for (size_t i = 0; i < vertexCount; i++) {
    // Positions
    vertexData.push_back(model.vertexCoords[i * 3 + 0]);
    vertexData.push_back(model.vertexCoords[i * 3 + 1]);
    vertexData.push_back(model.vertexCoords[i * 3 + 2]);

    // Colors
    if (model.vertexColors.size() >= (i + 1) * 3) {
        vertexData.push_back(model.vertexColors[i * 3 + 0]);
        vertexData.push_back(model.vertexColors[i * 3 + 1]);
        vertexData.push_back(model.vertexColors[i * 3 + 2]);
    } else {
        vertexData.push_back(1.0f);
        vertexData.push_back(1.0f);
        vertexData.push_back(1.0f);
    }

    // Normals
    if (model.vertexNormals.size() >= (i + 1) * 3) {
        vertexData.push_back(model.vertexNormals[i * 3 + 0]);
        vertexData.push_back(model.vertexNormals[i * 3 + 1]);
        vertexData.push_back(model.vertexNormals[i * 3 + 2]);
    } else {
        // fallback normal if missing
        vertexData.push_back(0.0f);
        vertexData.push_back(0.0f);
        vertexData.push_back(1.0f);
    }
}
    elementData.assign(model.elementArray.begin(), model.elementArray.end());

    unsigned int VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);

    // Position attribute (location = 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color attribute (location = 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Normal attribute (location = 2)
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    // Element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    std::vector<unsigned int> indices(model.elementArray.begin(), model.elementArray.end());
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);


    // In your render loop, replace the draw call with:
    glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, 0);

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Camera & matrices
        glm::mat4 view = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -3.0f));
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)windowWidth / windowHeight, 0.1f, 100.0f);

        shader.use();
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);

        shader.setVec3("lightDir", glm::normalize(glm::vec3(-0.5f, -1.0f, -0.3f)));
        shader.setFloat("lightStrength", 2.0f);
        shader.setFloat("brightness", 1.0f);
        shader.setVec3("cameraPos", glm::vec3(0.0f, 0.0f, 3.0f));

        DrawModel(monkey, shader, VAO);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }


    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    delete[] vertexCoords;
    delete[] vertexColors;
    delete[] elementArray;
    glfwTerminate();


    glfwTerminate();
    return 0;
};