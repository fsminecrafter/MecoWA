#pragma once
#include <glfw/include/GLFW/glfw3.h>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>
#include "engine.h"
#include "mecowa.h"

class CameraController {
private:
    Camera* camera = nullptr;
    bool firstMouse = true;
    double lastX = 0.0, lastY = 0.0;

public:
    CameraController(Camera& cam) : camera(&cam) {}

    void Update(GLFWwindow* window, float deltaTime) {
        if (!camera) return;

        // Calculate camera direction vectors
        glm::vec3 right;
        right.x = cos(glm::radians(camera->rotation.y)) * cos(glm::radians(camera->rotation.x));
        right.y = sin(glm::radians(camera->rotation.x));
        right.z = sin(glm::radians(camera->rotation.y)) * cos(glm::radians(camera->rotation.x));
        right = glm::normalize(right);

        glm::vec3 front = glm::normalize(glm::cross(right, glm::vec3(0.0f, 1.0f, 0.0f)));
        glm::vec3 up = glm::normalize(glm::cross(front, right));

        float velocity = cameraSpeed * deltaTime;

        // Move along view axes
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera->position -= front * velocity;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera->position += front * velocity;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera->position -= right * velocity;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera->position += right * velocity;

        // Optional: vertical movement along world Y
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) // move down
            camera->position -= glm::vec3(0.0f, 1.0f, 0.0f) * velocity;
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) // move up
            camera->position += glm::vec3(0.0f, 1.0f, 0.0f) * velocity;

        // Mouse rotation (left button)
        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);

            if (firstMouse) {
                lastX = xpos;
                lastY = ypos;
                firstMouse = false;
            }

            double xoffset = xpos - lastX;
            double yoffset = lastY - ypos; // Y reversed
            lastX = xpos;
            lastY = ypos;

            if (invertMouseX) xoffset = -xoffset;
            if (invertMouseY) yoffset = -yoffset;

            xoffset *= cameraSensitivity;
            yoffset *= cameraSensitivity;

            camera->rotation.y += (float)xoffset;
            camera->rotation.x += (float)yoffset;

            // Clamp pitch
            if (camera->rotation.x > cameraPitchClamp)
                camera->rotation.x = cameraPitchClamp;
            if (camera->rotation.x < -cameraPitchClamp)
                camera->rotation.x = -cameraPitchClamp;
        }
        else {
            firstMouse = true;
        }
    }
};
