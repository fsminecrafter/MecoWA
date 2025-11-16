#pragma once
#include <glm/glm.hpp>
#include <vector>
#include "objloader.h"
#include "shaderloader.h"
#include <glad/glad.h>

// Represents a loaded model instance in the scene
struct ModelInstance {
    OBJData model;                // Geometry data
    glm::vec3 position = { 0.0f, 0.0f, 0.0f };
    glm::vec3 rotation = { 0.0f, 0.0f, 0.0f };
    glm::vec3 scale = { 1.0f, 1.0f, 1.0f };

	int vertexCount = 0;
	int indiciesCount = 0;

    // GPU data
    GLuint VAO = 0, VBO = 0, EBO = 0;

    // CPU buffers (for flexibility, debugging, or re-upload)
    std::vector<float> vertexData;
    std::vector<unsigned int> elementData;
};

// Camera with position, rotation, and field of view
struct Camera {
    glm::vec3 position = { 0.0f, 0.0f, 0.0f };
    glm::vec3 rotation = { 0.0f, 0.0f, 0.0f };
    float fov = 45.0f;
};

// Global scene container
extern std::vector<ModelInstance> sceneModels;

// Function declarations
ModelInstance CreateModelInstance(const OBJData& model, glm::vec3 pos, glm::vec3 rot);
ModelInstance& CreateObject(const std::string& path, OBJData& objDataVar,
    const glm::vec3& pos, const glm::vec3& rot, const glm::vec3& scale);
void RenderModels(Shader& shader);

glm::mat4 ComputeModelMatrix(const ModelInstance& instance);
glm::mat4 getModelMatrix(const ModelInstance& instance);
void DrawModel(const ModelInstance& instance, Shader& shader, GLuint VAO);
void Transform(ModelInstance& instance, const glm::vec3& rotation, const glm::vec3& position);
void rotateObject(ModelInstance& instance, const glm::vec3& rotation);
void moveObject(ModelInstance& instance, const glm::vec3& position);

Camera CreateCamera(glm::vec3 pos, glm::vec3 rot, float fov);
glm::mat4 GetViewMatrix(const Camera& camera);
