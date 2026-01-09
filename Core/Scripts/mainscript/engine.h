#pragma once
#include <glm/glm/glm.hpp>
#include <vector>
#include "objloader.h"
#include "shaderloader.h"
#include <glad/include/glad/glad.h>
#include <Jolt/Jolt.h>

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

void Physics_SyncToEngine();
void RegisterPhysics_Box(ModelInstance& inst, const OBJData& mesh, float mass, float friction = 0.5f, float restitution = 0.1f, bool originAtBottom = true, glm::vec3 boxsize = glm::vec3(1.0f, 1.0f, 1.0f));

Camera CreateCamera(glm::vec3 pos, glm::vec3 rot, float fov);
glm::mat4 GetViewMatrix(const Camera& camera);
void PrintObjectPosition(const ModelInstance& instance, const std::string& name = "");
void PrintObjectTransform(const ModelInstance& instance, const std::string& name = "");

inline glm::vec3 FromJoltQuat(const JPH::Quat& q);
inline glm::vec3 FromJoltVec3(const JPH::Vec3& v);
inline JPH::Quat ToJoltQuat(const glm::vec3& eulerDeg);
inline JPH::Vec3 ToJoltVec3(const glm::vec3& v);
