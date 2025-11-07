#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "objloader.h"
#include "shaderloader.h"
#include <glad/glad.h>

struct ModelInstance {
    OBJData model;
    glm::vec3 position;
    glm::vec3 rotation;
    glm::vec3 scale; // Add this
};


ModelInstance CreateModelInstance(const OBJData& model, glm::vec3 pos, glm::vec3 rot);
glm::mat4 ComputeModelMatrix(const ModelInstance& instance);
void DrawModel(const ModelInstance& instance, Shader& shader, GLuint VAO);
