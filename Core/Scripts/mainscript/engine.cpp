#include "engine.h"
#include <iostream>

//#define DEBUG

ModelInstance CreateModelInstance(const OBJData& model, glm::vec3 pos, glm::vec3 rot) {
    ModelInstance instance;
    instance.model = model;
    instance.position = pos;
    instance.rotation = rot;
    return instance;
}

glm::mat4 ComputeModelMatrix(const ModelInstance& instance) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, instance.position);
    model = glm::rotate(model, glm::radians(instance.rotation.x), glm::vec3(1, 0, 0));
    model = glm::rotate(model, glm::radians(instance.rotation.y), glm::vec3(0, 1, 0));
    model = glm::rotate(model, glm::radians(instance.rotation.z), glm::vec3(0, 0, 1));
    model = glm::scale(model, instance.scale);
    return model;
}


void DrawModel(const ModelInstance& instance, Shader& shader, GLuint VAO) {
    glm::mat4 modelMatrix = ComputeModelMatrix(instance);
    shader.setMat4("model", modelMatrix);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES,
        static_cast<GLsizei>(instance.model.elementArray.size()),
        GL_UNSIGNED_INT,
        0);

#ifdef DEBUG
    std::cout << "[DrawModel] Drew " << instance.model.elementArray.size() / 3
        << " triangles (" << instance.model.vertexCoords.size() / 3
        << " vertices)\n";
#endif
}
