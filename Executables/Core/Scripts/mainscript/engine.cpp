#include "engine.h"
#include <iostream>
#include <vector>
#include "objloader.h"

std::vector<ModelInstance> sceneModels;

//#define DEBUG

ModelInstance CreateModelInstance(const OBJData& model, glm::vec3 pos, glm::vec3 rot) {
    ModelInstance instance;
    instance.model = model;
    instance.position = pos;
    instance.rotation = rot;
    instance.scale = glm::vec3(1.0f);
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

glm::mat4 getModelMatrix(const ModelInstance& instance) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, instance.position);
    model = glm::rotate(model, glm::radians(instance.rotation.x), glm::vec3(1, 0, 0));
    model = glm::rotate(model, glm::radians(instance.rotation.y), glm::vec3(0, 1, 0));
    model = glm::rotate(model, glm::radians(instance.rotation.z), glm::vec3(0, 0, 1));
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

Camera CreateCamera(glm::vec3 pos, glm::vec3 rot, float fov) {
    Camera cam;
    cam.position = pos;
    cam.rotation = rot;
    cam.fov = fov;
    return cam;
}

glm::mat4 GetViewMatrix(const Camera& camera) {
    // Start with identity
    glm::mat4 view = glm::mat4(1.0f);

    // Apply rotations (pitch, yaw, roll)
    view = glm::rotate(view, glm::radians(camera.rotation.x), glm::vec3(1, 0, 0)); // pitch
    view = glm::rotate(view, glm::radians(camera.rotation.y), glm::vec3(0, 1, 0)); // yaw
    view = glm::rotate(view, glm::radians(camera.rotation.z), glm::vec3(0, 0, 1)); // roll

    // Apply translation (move opposite of camera position)
    view = glm::translate(view, -camera.position);
    return view;
}

void Transform(ModelInstance& instance, const glm::vec3& rotation, const glm::vec3& position) {
    instance.rotation = rotation;
    instance.position = position;
}

void rotateObject(ModelInstance& instance, const glm::vec3& rotation) {
    instance.rotation += rotation;
}

void moveObject(ModelInstance& instance, const glm::vec3& position) {
    instance.position += position;
}

ModelInstance& CreateObject(const std::string& path, OBJData& objDataVar,
    const glm::vec3& pos, const glm::vec3& rot, const glm::vec3& scale)
{
    if (!loadOBJ(path, objDataVar)) {
        std::cerr << "[Scene] Failed to load .obj: " << path << std::endl;
        static ModelInstance dummy;
        return dummy;
    }

    ModelInstance instance = CreateModelInstance(objDataVar, pos, rot);
    instance.scale = scale;

    // CPU data
    size_t vertexCount = objDataVar.vertexCoords.size() / 3;
    instance.vertexData.reserve(vertexCount * 9);

    // Combine vertex, color, normal
    for (size_t i = 0; i < vertexCount; ++i) {
        glm::vec3 posV(
            objDataVar.vertexCoords[i * 3 + 0],
            objDataVar.vertexCoords[i * 3 + 1],
            objDataVar.vertexCoords[i * 3 + 2]);

        glm::vec3 colV(1.0f);
        if (objDataVar.vertexColors.size() >= (i + 1) * 3)
            colV = glm::vec3(
                objDataVar.vertexColors[i * 3 + 0],
                objDataVar.vertexColors[i * 3 + 1],
                objDataVar.vertexColors[i * 3 + 2]);

        glm::vec3 normV(0.0f);
        if (objDataVar.vertexNormals.size() >= (i + 1) * 3)
            normV = glm::vec3(
                objDataVar.vertexNormals[i * 3 + 0],
                objDataVar.vertexNormals[i * 3 + 1],
                objDataVar.vertexNormals[i * 3 + 2]);

        instance.vertexData.insert(instance.vertexData.end(),
            { posV.x, posV.y, posV.z,
              colV.x, colV.y, colV.z,
              normV.x, normV.y, normV.z });
    }

    instance.elementData.assign(objDataVar.elementArray.begin(), objDataVar.elementArray.end());

    // Upload to GPU
    glGenVertexArrays(1, &instance.VAO);
    glGenBuffers(1, &instance.VBO);
    glGenBuffers(1, &instance.EBO);

    glBindVertexArray(instance.VAO);
    glBindBuffer(GL_ARRAY_BUFFER, instance.VBO);
    glBufferData(GL_ARRAY_BUFFER, instance.vertexData.size() * sizeof(float), instance.vertexData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, instance.EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, instance.elementData.size() * sizeof(unsigned int), instance.elementData.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    sceneModels.push_back(instance);

    std::cout << "[Scene] Added object from " << path
        << " | Pos(" << pos.x << "," << pos.y << "," << pos.z << ")"
        << " Rot(" << rot.x << "," << rot.y << "," << rot.z << ")"
        << " Scale(" << scale.x << "," << scale.y << "," << scale.z << ")\n";

    return sceneModels.back();
}



void RenderModels(Shader& shader) {
    for (auto& instance : sceneModels) {
        glm::mat4 modelMatrix = ComputeModelMatrix(instance);
        shader.setMat4("model", modelMatrix);
        glBindVertexArray(instance.VAO);
        glDrawElements(GL_TRIANGLES, (GLsizei)instance.elementData.size(), GL_UNSIGNED_INT, 0);
    }
}