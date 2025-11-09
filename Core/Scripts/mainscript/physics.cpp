#define GLM_ENABLE_EXPERIMENTAL
#include "physics.h"
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>
#include <iomanip>
#include "engine_camera.h"
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtx/intersect.hpp>

std::vector<PhysicalModel> physicalModels;

#define Debug

// Global gravity (in G's, 1G = 9.81 m/s²)
static float gravityG = 1.0f;

// Registers a model with physics simulation
void RegisterPhysicalModel(ModelInstance& instance, const Material& mat) {
    float vertexCount = static_cast<float>(instance.model.vertexCoords.size()) / 3.0f;
    float baseMass = vertexCount * 0.0001f;  // arbitrary tuning factor

    PhysicalEntity phys;
    phys.instance = instance;
    phys.mass = baseMass * mat.density;
    phys.material = mat;
    phys.velocity = glm::vec3(0.0f);
    phys.angularVelocity = glm::vec3(0.0f);

    physicalModels.push_back({ &instance, phys });

#ifdef Debug
    std::cout << std::fixed << std::setprecision(4)
        << "[Physics] Registered model with mass " << phys.mass << " kg"
        << " | Vertices: " << vertexCount
        << " | Density: " << mat.density << std::endl;
#endif
}

// Updates the physics simulation
void UpdatePhysics(float deltaTime) {
    for (auto& obj : physicalModels) {
        if (!obj.instance || obj.physics.isStatic)
            continue;

        glm::vec3 gravity(0.0f, -9.81f * gravityG, 0.0f);

        // Apply gravity
        obj.physics.velocity += gravity * deltaTime;

        // Integrate motion
        obj.instance->position += obj.physics.velocity * deltaTime;
        obj.instance->rotation += obj.physics.angularVelocity * deltaTime;

        // Basic floor collision
        if (obj.instance->position.y < 0.0f) {
            obj.instance->position.y = 0.0f;
            obj.physics.velocity.y *= -0.3f; // damp bounce

#ifdef Debug
            std::cout << "[Physics] Collision detected: object hit ground at "
                << obj.instance->position.y << std::endl;
#endif
        }

#ifdef Debug
        std::cout << std::fixed << std::setprecision(4)
            << "[Physics] Object @ "
            << "Pos(" << obj.instance->position.x << ", "
            << obj.instance->position.y << ", "
            << obj.instance->position.z << ") | "
            << "Vel(" << obj.physics.velocity.x << ", "
            << obj.physics.velocity.y << ", "
            << obj.physics.velocity.z << ")" << std::endl;
#endif
    }
}

// Prints all active physics states for debugging
void PrintPhysicsState() {
#ifdef Debug
    std::cout << "====== Physics Debug Info ======" << std::endl;
    int i = 0;
    for (auto& obj : physicalModels) {
        if (!obj.instance) continue;

        std::cout << "Object[" << i++ << "] "
            << "Mass: " << obj.physics.mass << " kg "
            << "Pos: (" << obj.instance->position.x << ", "
            << obj.instance->position.y << ", "
            << obj.instance->position.z << ") "
            << "Vel: (" << obj.physics.velocity.x << ", "
            << obj.physics.velocity.y << ", "
            << obj.physics.velocity.z << ")" << std::endl;
    }
    std::cout << "================================" << std::endl;
#endif
}

static bool dragging = false;
static PhysicalModel* draggedModel = nullptr;
static glm::vec3 grabLocalOffset;
static glm::vec3 grabWorldPoint;

// Helper: perform a ray-triangle intersection
bool RayIntersectsTriangle(const glm::vec3& rayOrigin,
    const glm::vec3& rayDir,
    const glm::vec3& v0,
    const glm::vec3& v1,
    const glm::vec3& v2,
    float& distance) {
    glm::vec2 baryPos;
    return glm::intersectRayTriangle(rayOrigin, rayDir, v0, v1, v2, baryPos, distance);
}

// Perform raycast against all scene models
PhysicalModel* RaycastModels(const glm::vec3& rayOrigin,
    const glm::vec3& rayDir,
    glm::vec3& outHitPoint) {
    float nearest = FLT_MAX;
    PhysicalModel* result = nullptr;

    for (auto& obj : physicalModels) {
        auto& model = obj.physics.instance.model;
        const auto& verts = model.vertexCoords;
        const auto& indices = model.elementArray;

        glm::mat4 modelMat = ComputeModelMatrix(obj.physics.instance);

        for (size_t i = 0; i < indices.size(); i += 3) {
            glm::vec3 v0(verts[indices[i] * 3 + 0], verts[indices[i] * 3 + 1], verts[indices[i] * 3 + 2]);
            glm::vec3 v1(verts[indices[i + 1] * 3 + 0], verts[indices[i + 1] * 3 + 1], verts[indices[i + 1] * 3 + 2]);
            glm::vec3 v2(verts[indices[i + 2] * 3 + 0], verts[indices[i + 2] * 3 + 1], verts[indices[i + 2] * 3 + 2]);

            v0 = glm::vec3(modelMat * glm::vec4(v0, 1.0f));
            v1 = glm::vec3(modelMat * glm::vec4(v1, 1.0f));
            v2 = glm::vec3(modelMat * glm::vec4(v2, 1.0f));

            float dist;
            if (RayIntersectsTriangle(rayOrigin, rayDir, v0, v1, v2, dist)) {
                if (dist < nearest) {
                    nearest = dist;
                    outHitPoint = rayOrigin + rayDir * dist;
                    result = &obj;
                }
            }
        }
    }

    return result;
}

void OnRightClickPressed(const Camera& cam, double mouseX, double mouseY, int windowWidthz, int windowHeighty) {
    if (dragging) return;

    // Compute ray from camera
    glm::vec2 ndc = {
        (2.0f * mouseX) / windowWidthz - 1.0f,
        1.0f - (2.0f * mouseY) / windowHeighty
    };

    glm::mat4 projection = glm::perspective(glm::radians(cam.fov), (float)windowWidthz / windowHeighty, 0.1f, 1000.0f);
    glm::mat4 view = GetViewMatrix(cam);
    glm::mat4 invVP = glm::inverse(projection * view);

    glm::vec4 rayStartNDC(ndc.x, ndc.y, -1.0f, 1.0f);
    glm::vec4 rayEndNDC(ndc.x, ndc.y, 0.0f, 1.0f);

    glm::vec4 rayStartWorld = invVP * rayStartNDC; rayStartWorld /= rayStartWorld.w;
    glm::vec4 rayEndWorld = invVP * rayEndNDC; rayEndWorld /= rayEndWorld.w;

    glm::vec3 rayDir = glm::normalize(glm::vec3(rayEndWorld - rayStartWorld));
    glm::vec3 rayOrigin = glm::vec3(rayStartWorld);

    glm::vec3 hit;
    PhysicalModel* hitModel = RaycastModels(rayOrigin, rayDir, hit);

    if (hitModel) {
        draggedModel = hitModel;
        grabWorldPoint = hit;
        grabLocalOffset = hit - hitModel->instance->position;
        dragging = true;

#ifdef Debug
        std::cout << "[Physics] Grabbed object at "
            << "Pos(" << hitModel->instance->position.x << ", "
            << hitModel->instance->position.y << ", "
            << hitModel->instance->position.z << ") "
            << " | Hit (" << hit.x << ", " << hit.y << ", " << hit.z << ")\n";
#endif
    }
}

void OnRightClickReleased() {
    dragging = false;
    draggedModel = nullptr;
}

// Update dragging constraint
void UpdateDrag(const Camera& cam, double mouseX, double mouseY, int windowWidthz, int windowHeighty) {
    if (!dragging || !draggedModel) return;

    glm::vec2 ndc = {
        (2.0f * mouseX) / windowWidthz - 1.0f,
        1.0f - (2.0f * mouseY) / windowHeighty
    };

    glm::mat4 projection = glm::perspective(glm::radians(cam.fov), (float)windowWidthz / windowHeighty, 0.1f, 1000.0f);
    glm::mat4 view = GetViewMatrix(cam);
    glm::mat4 invVP = glm::inverse(projection * view);

    glm::vec4 rayStartNDC(ndc.x, ndc.y, -1.0f, 1.0f);
    glm::vec4 rayEndNDC(ndc.x, ndc.y, 0.0f, 1.0f);

    glm::vec4 rayStartWorld = invVP * rayStartNDC; rayStartWorld /= rayStartWorld.w;
    glm::vec4 rayEndWorld = invVP * rayEndNDC; rayEndWorld /= rayEndWorld.w;

    glm::vec3 rayDir = glm::normalize(glm::vec3(rayEndWorld - rayStartWorld));
    glm::vec3 rayOrigin = glm::vec3(rayStartWorld);

    // Move the object so the grab point follows the ray direction
    float grabDist = glm::length(grabWorldPoint - cam.position);
    glm::vec3 newGrabPos = rayOrigin + rayDir * grabDist;
    glm::vec3 newObjectPos = newGrabPos - grabLocalOffset;

    draggedModel->instance->position = glm::mix(draggedModel->instance->position, newObjectPos, 0.2f);
}