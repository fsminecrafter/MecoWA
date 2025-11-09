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

void UpdatePhysics(float deltaTime) {
    const float airDensity = 1.2f;        // kg/m³ (typical at sea level)
    const float dragCoefficient = 0.47f;  // approximate for a sphere, tweak per object
    const float objectArea = 1.0f;        // cross-sectional area in m², you can scale per object
    const float terminalVelocity = 50.0f; // m/s, maximum fall speed

    for (auto& obj : physicalModels) {
        if (!obj.instance || obj.physics.isStatic)
            continue;

        // --- GRAVITY ---
        glm::vec3 gravity(0.0f, -9.81f * gravityG, 0.0f);

        // --- AIR RESISTANCE ---
        glm::vec3 velocityDir = glm::normalize(obj.physics.velocity);
        float speed = glm::length(obj.physics.velocity);
        glm::vec3 drag(0.0f);

        if (speed > 0.0f) {
            // Drag force: F = 0.5 * rho * v^2 * Cd * A
            drag = -0.5f * airDensity * speed * speed * dragCoefficient * objectArea * velocityDir;
        }

        // --- TOTAL ACCELERATION ---
        glm::vec3 acceleration = gravity + drag / obj.physics.mass;

        // Clamp vertical velocity to terminal velocity
        if (acceleration.y < 0.0f && obj.physics.velocity.y + acceleration.y * deltaTime < -terminalVelocity) {
            acceleration.y = (-terminalVelocity - obj.physics.velocity.y) / deltaTime;
        }

        // --- EXTERNAL FORCES / INERTIA ---
        if (glm::length(obj.physics.forces) > 0.0f) {
            acceleration += obj.physics.forces / obj.physics.mass;
            obj.physics.forces = glm::vec3(0.0f);
        }

        // --- ROTATION / ANGULAR INERTIA ---
        glm::vec3 angularAccel = obj.physics.angularVelocity / obj.physics.mass; // simplified
        obj.physics.angularVelocity += angularAccel * deltaTime;

        // --- INTEGRATE MOTION ---
        obj.physics.velocity += acceleration * deltaTime;
        obj.instance->position += obj.physics.velocity * deltaTime;
        obj.instance->rotation += obj.physics.angularVelocity * deltaTime;

        // --- COLLISION WITH FLOOR ---
        if (obj.instance->position.y < 0.0f) {
            obj.instance->position.y = 0.0f;

            // Bounce effect with damping
            obj.physics.velocity.y *= -0.3f;

            // Horizontal friction
            obj.physics.velocity.x *= 0.98f;
            obj.physics.velocity.z *= 0.98f;
        }

#ifdef Debug
        std::cout << std::fixed << std::setprecision(4)
            << "[Physics] Object @ "
            << "Pos(" << obj.instance->position.x << ", "
            << obj.instance->position.y << ", "
            << obj.instance->position.z << ") | "
            << "Vel(" << obj.physics.velocity.x << ", "
            << obj.physics.velocity.y << ", "
            << obj.physics.velocity.z << ") | "
            << "Rot(" << obj.instance->rotation.x << ", "
            << obj.instance->rotation.y << ", "
            << obj.instance->rotation.z << ") | "
            << "AngularVel(" << obj.physics.angularVelocity.x << ", "
            << obj.physics.angularVelocity.y << ", "
            << obj.physics.angularVelocity.z << ") | "
            << "Mass(" << obj.physics.mass << ")" << std::endl;
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
static bool prevIsStatic = false;  // remember original static state

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

void OnRightClickPressed(const Camera& cam, double mouseX, double mouseY, int windowWidth, int windowHeight) {
    if (dragging) return;

    glm::vec2 ndc = {
        (2.0f * mouseX) / windowWidth - 1.0f,
        1.0f - (2.0f * mouseY) / windowHeight
    };

    glm::mat4 projection = glm::perspective(glm::radians(cam.fov), (float)windowWidth / windowHeight, 0.1f, 1000.0f);
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

        // Remember previous static state and make object static
        prevIsStatic = hitModel->physics.isStatic;
        hitModel->physics.isStatic = true;

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
    if (!dragging || !draggedModel) return;

    // Restore original static state
    draggedModel->physics.isStatic = prevIsStatic;

    dragging = false;
    draggedModel = nullptr;
}

// Update dragging constraint with inertia based on mass
void UpdateDrag(const Camera& cam, double mouseX, double mouseY, int windowWidth, int windowHeight) {
    if (!dragging || !draggedModel) return;

    glm::vec2 ndc = {
        (2.0f * mouseX) / windowWidth - 1.0f,
        1.0f - (2.0f * mouseY) / windowHeight
    };

    glm::mat4 projection = glm::perspective(glm::radians(cam.fov), (float)windowWidth / windowHeight, 0.1f, 1000.0f);
    glm::mat4 view = GetViewMatrix(cam);
    glm::mat4 invVP = glm::inverse(projection * view);

    glm::vec4 rayStartNDC(ndc.x, ndc.y, -1.0f, 1.0f);
    glm::vec4 rayEndNDC(ndc.x, ndc.y, 0.0f, 1.0f);

    glm::vec4 rayStartWorld = invVP * rayStartNDC; rayStartWorld /= rayStartWorld.w;
    glm::vec4 rayEndWorld = invVP * rayEndNDC; rayEndWorld /= rayEndWorld.w;

    glm::vec3 rayDir = glm::normalize(glm::vec3(rayEndWorld - rayStartWorld));
    glm::vec3 rayOrigin = glm::vec3(rayStartWorld);

    float grabDist = glm::length(grabWorldPoint - cam.position);
    glm::vec3 newGrabPos = rayOrigin + rayDir * grabDist;
    glm::vec3 newObjectPos = newGrabPos - grabLocalOffset;

    float massFactor = 1000 / std::sqrt(draggedModel->physics.mass); // tweak factor as needed
    draggedModel->instance->position = glm::mix(draggedModel->instance->position, newObjectPos, 0.2f * massFactor);
}