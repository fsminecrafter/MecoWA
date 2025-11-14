#define GLM_ENABLE_EXPERIMENTAL
#include "physics.h"
#include <glm/gtx/rotate_vector.hpp>
#include <iostream>
#include <iomanip>
#include "engine_camera.h"
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtx/intersect.hpp>
#include <cmath>
#include <limits>

std::vector<PhysicalModel> physicalModels;

#define Debug

// Compute signed tetrahedron volume (origin, a, b, c)
static double SignedTetraVolume(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) {
    // Volume = dot(a, cross(b, c)) / 6
    return glm::dot(a, glm::cross(b, c)) / 6.0;
}

// Computes both volume (m^3) and center-of-gravity (centroid) in same units as mesh
// Returns pair: (volume_m3, centroid)
// robust ComputeVolumeAndCentroid: uses absolute tetra volumes by default
static std::pair<double, glm::dvec3> ComputeVolumeAndCentroid(const ModelInstance& instance) {
    const auto& verts_f = instance.model.vertexCoords;
    const auto& idx = instance.model.elementArray;

    const size_t vcount = verts_f.size() / 3;
    std::vector<glm::dvec3> verts;
    verts.reserve(vcount);
    for (size_t i = 0; i < vcount; ++i) {
        verts.emplace_back(
            (double)verts_f[i * 3 + 0],
            (double)verts_f[i * 3 + 1],
            (double)verts_f[i * 3 + 2]
        );
    }

    double signedVolSum = 0.0;
    glm::dvec3 signedWeightedCentroid(0.0);

    double absVolSum = 0.0;
    glm::dvec3 absWeightedCentroid(0.0);

    // helper lambda for signed tetra volume
    auto SignedTetraVolume = [](const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) -> double {
        return glm::dot(a, glm::cross(b, c)) / 6.0;
        };

    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        uint32_t i0 = idx[i + 0];
        uint32_t i1 = idx[i + 1];
        uint32_t i2 = idx[i + 2];
        if (i0 >= verts.size() || i1 >= verts.size() || i2 >= verts.size()) continue;

        const glm::dvec3& v0 = verts[i0];
        const glm::dvec3& v1 = verts[i1];
        const glm::dvec3& v2 = verts[i2];

        // signed tetra volume (origin, v0, v1, v2)
        double tetSignedVol = SignedTetraVolume(v0, v1, v2);
        // centroid of that tetra (origin + v0 + v1 + v2) / 4  -> since origin is zero: (v0+v1+v2)/4
        glm::dvec3 tetCentroid = (v0 + v1 + v2) * 0.25;

        signedVolSum += tetSignedVol;
        signedWeightedCentroid += tetCentroid * tetSignedVol;

        double tetAbsVol = std::abs(tetSignedVol);
        absVolSum += tetAbsVol;
        absWeightedCentroid += tetCentroid * tetAbsVol;
    }

    const double eps = 1e-12;
    glm::dvec3 centroid;
    double volume_m3;

    // --- Choose robust absolute-volume centroid if signed volume is small or inconsistent ---
    if (std::abs(signedVolSum) > eps) {
        // If signed volume is large and consistent, use it (exact for closed/wound meshes)
        centroid = signedWeightedCentroid / signedVolSum;
        volume_m3 = signedVolSum;
    }
    else if (absVolSum > eps) {
        // Fallback: use absolute-volume weighted centroid (robust for mixed winding / non-closed)
        centroid = absWeightedCentroid / absVolSum;
        // 'volume' from absolute tetra volumes -- may overcount if mesh self-intersects,
        // but it's a sensible positive measure of enclosed mass for physics mass estimation.
        volume_m3 = absVolSum;
    }
    else {
        // Last resort: surface-area centroid fallback
        double totalArea = 0.0;
        glm::dvec3 areaWeighted(0.0);
        for (size_t i = 0; i + 2 < idx.size(); i += 3) {
            uint32_t i0 = idx[i + 0], i1 = idx[i + 1], i2 = idx[i + 2];
            if (i0 >= verts.size() || i1 >= verts.size() || i2 >= verts.size()) continue;
            glm::dvec3 v0 = verts[i0], v1 = verts[i1], v2 = verts[i2];
            double area = 0.5 * glm::length(glm::cross(v1 - v0, v2 - v0));
            glm::dvec3 triC = (v0 + v1 + v2) / 3.0;
            areaWeighted += triC * area;
            totalArea += area;
        }
        if (totalArea > eps) centroid = areaWeighted / totalArea;
        else centroid = glm::dvec3(0.0);
        volume_m3 = 0.0;
    }

#ifdef Debug
    glm::dvec3 signedCentroid = (std::abs(signedVolSum) > eps) ? (signedWeightedCentroid / signedVolSum) : glm::dvec3(0.0);
    glm::dvec3 absCentroid = (absVolSum > eps) ? (absWeightedCentroid / absVolSum) : glm::dvec3(0.0);
    double signedVolAbs = std::abs(signedVolSum);
    double absVolAbs = absVolSum;
    std::cout << std::fixed << std::setprecision(6)
        << "[ComputeVolumeAndCentroid] signedVol=" << signedVolSum
        << " | abs(signedVol)=" << signedVolAbs
        << " | absAccumVol=" << absVolAbs << "\n"
        << "  signedCentroid=(" << (double)signedCentroid.x << "," << (double)signedCentroid.y << "," << (double)signedCentroid.z << ")\n"
        << "  absCentroid=(" << (double)absCentroid.x << "," << (double)absCentroid.y << "," << (double)absCentroid.z << ")\n"
        << "  chosenCentroid=(" << (double)centroid.x << "," << (double)centroid.y << "," << (double)centroid.z << ")\n";
    if (std::abs(signedVolSum) <= eps) {
        std::cout << "[ComputeVolumeAndCentroid] NOTE: signed volume near-zero or inconsistent winding. Using absolute-volume centroid.\n";
    }
#endif

    return { volume_m3, centroid };
}


// Registers a model with physics simulation
void RegisterPhysicalModel(ModelInstance& instance, const Material& mat) {
    auto [volume_m3, centroid_d] = ComputeVolumeAndCentroid(instance);
    double volumeAbs = std::abs(volume_m3);
    double vol_cm3 = volumeAbs * 1e6; // m^3 -> cm^3
    glm::vec3 centroid = glm::vec3(centroid_d);

    float vertexCount = static_cast<float>(instance.model.vertexCoords.size()) / 3.0f;
    float mass = (float)std::abs(volume_m3) * mat.density;

    PhysicalEntity phys;
    phys.instance = instance;
    phys.mass = mass;
    phys.material = mat;
    phys.velocity = glm::vec3(0.0f);
    phys.angularVelocity = glm::vec3(0.0f);
    phys.centerOfGravity = centroid;
    phys.volumeCM3 = vol_cm3;

    physicalModels.push_back({ &instance, phys });

#ifdef Debug
    std::cout << std::fixed << std::setprecision(4)
        << "[Physics] Registered model with mass " << phys.mass << " kg"
        << " | Vertices: " << vertexCount
        << " | Density: " << mat.density << std::endl
	    << " | Center of Gravity: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")"
		<< " | Volume: " << vol_cm3 << " cm³" << std::endl;
#endif
}

void UpdatePhysics(float deltaTime) {

    for (auto& obj : physicalModels) {
        if (!obj.instance || obj.physics.isStatic)
            continue;

        float mass = obj.physics.mass;
        float volumeM3 = obj.physics.volumeCM3 / 1'000'000.0f;  // back to m3

        // --- CROSS SECTION AREA ---
        // Approximate area from volume: A ~ V^(2/3)
        float crossSectionArea = pow(volumeM3, 2.0f / 3.0f);

        // --- DRAG COEFFICIENT ---
        float Cd = obj.physics.material.dragCoefficient; // add to your material system
        if (Cd <= 0.0f) Cd = 0.60f; // default cube-ish object

        // --- GRAVITY (mass-independent) ---
        glm::vec3 gravity(0.0f, -9.81f * gravityG, 0.0f);

        // --- DRAG FORCE ---
        glm::vec3 vel = obj.physics.velocity;
        float speed = glm::length(vel);
        glm::vec3 drag(0.0f);

        if (speed > 0.0f) {
            glm::vec3 vDir = vel / speed;
            drag = -0.5f * airDensity * speed * speed * Cd * crossSectionArea * vDir;
        }

        // --- ACCELERATION ---
        glm::vec3 acceleration = gravity + drag / mass;

        // -----------------------------
        // REALISTIC TERMINAL VELOCITY
        // v_t = sqrt( (2*m*g) / (rho * Cd * A) )
        // -----------------------------
        float terminalV = sqrt((2.0f * mass * 9.81f) /
            (airDensity * Cd * crossSectionArea));

        if (obj.physics.velocity.y < -terminalV)
            obj.physics.velocity.y = -terminalV;

        // --- EXTERNAL FORCES ---
        if (glm::length(obj.physics.forces) > 0.0f) {
            acceleration += obj.physics.forces / mass;
            obj.physics.forces = glm::vec3(0.0f);
        }

        // --- ROTATION / MOMENT OF INERTIA ---
        // For irregular shape: I ~ k * m * r2  (k~0.4 works well)
        float radius = pow((3.0f * volumeM3) / (4.0f * 3.14159f), 1.0f / 3.0f);
        float MOI = 0.4f * mass * radius * radius;

        glm::vec3 angularAccel = obj.physics.torque / MOI;
        obj.physics.angularVelocity += angularAccel * deltaTime;
        obj.physics.torque = glm::vec3(0.0f);

        // --- APPLY ROTATION AROUND CENTER OF GRAVITY ---

        // Step 1: Build rotation quaternion from angular velocity
        glm::quat dq = glm::quat(
            0.0f,
            obj.physics.angularVelocity.x * deltaTime,
            obj.physics.angularVelocity.y * deltaTime,
            obj.physics.angularVelocity.z * deltaTime
        );

        // Current orientation (convert euler to quat)
        glm::quat currentRot = glm::quat(obj.instance->rotation);

        // New orientation
        glm::quat newRot = currentRot + 0.5f * dq * currentRot;
        newRot = glm::normalize(newRot);

        // Step 2: Save rotation back as Euler
        obj.instance->rotation = glm::eulerAngles(newRot);

        // Step 3: Apply rotation around CG
        glm::vec3 cgWorld = obj.instance->position + obj.physics.centerOfGravity;

        // Translate object so CG is pivot
        glm::vec3 localOffset = obj.instance->position - cgWorld;

        // Rotate object using quaternion
        localOffset = localOffset * newRot;

        // Move object back
        obj.instance->position = cgWorld + localOffset;


        // --- INTEGRATE ---
        obj.physics.velocity += acceleration * deltaTime;

        obj.instance->position += obj.physics.velocity * deltaTime;
        obj.instance->rotation += obj.physics.angularVelocity * deltaTime;

        // --- FLOOR COLLISION ---
        if (obj.instance->position.y < 0.0f) {
            obj.instance->position.y = 0.0f;
            obj.physics.velocity.y *= -0.3f;

            obj.physics.velocity.x *= 0.92f;
            obj.physics.velocity.z *= 0.92f;
        }

#ifdef Debug
        std::cout
            << "[Physics] Object "
            << "Pos(" << obj.instance->position.x << ", "
            << obj.instance->position.y << ", "
            << obj.instance->position.z << ") | "
            << "Vel(" << obj.physics.velocity.x << ", "
            << obj.physics.velocity.y << ", "
            << obj.physics.velocity.z << ") | "
            << "TermV(" << terminalV << ") | "
            << "Mass(" << mass << ") | "
            << "Area(" << crossSectionArea << ")\n";
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
            << obj.physics.velocity.z << ")" << std::endl
            << obj.instance->rotation.x << ", "
            << obj.instance->rotation.y << ", "
            << obj.instance->rotation.z << ") ";
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