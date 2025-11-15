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

// -------------------- utilities --------------------

static double SignedTetraVolume(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) {
    return glm::dot(a, glm::cross(b, c)) / 6.0;
}

// Closest point on triangle to point (Real-Time Collision Detection algorithm)
static glm::vec3 ClosestPointOnTriangle(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    // Check vertex region outside A
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ap = p - a;
    float d1 = glm::dot(ab, ap);
    float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    // Vertex region outside B
    glm::vec3 bp = p - b;
    float d3 = glm::dot(ab, bp);
    float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    // Edge region AB
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab;
    }

    // Vertex region outside C
    glm::vec3 cp = p - c;
    float d5 = glm::dot(ab, cp);
    float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    // Edge region AC
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + w * ac;
    }

    // Edge region BC
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    // Inside face region
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

// compute model-space AABB transformed to world space
static void ComputeAABB(const ModelInstance& inst, glm::vec3& outMin, glm::vec3& outMax) {
    const auto& verts = inst.model.vertexCoords;
    if (verts.empty()) {
        outMin = inst.position;
        outMax = inst.position;
        return;
    }

    glm::mat4 M = ComputeModelMatrix(inst);
    outMin = glm::vec3(std::numeric_limits<float>::infinity());
    outMax = glm::vec3(-std::numeric_limits<float>::infinity());
    for (size_t i = 0; i < verts.size(); i += 3) {
        glm::vec4 p(verts[i + 0], verts[i + 1], verts[i + 2], 1.0f);
        glm::vec3 pw = glm::vec3(M * p);
        outMin = glm::min(outMin, pw);
        outMax = glm::max(outMax, pw);
    }
}

// simple AABB overlap test
static bool AABBOverlap(const glm::vec3& aMin, const glm::vec3& aMax, const glm::vec3& bMin, const glm::vec3& bMax) {
    return (aMin.x <= bMax.x && aMax.x >= bMin.x) &&
        (aMin.y <= bMax.y && aMax.y >= bMin.y) &&
        (aMin.z <= bMax.z && aMax.z >= bMin.z);
}

// -------------------- collision pipeline --------------------

// broadphase: returns pairs that AABB-overlap
std::vector<Contact> BroadphaseGeneratePairs() {
    std::vector<Contact> empty;
    int n = (int)physicalModels.size();
    std::vector<glm::vec3> mins(n), maxs(n);
    for (int i = 0; i < n; ++i) ComputeAABB(*physicalModels[i].instance, mins[i], maxs[i]);

    std::vector<Contact> pairs;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (!AABBOverlap(mins[i], maxs[i], mins[j], maxs[j])) continue;
            Contact c; c.A = &physicalModels[i]; c.B = &physicalModels[j];
            pairs.push_back(c);
        }
    }
    return pairs;
}

// narrowphase: vertex->triangle distance both ways to detect candidate contact
std::vector<Contact> NarrowphaseGenerateContacts(PhysicalModel* A, PhysicalModel* B) {
    std::vector<Contact> result;
    const auto& meshA = A->physics.instance.model;
    const auto& meshB = B->physics.instance.model;

    glm::mat4 MA = ComputeModelMatrix(A->physics.instance);
    glm::mat4 MB = ComputeModelMatrix(B->physics.instance);

    const auto& vertsA = meshA.vertexCoords;
    const auto& idxA = meshA.elementArray;
    const auto& vertsB = meshB.vertexCoords;
    const auto& idxB = meshB.elementArray;

    // Threshold for contact detection (small penetration tolerance)
    const float contactThreshold = 0.001f;

    // A vertices -> B triangles
    for (size_t vi = 0; vi + 2 < vertsA.size(); vi += 3) {
        glm::vec3 pv = glm::vec3(MA * glm::vec4(vertsA[vi], vertsA[vi + 1], vertsA[vi + 2], 1.0f));
        for (size_t t = 0; t + 2 < idxB.size(); t += 3) {
            glm::vec3 b0 = glm::vec3(MB * glm::vec4(vertsB[idxB[t + 0] * 3 + 0], vertsB[idxB[t + 0] * 3 + 1], vertsB[idxB[t + 0] * 3 + 2], 1.0f));
            glm::vec3 b1 = glm::vec3(MB * glm::vec4(vertsB[idxB[t + 1] * 3 + 0], vertsB[idxB[t + 1] * 3 + 1], vertsB[idxB[t + 1] * 3 + 2], 1.0f));
            glm::vec3 b2 = glm::vec3(MB * glm::vec4(vertsB[idxB[t + 2] * 3 + 0], vertsB[idxB[t + 2] * 3 + 1], vertsB[idxB[t + 2] * 3 + 2], 1.0f));
            glm::vec3 closest = ClosestPointOnTriangle(pv, b0, b1, b2);
            glm::vec3 diff = pv - closest;
            float dist = glm::length(diff);
            if (dist <= contactThreshold) {
                Contact c;
                c.A = A; c.B = B;
                c.point = (pv + closest) * 0.5f;
                c.normal = glm::normalize((closest - pv) + glm::vec3(1e-8f)); // from A->B
                c.penetration = contactThreshold - dist;
                result.push_back(c);
            }
        }
    }

    // B vertices -> A triangles
    for (size_t vi = 0; vi + 2 < vertsB.size(); vi += 3) {
        glm::vec3 pv = glm::vec3(MB * glm::vec4(vertsB[vi], vertsB[vi + 1], vertsB[vi + 2], 1.0f));
        for (size_t t = 0; t + 2 < idxA.size(); t += 3) {
            glm::vec3 a0 = glm::vec3(MA * glm::vec4(vertsA[idxA[t + 0] * 3 + 0], vertsA[idxA[t + 0] * 3 + 1], vertsA[idxA[t + 0] * 3 + 2], 1.0f));
            glm::vec3 a1 = glm::vec3(MA * glm::vec4(vertsA[idxA[t + 1] * 3 + 0], vertsA[idxA[t + 1] * 3 + 1], vertsA[idxA[t + 1] * 3 + 2], 1.0f));
            glm::vec3 a2 = glm::vec3(MA * glm::vec4(vertsA[idxA[t + 2] * 3 + 0], vertsA[idxA[t + 2] * 3 + 1], vertsA[idxA[t + 2] * 3 + 2], 1.0f));
            glm::vec3 closest = ClosestPointOnTriangle(pv, a0, a1, a2);
            glm::vec3 diff = pv - closest;
            float dist = glm::length(diff);
            if (dist <= contactThreshold) {
                Contact c;
                c.A = A; c.B = B;
                c.point = (pv + closest) * 0.5f;
                c.normal = glm::normalize((closest - pv) + glm::vec3(1e-8f)); // from B->A (we will flip)
                c.normal = -c.normal; // ensure normal is A -> B
                c.penetration = contactThreshold - dist;
                result.push_back(c);
            }
        }
    }

    return result;
}

// Apply an impulse at world-space contact point to a physical model
void ApplyImpulse(PhysicalModel& pm, const glm::vec3& impulse, const glm::vec3& contactPointWorld) {
    if (pm.physics.isStatic) return;
    float invM = (pm.physics.mass > 0.0f) ? (1.0f / pm.physics.mass) : 0.0f;

    // linear
    pm.physics.velocity += impulse * invM;

    // angular: DELTAL = r x impulse
    glm::vec3 r = contactPointWorld - (pm.instance->position + pm.physics.centerOfGravity);
    pm.physics.angularMomentum += glm::cross(r, impulse);

    // update angular velocity from L: need current world-space inverse inertia
    // (inertia tensor world must be up-to-date)
    glm::mat3 R = glm::mat3_cast(glm::quat(pm.instance->rotation));
    pm.physics.inertiaTensorWorld = R * pm.physics.inertiaTensorLocal * glm::transpose(R);
    pm.physics.inertiaTensorWorldInv = glm::inverse(pm.physics.inertiaTensorWorld + glm::mat3(1e-8f));

    pm.physics.angularVelocity = pm.physics.inertiaTensorWorldInv * pm.physics.angularMomentum;
}

// Resolve a single contact using impulse resolution with friction+restitution
void ResolveContactImpulse(const Contact& c) {
    PhysicalModel* A = c.A;
    PhysicalModel* B = c.B;
    if (!A || !B) return;

    // If either static, handle correctly
    float invMassA = (A->physics.isStatic || A->physics.mass <= 0.0f) ? 0.0f : 1.0f / A->physics.mass;
    float invMassB = (B->physics.isStatic || B->physics.mass <= 0.0f) ? 0.0f : 1.0f / B->physics.mass;

    // relative position from COM to contact
    glm::vec3 rA = c.point - (A->instance->position + A->physics.centerOfGravity);
    glm::vec3 rB = c.point - (B->instance->position + B->physics.centerOfGravity);

    // update world inertia inverse for both (ensure not singular)
    glm::mat3 RA = glm::mat3_cast(glm::quat(A->instance->rotation));
    glm::mat3 RB = glm::mat3_cast(glm::quat(B->instance->rotation));
    A->physics.inertiaTensorWorld = RA * A->physics.inertiaTensorLocal * glm::transpose(RA);
    B->physics.inertiaTensorWorld = RB * B->physics.inertiaTensorLocal * glm::transpose(RB);
    A->physics.inertiaTensorWorldInv = glm::inverse(A->physics.inertiaTensorWorld + glm::mat3(1e-8f));
    B->physics.inertiaTensorWorldInv = glm::inverse(B->physics.inertiaTensorWorld + glm::mat3(1e-8f));

    // velocities at contact
    glm::vec3 vA = A->physics.velocity + glm::cross(A->physics.angularVelocity, rA);
    glm::vec3 vB = B->physics.velocity + glm::cross(B->physics.angularVelocity, rB);
    glm::vec3 rv = vA - vB;

    glm::vec3 n = glm::normalize(c.normal + glm::vec3(1e-8f));
    float velAlongNormal = glm::dot(rv, n);

    // don't resolve separating contacts
    if (velAlongNormal > 0.0f && c.penetration <= 0.0f) return;

    // restitution: average
    float e = std::min(A->physics.material.restitution, B->physics.material.restitution);

    // compute denominator: invMass sum + rotational terms
    glm::vec3 rnA = glm::cross(rA, n);
    glm::vec3 rnB = glm::cross(rB, n);
    float denom = invMassA + invMassB +
        glm::dot(n, glm::cross(A->physics.inertiaTensorWorldInv * rnA, rA)) +
        glm::dot(n, glm::cross(B->physics.inertiaTensorWorldInv * rnB, rB));

    float j = 0.0f;
    if (denom > 1e-8f) {
        j = -(1.0f + e) * velAlongNormal / denom;
    }

    // Positional correction (baumgarte) to avoid sinking
    const float k_slop = 0.01f;
    const float percent = 0.2f;
    float correction = std::max(c.penetration - k_slop, 0.0f) / (invMassA + invMassB) * percent;
    glm::vec3 correctionVec = correction * n;
    if (!A->physics.isStatic) A->instance->position += correctionVec * invMassA;
    if (!B->physics.isStatic) B->instance->position -= correctionVec * invMassB;

    // Apply normal impulse
    glm::vec3 impulse = j * n;
    if (!A->physics.isStatic) ApplyImpulse(*A, impulse, c.point);
    if (!B->physics.isStatic) ApplyImpulse(*B, -impulse, c.point);

    // Friction impulse (Coulomb)
    // tangent
    glm::vec3 vt = rv - velAlongNormal * n;
    float vtLen = glm::length(vt);
    glm::vec3 tangent = (vtLen > 1e-6f) ? (vt / vtLen) : glm::vec3(0.0f);

    float mu = std::sqrt(A->physics.material.friction * B->physics.material.friction); // approximate
    // magnitude of friction impulse
    // compute denominator in tangent direction similarly:
    glm::vec3 rtA = glm::cross(rA, tangent);
    glm::vec3 rtB = glm::cross(rB, tangent);
    float denomT = invMassA + invMassB +
        glm::dot(tangent, glm::cross(A->physics.inertiaTensorWorldInv * rtA, rA)) +
        glm::dot(tangent, glm::cross(B->physics.inertiaTensorWorldInv * rtB, rB));
    float jt = 0.0f;
    if (denomT > 1e-8f) {
        jt = -glm::dot(rv, tangent) / denomT;
        // clamp friction
        if (std::abs(jt) > j * mu) jt = glm::sign(jt) * j * mu;
    }
    glm::vec3 frictionImpulse = jt * tangent;
    if (!A->physics.isStatic) ApplyImpulse(*A, frictionImpulse, c.point);
    if (!B->physics.isStatic) ApplyImpulse(*B, -frictionImpulse, c.point);

#ifdef Debug
    std::cout << "[Contact] j=" << j << " jt=" << jt << " contactPen=" << c.penetration << "\n";
#endif
}

// Compute kinetic energy of a physical model (trans + rot)
float ComputeKineticEnergy(const PhysicalModel& pm) {
    float trans = 0.5f * pm.physics.mass * glm::dot(pm.physics.velocity, pm.physics.velocity);

    // rotational energy: 0.5 * w^T * I * w
    glm::vec3 w = pm.physics.angularVelocity;
    glm::vec3 Iw = pm.physics.inertiaTensorWorld * w;
    float rot = 0.5f * glm::dot(w, Iw);
    return trans + rot;
}

// -------------------- main physics loop --------------------

void RegisterPhysicalModel(ModelInstance& instance, const Material& mat) {
    // compute volume & centroid (reuse your existing method or call ComputeVolumeAndCentroid if available)
    // For brevity assume you have phys.volumeCM3 and centerOfGravity already computed previously
    // Here we approximate mass from volume * density (volume in m^3)
    glm::vec3 cg = instance.position + glm::vec3(0.0f); // if you already compute centroid, use that
    float volume_m3 = (instance.model.vertexCoords.size() > 0) ? 1e-6f : 0.0f; // fallback hack: user should compute
    float mass = std::max(0.001f, volume_m3 * mat.density); // guard

    PhysicalEntity phys;
    phys.instance = instance;
    phys.mass = mass;
    phys.material = mat;
    phys.velocity = glm::vec3(0.0f);
    phys.angularVelocity = glm::vec3(0.0f);
    phys.centerOfGravity = glm::vec3(0.0f);
    phys.volumeCM3 = 0.0f;

    // inertia initialization: user should compute accurate inertia; we set diagonal approximate
    float r = 0.5f;
    float I = 0.4f * mass * r * r;
    phys.inertiaTensorLocal = glm::mat3(I);
    phys.inertiaTensorLocalInv = glm::inverse(phys.inertiaTensorLocal);
    phys.angularMomentum = glm::vec3(0.0f);

    physicalModels.push_back({ &instance, phys });

#ifdef Debug
    std::cout << "[Physics] Registered model mass=" << phys.mass << "\n";
#endif
}

void UpdatePhysics(float deltaTime) {
    if (deltaTime <= 0.0f) return;

    // 1) integrate forces -> linear velocity (explicit)
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;

        // gravity
        glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
        pm.physics.velocity += gravity * deltaTime;

        // apply accumulated forces (F = m a)
        if (glm::length(pm.physics.forces) > 0.0f) {
            pm.physics.velocity += (pm.physics.forces / pm.physics.mass) * deltaTime;
            pm.physics.forces = glm::vec3(0.0f);
        }

        // accumulate torque already done externally via pm.physics.torque
    }

    // 2) collision detection + resolution
    // broadphase
    auto pairs = BroadphaseGeneratePairs();

    // generate contacts and solve immediately (basic iterative)
    std::vector<Contact> allContacts;
    for (auto& p : pairs) {
        auto cs = NarrowphaseGenerateContacts(p.A, p.B);
        for (auto& c : cs) allContacts.push_back(c);
    }

    // iterate resolving contacts (simple 10 iterations)
    int iters = 8;
    for (int it = 0; it < iters; ++it) {
        for (auto& c : allContacts) ResolveContactImpulse(c);
    }

    // 3) rotational dynamics: update inertia world, integrate angular momentum -> orientation
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;

        // compute world inertia
        glm::quat q = glm::quat(pm.instance->rotation);
        glm::mat3 R = glm::mat3_cast(q);
        pm.physics.inertiaTensorWorld = R * pm.physics.inertiaTensorLocal * glm::transpose(R);
        // invert (safe)
        pm.physics.inertiaTensorWorldInv = glm::inverse(pm.physics.inertiaTensorWorld + glm::mat3(1e-8f));

        // angular momentum update (torque is in world space)
        pm.physics.angularMomentum += pm.physics.torque * deltaTime;
        pm.physics.torque = glm::vec3(0.0f);

        // gyroscopic term and compute angular velocity from L: w = I^-1 * L
        pm.physics.angularVelocity = pm.physics.inertiaTensorWorldInv * pm.physics.angularMomentum;

        // integrate orientation using semi-implicit quaternion integration
        // dq/dt = 0.5 * q * [0, w]
        glm::quat wq(0.0f, pm.physics.angularVelocity.x, pm.physics.angularVelocity.y, pm.physics.angularVelocity.z);
        glm::quat qNew = q + 0.5f * wq * q * (float)deltaTime;
        qNew = glm::normalize(qNew);
        pm.instance->rotation = glm::eulerAngles(qNew);
    }

    // 4) integrate linear velocities -> positions (semi-implicit)
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;

        pm.instance->position += pm.physics.velocity * deltaTime;
    }

    // Debug energy
#ifdef Debug
    float totalE = 0.0f;
    for (auto& pm : physicalModels) totalE += ComputeKineticEnergy(pm);
    std::cout << "[Physics] total kinetic energy: " << totalE << std::endl;
#endif
}

// Pretty-print
void PrintPhysicsState() {
#ifdef Debug
    std::cout << "====== Physics Debug Info ======" << std::endl;
    int i = 0;
    for (auto& obj : physicalModels) {
        if (!obj.instance) continue;
        std::cout << "Object[" << i++ << "] mass=" << obj.physics.mass
            << " pos=(" << obj.instance->position.x << "," << obj.instance->position.y << "," << obj.instance->position.z << ")"
            << " vel=(" << obj.physics.velocity.x << "," << obj.physics.velocity.y << "," << obj.physics.velocity.z << ")"
            << " angVel=(" << obj.physics.angularVelocity.x << "," << obj.physics.angularVelocity.y << "," << obj.physics.angularVelocity.z << ")\n";
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