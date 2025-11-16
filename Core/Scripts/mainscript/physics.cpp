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
#include "objloader.h"
#include "ComputeInertiaTensor.h"

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

static void ComputeAABB(const ModelInstance& inst, glm::vec3& outMin, glm::vec3& outMax) {
    const auto& verts = inst.model.vertexCoords;

    if (verts.empty()) {
        outMin = inst.position;
        outMax = inst.position;
        return;
    }

    if (verts.size() % 3 != 0) {
        std::cerr << "[ComputeAABB] Warning: vertexCoords.size() is not multiple of 3! "
            << "size=" << verts.size() << std::endl;
        outMin = inst.position;
        outMax = inst.position;
        return;
    }

    glm::mat4 M = ComputeModelMatrix(inst);
    outMin = glm::vec3(std::numeric_limits<float>::infinity());
    outMax = glm::vec3(-std::numeric_limits<float>::infinity());

    for (size_t i = 0; i + 2 < verts.size(); i += 3) {
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

// Helper: compute center-of-gravity in world space from stored local centroid
static glm::vec3 ComputeCenterWorld(const ModelInstance& instance, const glm::vec3& centerLocal) {
    // rotation -> matrix (assumes instance.rotation is Euler degrees)
    glm::quat q = glm::quat(glm::radians(instance.rotation));
    glm::mat3 R = glm::mat3_cast(q);
    return instance.position + R * centerLocal * instance.scale; // account for scale
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
    if (!A || !B) return result;

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

    // Loop vertices correctly (iterate vertex count, not raw floats)
    size_t vcountA = vertsA.size() / 3;
    for (size_t vi = 0; vi < vcountA; ++vi)
    {
        glm::vec3 pv = glm::vec3(
            MA * glm::vec4(
                vertsA[vi * 3 + 0],
                vertsA[vi * 3 + 1],
                vertsA[vi * 3 + 2],
                1.0f
            )
        );

        for (size_t t = 0; t + 2 < idxB.size(); t += 3)
        {
            uint32_t i0 = idxB[t + 0];
            uint32_t i1 = idxB[t + 1];
            uint32_t i2 = idxB[t + 2];
            // guard indices
            if ((size_t)i0 * 3 + 2 >= vertsB.size() ||
                (size_t)i1 * 3 + 2 >= vertsB.size() ||
                (size_t)i2 * 3 + 2 >= vertsB.size()) continue;

            glm::vec3 b0 = glm::vec3(MB * glm::vec4(
                vertsB[i0 * 3 + 0],
                vertsB[i0 * 3 + 1],
                vertsB[i0 * 3 + 2], 1.0f));

            glm::vec3 b1 = glm::vec3(MB * glm::vec4(
                vertsB[i1 * 3 + 0],
                vertsB[i1 * 3 + 1],
                vertsB[i1 * 3 + 2], 1.0f));

            glm::vec3 b2 = glm::vec3(MB * glm::vec4(
                vertsB[i2 * 3 + 0],
                vertsB[i2 * 3 + 1],
                vertsB[i2 * 3 + 2], 1.0f));

            glm::vec3 triNormal = glm::normalize(glm::cross(b1 - b0, b2 - b0));
            glm::vec3 closest = ClosestPointOnTriangle(pv, b0, b1, b2);
            glm::vec3 diff = pv - closest;
            float dist = glm::length(diff);

            if (dist <= contactThreshold)
            {
                // orient normal correctly (pointing from A -> B)
                if (glm::dot(diff, triNormal) < 0.0f)
                    triNormal = -triNormal;

                Contact c;
                c.A = A;
                c.B = B;
                c.point = 0.5f * (pv + closest);
                c.normal = triNormal;
                c.penetration = contactThreshold - dist;
                result.push_back(c);
            }
        }
    }

    // B vertices -> A triangles
    size_t vcountB = vertsB.size() / 3;
    for (size_t vi = 0; vi < vcountB; ++vi)
    {
        glm::vec3 pv = glm::vec3(
            MB * glm::vec4(
                vertsB[vi * 3 + 0],
                vertsB[vi * 3 + 1],
                vertsB[vi * 3 + 2],
                1.0f
            )
        );

        for (size_t t = 0; t + 2 < idxA.size(); t += 3)
        {
            uint32_t i0 = idxA[t + 0];
            uint32_t i1 = idxA[t + 1];
            uint32_t i2 = idxA[t + 2];
            if ((size_t)i0 * 3 + 2 >= vertsA.size() ||
                (size_t)i1 * 3 + 2 >= vertsA.size() ||
                (size_t)i2 * 3 + 2 >= vertsA.size()) continue;

            glm::vec3 a0 = glm::vec3(MA * glm::vec4(
                vertsA[i0 * 3 + 0],
                vertsA[i0 * 3 + 1],
                vertsA[i0 * 3 + 2], 1.0f));

            glm::vec3 a1 = glm::vec3(MA * glm::vec4(
                vertsA[i1 * 3 + 0],
                vertsA[i1 * 3 + 1],
                vertsA[i1 * 3 + 2], 1.0f));

            glm::vec3 a2 = glm::vec3(MA * glm::vec4(
                vertsA[i2 * 3 + 0],
                vertsA[i2 * 3 + 1],
                vertsA[i2 * 3 + 2], 1.0f));

            glm::vec3 triNormal = glm::normalize(glm::cross(a1 - a0, a2 - a0));
            glm::vec3 closest = ClosestPointOnTriangle(pv, a0, a1, a2);
            glm::vec3 diff = pv - closest;
            float dist = glm::length(diff);

            if (dist <= contactThreshold)
            {
                if (glm::dot(diff, triNormal) < 0.0f)
                    triNormal = -triNormal;

                Contact c;
                c.A = A;
                c.B = B;
                c.point = 0.5f * (pv + closest);
                // For B->A pass we want normal from A->B, so flip
                c.normal = -triNormal;
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
    // compute center-of-gravity in world space from stored local centroid
    glm::vec3 centerWorld = ComputeCenterWorld(*pm.instance, pm.physics.centerOfGravity);
    glm::vec3 r = contactPointWorld - centerWorld;
    pm.physics.angularMomentum += glm::cross(r, impulse);

    // update angular velocity from L: need current world-space inverse inertia
    // (inertia tensor world must be up-to-date)
    glm::quat q = glm::quat(glm::radians(pm.instance->rotation));
    glm::mat3 R = glm::mat3_cast(q);
    pm.physics.inertiaTensorWorld = R * pm.physics.inertiaTensorLocal * glm::transpose(R);
    pm.physics.inertiaTensorWorldInv = glm::inverse(pm.physics.inertiaTensorWorld + glm::mat3(1e-8f));

    pm.physics.angularVelocity = pm.physics.inertiaTensorWorldInv * pm.physics.angularMomentum;
}

// Resolve a single contact using impulse resolution with friction+restitution
void ResolveContactImpulse(const Contact& c) {
    PhysicalModel* A = c.A;
    PhysicalModel* B = c.B;
    if (!A || !B) return;

    float invMassA = (!A->physics.isStatic && A->physics.mass > 0.0f) ? 1.0f / A->physics.mass : 0.0f;
    float invMassB = (!B->physics.isStatic && B->physics.mass > 0.0f) ? 1.0f / B->physics.mass : 0.0f;
    float invMassSum = invMassA + invMassB;
    if (invMassSum <= 0.0f) return; // both static

    glm::vec3 centerA = ComputeCenterWorld(*A->instance, A->physics.centerOfGravity);
    glm::vec3 centerB = ComputeCenterWorld(*B->instance, B->physics.centerOfGravity);
    glm::vec3 rA = c.point - centerA;
    glm::vec3 rB = c.point - centerB;

    // Update world-space inertia
    auto updateInertia = [](PhysicalEntity& p, const glm::vec3& euler) {
        glm::mat3 R = glm::mat3_cast(glm::quat(glm::radians(euler)));
        p.inertiaTensorWorld = R * p.inertiaTensorLocal * glm::transpose(R);
        p.inertiaTensorWorldInv = glm::inverse(p.inertiaTensorWorld + glm::mat3(1e-8f));
        };
    updateInertia(A->physics, A->instance->rotation);
    updateInertia(B->physics, B->instance->rotation);

    glm::vec3 vA = A->physics.velocity + glm::cross(A->physics.angularVelocity, rA);
    glm::vec3 vB = B->physics.velocity + glm::cross(B->physics.angularVelocity, rB);
    glm::vec3 rv = vA - vB;

    glm::vec3 n = glm::normalize(c.normal + glm::vec3(1e-8f));
    float velAlongNormal = glm::dot(rv, n);

    if (velAlongNormal > 0.0f && c.penetration <= 0.0f) return;

    float e = std::min(A->physics.material.restitution, B->physics.material.restitution);

    glm::vec3 rnA = glm::cross(rA, n);
    glm::vec3 rnB = glm::cross(rB, n);
    float denom = invMassA + invMassB +
        glm::dot(n, glm::cross(A->physics.inertiaTensorWorldInv * rnA, rA)) +
        glm::dot(n, glm::cross(B->physics.inertiaTensorWorldInv * rnB, rB));

    float j = (denom > 1e-8f) ? -(1.0f + e) * velAlongNormal / denom : 0.0f;

    // Positional correction: clamp to avoid explosions
    const float k_slop = 0.01f;
    const float percent = 0.2f;
    if (c.penetration > k_slop && invMassSum > 0.0f) {
        float correction = glm::clamp((c.penetration - k_slop) / invMassSum * percent, -0.1f, 0.1f);
        if (!A->physics.isStatic) A->instance->position += correction * n * invMassA;
        if (!B->physics.isStatic) B->instance->position -= correction * n * invMassB;
    }

    glm::vec3 impulse = j * n;
    if (!A->physics.isStatic) ApplyImpulse(*A, impulse, c.point);
    if (!B->physics.isStatic) ApplyImpulse(*B, -impulse, c.point);

    // friction
    glm::vec3 vt = rv - velAlongNormal * n;
    float vtLen = glm::length(vt);
    glm::vec3 tangent = (vtLen > 1e-6f) ? vt / vtLen : glm::vec3(0.0f);
    float mu = std::sqrt(A->physics.material.friction * B->physics.material.friction);

    glm::vec3 rtA = glm::cross(rA, tangent);
    glm::vec3 rtB = glm::cross(rB, tangent);
    float denomT = invMassA + invMassB +
        glm::dot(tangent, glm::cross(A->physics.inertiaTensorWorldInv * rtA, rA)) +
        glm::dot(tangent, glm::cross(B->physics.inertiaTensorWorldInv * rtB, rB));

    float jt = (denomT > 1e-8f) ? -glm::dot(rv, tangent) / denomT : 0.0f;
    jt = glm::clamp(jt, -std::abs(j) * mu, std::abs(j) * mu);

    glm::vec3 frictionImpulse = jt * tangent;
    if (!A->physics.isStatic) ApplyImpulse(*A, frictionImpulse, c.point);
    if (!B->physics.isStatic) ApplyImpulse(*B, -frictionImpulse, c.point);

#ifdef Debug
    std::cout << "[Contact] j=" << j << " jt=" << jt << " penetration=" << c.penetration << "\n";
#endif
}


// Compute kinetic energy of a physical model (trans + rot)
float ComputeKineticEnergy(const PhysicalModel& pm) {
    if (pm.physics.isStatic) return 0.0f;
    float trans = 0.5f * pm.physics.mass * glm::dot(pm.physics.velocity, pm.physics.velocity);

    // rotational energy: 0.5 * w^T * I * w
    glm::vec3 w = pm.physics.angularVelocity;
    glm::vec3 Iw = pm.physics.inertiaTensorWorld * w;
    float rot = 0.5f * glm::dot(w, Iw);
    return trans + rot;
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

// -------------------- main physics loop --------------------

void RegisterPhysicalModel(ModelInstance& instance, const Material& mat, bool isStatic) {
    // prevent duplicate registration
    for (auto& pm : physicalModels) {
        if (pm.instance == &instance) return;
    }

    auto [volume_m3, centroid_d] = ComputeVolumeAndCentroid(instance);
    double absVolume = std::max(std::abs(volume_m3), 1e-6); // avoid zero
    float mass = isStatic ? 0.0f : std::max((float)(absVolume * mat.density), 0.01f);

    // find lowest vertex in model space
    glm::vec3 minVert(std::numeric_limits<float>::infinity());
    for (size_t i = 0; i + 2 < instance.model.vertexCoords.size(); i += 3)
        minVert = glm::min(minVert, glm::vec3(instance.model.vertexCoords[i], instance.model.vertexCoords[i + 1], instance.model.vertexCoords[i + 2]));

    // adjust initial position so bottom sits at current y
    instance.position.y = float(instance.position.y) + (-float(minVert.y));

    PhysicalEntity phys;
    phys.instance = instance;
    phys.mass = mass;
    phys.material = mat;
    phys.isStatic = isStatic;
    phys.centerOfGravity = glm::vec3(centroid_d);
    phys.volumeCM3 = float(absVolume * 1e6);
    phys.velocity = phys.angularVelocity = phys.angularMomentum = glm::vec3(0.0f);
    phys.forces = phys.torque = glm::vec3(0.0f);

    // compute inertia tensor safely
    glm::mat3 I_local(0.0f);
    try {
        I_local = glm::mat3(ComputeInertiaTensor(instance, centroid_d, mat.density));
    }
    catch (...) {
        float r = std::max(0.1f, (float)std::cbrt(absVolume));
        float Ival = 0.4f * mass * r * r;
        I_local = glm::mat3(Ival);
    }
    phys.inertiaTensorLocal = I_local;
    phys.inertiaTensorLocalInv = glm::inverse(I_local + glm::mat3(1e-8f));
    phys.inertiaTensorWorld = phys.inertiaTensorWorldInv = I_local;

    physicalModels.push_back({ &instance, phys });

#ifdef Debug
    glm::vec3 centerWorld = ComputeCenterWorld(instance, glm::vec3(centroid_d));
    std::cout << "[Register] model at " << &instance
        << " mass=" << phys.mass
        << " centroid_local=(" << centroid_d.x << "," << centroid_d.y << "," << centroid_d.z << ")"
        << " centroid_world=(" << centerWorld.x << "," << centerWorld.y << "," << centerWorld.z << ")"
        << " isStatic=" << (isStatic ? "Yes" : "No") << "\n";
#endif
}

void UpdatePhysics(float deltaTime) {
    if (deltaTime <= 0.0f) return;

    const glm::vec3 gravity(0.0f, -9.81f, 0.0f);

    // --- 1) apply accumulated forces (F = m a) ---
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;

        if (glm::length(pm.physics.forces) > 0.0f && pm.physics.mass > 0.0f) {
            pm.physics.velocity += (pm.physics.forces / pm.physics.mass) * deltaTime;
            pm.physics.forces = glm::vec3(0.0f);
        }
        if (glm::length(pm.physics.torque) > 0.0f) {
            pm.physics.angularMomentum += pm.physics.torque * deltaTime;
            pm.physics.torque = glm::vec3(0.0f);
        }
    }

    // --- 2) determine substeps and apply gravity half-step ---
    int maxSubsteps = 1;
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;
        int substeps = ComputeSubsteps(pm, deltaTime);
        maxSubsteps = std::max(maxSubsteps, substeps);
    }

    float subDelta = deltaTime / maxSubsteps;

    for (int step = 0; step < maxSubsteps; ++step) {

        // --- 2a) gravity half-step ---
        for (auto& pm : physicalModels) {
            if (!pm.instance || pm.physics.isStatic) continue;
            pm.physics.velocity += gravity * 0.5f * subDelta;
        }

        // --- 2b) collision detection + CCD ---
        for (auto& pm : physicalModels) {
            if (!pm.instance || pm.physics.isStatic) continue;

            glm::vec3 hitPoint;
            PhysicalModel* hitModel = nullptr;
            if (CCDCheck(pm, subDelta, hitPoint, hitModel)) {
                if (hitModel) {
                    // resolve collision impulse immediately for CCD hit
                    Contact c;
                    c.A = &pm;
                    c.B = hitModel;
                    c.normal = glm::normalize(pm.instance->position - hitPoint);
                    c.penetration = 0.0f; // small offset already applied
                    ResolveContactImpulse(c);
                }
            } else {
                // normal semi-implicit integration if no CCD hit
                pm.instance->position += pm.physics.velocity * subDelta;
            }
        }

        // --- 2c) collision resolution for contacts ---
        auto pairs = BroadphaseGeneratePairs();
        std::vector<Contact> allContacts;
        for (auto& p : pairs) {
            auto cs = NarrowphaseGenerateContacts(p.A, p.B);
            allContacts.insert(allContacts.end(), cs.begin(), cs.end());
        }

        const int iterations = 8;
        const float k_slop = 0.01f;
        const float percent = 0.2f;
        const float maxCorrection = 0.02f;

        for (int it = 0; it < iterations; ++it) {
            for (auto& c : allContacts) {
                ResolveContactImpulse(c);

                float invMassA = (!c.A->physics.isStatic && c.A->physics.mass > 0.0f) ? 1.0f / c.A->physics.mass : 0.0f;
                float invMassB = (!c.B->physics.isStatic && c.B->physics.mass > 0.0f) ? 1.0f / c.B->physics.mass : 0.0f;
                float invMassSum = invMassA + invMassB;
                if (c.penetration > k_slop && invMassSum > 0.0f) {
                    float correction = glm::clamp((c.penetration - k_slop) / invMassSum * percent, -maxCorrection, maxCorrection);
                    if (!c.A->physics.isStatic) c.A->instance->position += correction * c.normal * invMassA;
                    if (!c.B->physics.isStatic) c.B->instance->position -= correction * c.normal * invMassB;
                }
            }
        }

        // --- 2d) finish gravity half-step ---
        for (auto& pm : physicalModels) {
            if (!pm.instance || pm.physics.isStatic) continue;
            pm.physics.velocity += gravity * 0.5f * subDelta;
        }
    }

    // --- 3) rotational dynamics ---
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;

        glm::quat q = glm::quat(glm::radians(pm.instance->rotation));
        glm::mat3 R = glm::mat3_cast(q);

        pm.physics.inertiaTensorWorld = R * pm.physics.inertiaTensorLocal * glm::transpose(R);
        glm::mat3 eps(0.0f); eps[0][0] = eps[1][1] = eps[2][2] = 1e-8f;
        pm.physics.inertiaTensorWorldInv = glm::inverse(pm.physics.inertiaTensorWorld + eps);

        pm.physics.angularVelocity = pm.physics.inertiaTensorWorldInv * pm.physics.angularMomentum;

        glm::quat wq(0.0f, pm.physics.angularVelocity.x, pm.physics.angularVelocity.y, pm.physics.angularVelocity.z);
        glm::quat qNew = glm::normalize(q + 0.5f * wq * q * deltaTime);
        pm.instance->rotation = glm::degrees(glm::eulerAngles(qNew));
    }

#ifdef Debug
    PrintPhysicsState();
#endif
}

// Pretty-print
void PrintPhysicsState() {
    std::cout << "====== Physics Debug Info ======" << std::endl;
    int i = 0;
    for (auto& obj : physicalModels) {
        if (!obj.instance) continue;
        glm::vec3 centerWorld = ComputeCenterWorld(*obj.instance, obj.physics.centerOfGravity);
        std::cout << "Object[" << i++ << "] mass=" << obj.physics.mass
            << " pos=(" << obj.instance->position.x << "," << obj.instance->position.y << "," << obj.instance->position.z << ")"
            << " com=(" << centerWorld.x << "," << centerWorld.y << "," << centerWorld.z << ")"
            << " vel=(" << obj.physics.velocity.x << "," << obj.physics.velocity.y << "," << obj.physics.velocity.z << ")"
            << " angVel=(" << obj.physics.angularVelocity.x << "," << obj.physics.angularVelocity.y << "," << obj.physics.angularVelocity.z << ")\n";
    }
    std::cout << "================================" << std::endl;
}

void UpdatePhysics(float deltaTime) {
    if (deltaTime <= 0.0f) return;

    const glm::vec3 gravity(0.0f, -9.81f, 0.0f);

    // --- 1) apply accumulated forces (F = m a) ---
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;

        if (glm::length(pm.physics.forces) > 0.0f && pm.physics.mass > 0.0f) {
            // Semi-implicit velocity update
            pm.physics.velocity += (pm.physics.forces / pm.physics.mass) * deltaTime;
            pm.physics.forces = glm::vec3(0.0f);
        }
        if (glm::length(pm.physics.torque) > 0.0f) {
            pm.physics.angularMomentum += pm.physics.torque * deltaTime;
            pm.physics.torque = glm::vec3(0.0f);
        }
    }

    // --- 2) preliminary gravity integration (for hybrid stability) ---
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;
        pm.physics.velocity += gravity * deltaTime * 0.5f; // half-step for hybrid
    }

    // --- 3) collision detection + iterative resolution ---
    auto pairs = BroadphaseGeneratePairs();
    std::vector<Contact> allContacts;
    for (auto& p : pairs) {
        auto cs = NarrowphaseGenerateContacts(p.A, p.B);
        allContacts.insert(allContacts.end(), cs.begin(), cs.end());
    }

    const int iterations = 8;
    const float k_slop = 0.01f;
    const float percent = 0.2f;
    const float maxCorrection = 0.02f;

    for (int it = 0; it < iterations; ++it) {
        for (auto& c : allContacts) {
            ResolveContactImpulse(c);

            float invMassA = (!c.A->physics.isStatic && c.A->physics.mass > 0.0f) ? 1.0f / c.A->physics.mass : 0.0f;
            float invMassB = (!c.B->physics.isStatic && c.B->physics.mass > 0.0f) ? 1.0f / c.B->physics.mass : 0.0f;
            float invMassSum = invMassA + invMassB;
            if (c.penetration > k_slop && invMassSum > 0.0f) {
                float correction = glm::clamp((c.penetration - k_slop) / invMassSum * percent, -maxCorrection, maxCorrection);
                if (!c.A->physics.isStatic) c.A->instance->position += correction * c.normal * invMassA;
                if (!c.B->physics.isStatic) c.B->instance->position -= correction * c.normal * invMassB;
            }
        }
    }

    // --- 4) linear integration (final velocity update + position) ---
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;

        // finish gravity step
        pm.physics.velocity += gravity * deltaTime * 0.5f;

        // hybrid: linear position semi-implicit
        pm.instance->position += pm.physics.velocity * deltaTime;
    }

    // --- 5) rotational dynamics (hybrid angular integration) ---
    for (auto& pm : physicalModels) {
        if (!pm.instance || pm.physics.isStatic) continue;

        glm::quat q = glm::quat(glm::radians(pm.instance->rotation));
        glm::mat3 R = glm::mat3_cast(q);

        pm.physics.inertiaTensorWorld = R * pm.physics.inertiaTensorLocal * glm::transpose(R);
        glm::mat3 eps(0.0f); eps[0][0] = eps[1][1] = eps[2][2] = 1e-8f;
        pm.physics.inertiaTensorWorldInv = glm::inverse(pm.physics.inertiaTensorWorld + eps);

        // angular velocity from angular momentum
        pm.physics.angularVelocity = pm.physics.inertiaTensorWorldInv * pm.physics.angularMomentum;

        // integrate orientation (hybrid: quaternion with half-step torque)
        glm::quat wq(0.0f, pm.physics.angularVelocity.x, pm.physics.angularVelocity.y, pm.physics.angularVelocity.z);
        glm::quat qNew = glm::normalize(q + 0.5f * wq * q * deltaTime);
        pm.instance->rotation = glm::degrees(glm::eulerAngles(qNew));
    }

#ifdef Debug
    PrintPhysicsState();
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

        for (size_t i = 0; i + 2 < indices.size(); i += 3) {
            uint32_t a = indices[i + 0];
            uint32_t b = indices[i + 1];
            uint32_t c = indices[i + 2];
            // bounds check
            if ((size_t)a * 3 + 2 >= verts.size() ||
                (size_t)b * 3 + 2 >= verts.size() ||
                (size_t)c * 3 + 2 >= verts.size()) continue;

            glm::vec3 v0 = glm::vec3(modelMat * glm::vec4(verts[a * 3 + 0], verts[a * 3 + 1], verts[a * 3 + 2], 1.0f));
            glm::vec3 v1 = glm::vec3(modelMat * glm::vec4(verts[b * 3 + 0], verts[b * 3 + 1], verts[b * 3 + 2], 1.0f));
            glm::vec3 v2 = glm::vec3(modelMat * glm::vec4(verts[c * 3 + 0], verts[c * 3 + 1], verts[c * 3 + 2], 1.0f));

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
        // store local offset relative to instance position (not COM)
        grabLocalOffset = hit - hitModel->instance->position;
        dragging = true;

        // Remember previous static state and make object static
        prevIsStatic = hitModel->physics.isStatic;
        hitModel->physics.isStatic = true;

#ifdef Debug
        std::cout << "[Physics] Grabbed object at "
            << "Pos(" << hitModel->instance->position.x << ", "
            << hitModel->instance->position.y << ", " << hitModel->instance->position.z << ") "
            << " | Hit (" << hit.x << ", " << hit.y << ", " << hit.z << ")\n";
#endif
    }
}

void OnRightClickReleased() {
    if (!dragging || !draggedModel) return;
    draggedModel->physics.velocity = glm::vec3(0.0f);
    draggedModel->physics.angularVelocity = glm::vec3(0.0f);

    // Restore original static state
    draggedModel->physics.isStatic = prevIsStatic;

    dragging = false;
    draggedModel = nullptr;
}

// Update dragging constraint with inertia based on mass
void UpdateDrag(const Camera& cam, double mouseX, double mouseY, int windowWidth, int windowHeight) {
    if (!dragging || !draggedModel) return;
    draggedModel->physics.velocity = glm::vec3(0.0f);
    draggedModel->physics.angularVelocity = glm::vec3(0.0f);

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

    float massFactor = 1.0f;
    if (draggedModel->physics.mass > 0.0f) massFactor = 1000.0f / std::sqrt(draggedModel->physics.mass); // tweak factor as needed
    draggedModel->instance->position = glm::mix(draggedModel->instance->position, newObjectPos, glm::clamp(0.2f * massFactor, 0.01f, 1.0f));
}
