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

    // Loop vertices correctly
    for (size_t vi = 0; vi < vertsA.size() / 3; vi++)
    {
        glm::vec3 pv = glm::vec3(
            MA * glm::vec4(
                vertsA[vi * 3 + 0],
                vertsA[vi * 3 + 1],
                vertsA[vi * 3 + 2],
                1.0f
            )
        );

        for (size_t t = 0; t < idxB.size(); t += 3)
        {
            glm::vec3 b0 = glm::vec3(MB * glm::vec4(
                vertsB[idxB[t] * 3 + 0],
                vertsB[idxB[t] * 3 + 1],
                vertsB[idxB[t] * 3 + 2], 1.0f));

            glm::vec3 b1 = glm::vec3(MB * glm::vec4(
                vertsB[idxB[t + 1] * 3 + 0],
                vertsB[idxB[t + 1] * 3 + 1],
                vertsB[idxB[t + 1] * 3 + 2], 1.0f));

            glm::vec3 b2 = glm::vec3(MB * glm::vec4(
                vertsB[idxB[t + 2] * 3 + 0],
                vertsB[idxB[t + 2] * 3 + 1],
                vertsB[idxB[t + 2] * 3 + 2], 1.0f));

            glm::vec3 triNormal = glm::normalize(glm::cross(b1 - b0, b2 - b0));
            glm::vec3 closest = ClosestPointOnTriangle(pv, b0, b1, b2);
            glm::vec3 diff = pv - closest;
            float dist = glm::length(diff);

            if (dist <= contactThreshold)
            {
                // orient normal correctly
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
    // Store pointer to instance (avoid dangling reference)

        // check already registered (avoid duplicates)
    for (auto& pm : physicalModels) {
        if (pm.instance == &instance) {
            std::cerr << "[RegisterPhysicalModel] Warning: instance already registered, ignoring.\n";
            return;
        }
    }

    // compute volume & centroid from mesh (robust)
    double volume_m3 = 0.0;
    glm::dvec3 centroid_d(0.0);
    try {
        auto res = ComputeVolumeAndCentroid(instance);
        volume_m3 = res.first;
        centroid_d = res.second;
    }
    catch (...) {
        // fallback safe behavior
        std::cerr << "[RegisterPhysicalModel] ComputeVolumeAndCentroid threw, falling back.\n";
    }

    double absVolume = std::abs(volume_m3);
    // sensible fallback volume if mesh failed (not 1e-6 m^3 unless you want that)
    const double fallbackVolume = 1e-3; // 1 liter ~ 0.001 m^3 => heavier fallback
    if (absVolume < 1e-12) absVolume = fallbackVolume;

    float mass = (float)(absVolume * (double)mat.density);

    // clamp minimal mass to avoid singular physics
    const float minMass = 0.01f; // 10 grams minimum
    if (mass < minMass && !isStatic) mass = minMass;

    PhysicalEntity phys;
    phys.instance = instance; // store pointer to the real instance
    phys.mass = isStatic ? std::numeric_limits<float>::infinity() : mass;
    phys.material = mat;
    phys.velocity = glm::vec3(0.0f);
    phys.angularVelocity = glm::vec3(0.0f);
    phys.centerOfGravity = glm::vec3(centroid_d); // use computed centroid
    phys.volumeCM3 = (float)(absVolume * 1e6);    // m^3 -> cm^3
    phys.isStatic = isStatic;

    glm::mat3 I_local(0.0f);
    bool haveInertia = false;
    try {
        glm::dmat3 I_d = ComputeInertiaTensor(instance, glm::dvec3(centroid_d), mat.density);
        I_local = glm::mat3(I_d);
        haveInertia = true;
    }
    catch (...) { haveInertia = false; }

    if (!haveInertia) {
        float r = std::max(0.1f, (float)std::cbrt(absVolume)); // heuristic radius
        float Ival = 0.4f * (isStatic ? 1.0f : phys.mass) * r * r;
        I_local = glm::mat3(Ival);
    }
    phys.inertiaTensorLocal = I_local;
    // safe inverse (avoid singular)
    glm::mat3 safe = phys.inertiaTensorLocal + glm::mat3(1e-8f);
    phys.inertiaTensorLocalInv = glm::inverse(safe);

    phys.angularMomentum = glm::vec3(0.0f);
    phys.forces = glm::vec3(0.0f);
    phys.torque = glm::vec3(0.0f);

    physicalModels.push_back({ &instance, phys });

#ifdef Debug
    std::cout << std::fixed << std::setprecision(6)
        << "[Register] model at " << &instance
        << " | vol(m^3)=" << absVolume
        << " | vol(cm^3)=" << (absVolume * 1e6)
        << " | centroid=(" << centroid_d.x << "," << centroid_d.y << "," << centroid_d.z << ")"
        << " | mass(kg)=" << phys.mass
        << " | isStatic=" << (isStatic ? "Yes" : "No")
        << "\n";
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

#ifdef Debug
    std::cout << "\n[Physics Debug] ==============================\n";

    int index = 0;
    for (auto& pm : physicalModels)
    {
        const auto& P = pm.physics;     // shortcut
        const auto& T = *pm.instance;   // ModelInstance transform

        float KE = ComputeKineticEnergy(pm);

        std::cout
            << "[Object " << index << "]\n"
            << "  Pos:      (" << T.position.x << ", " << T.position.y << ", " << T.position.z << ")\n"
            << "  Rot:      (" << T.rotation.x << ", " << T.rotation.y << ", " << T.rotation.z << ")\n"
            << "  Vel:      (" << P.velocity.x << ", " << P.velocity.y << ", " << P.velocity.z << ")\n"
            << "  Ang Vel:  (" << P.angularVelocity.x << ", " << P.angularVelocity.y << ", " << P.angularVelocity.z << ")\n"
            << "  Mass:      " << P.mass << "\n"
            << "  Kinetic E: " << KE << "\n\n";

        index++;
    }

    std::cout << "==============================================\n";
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
            size_t i0 = indices[i] * 3;
            size_t i1 = indices[i + 1] * 3;
            size_t i2 = indices[i + 2] * 3;
            if (i0 + 2 >= verts.size() || i1 + 2 >= verts.size() || i2 + 2 >= verts.size()) continue;

            glm::vec3 v0 = glm::vec3(modelMat * glm::vec4(verts[i0 + 0], verts[i0 + 1], verts[i0 + 2], 1.0f));
            glm::vec3 v1 = glm::vec3(modelMat * glm::vec4(verts[i1 + 0], verts[i1 + 1], verts[i1 + 2], 1.0f));
            glm::vec3 v2 = glm::vec3(modelMat * glm::vec4(verts[i2 + 0], verts[i2 + 1], verts[i2 + 2], 1.0f));

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
