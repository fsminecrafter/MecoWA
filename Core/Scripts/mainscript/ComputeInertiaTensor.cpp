#include <glm/glm.hpp>
#include "ComputeInertiaTensor.h"
#include <vector>
#include <cmath>

/*
   Compute inertia tensor of a closed mesh around a given center of gravity.
   Uses the standard Mirtich 1996 method:
   "Fast and Accurate Computation of Polyhedral Mass Properties".
*/

static void PolyInertia_AddTriangle(
    const glm::dvec3& v0,
    const glm::dvec3& v1,
    const glm::dvec3& v2,
    double& T0, double& T1, double& T2,
    double& TP0, double& TP1, double& TP2,
    double& TQ0, double& TQ1, double& TQ2)
{
    auto f1 = [&](double x0, double x1, double x2) {
        return x0 + x1 + x2;
        };
    auto f2 = [&](double x0, double x1, double x2) {
        return x0 * x0 + x1 * x1 + x2 * x2 + x0 * x1 + x1 * x2 + x2 * x0;
        };
    auto f3 = [&](double x0, double x1, double x2) {
        return x0 * x0 * x0 + x1 * x1 * x1 + x2 * x2 * x2
            + x0 * x0 * (x1 + x2) + x1 * x1 * (x2 + x0) + x2 * x2 * (x0 + x1)
            + x0 * x1 * x2;
        };

    glm::dvec3 a = v0;
    glm::dvec3 b = v1;
    glm::dvec3 c = v2;

    // cross product gives 2 * signed area vector
    glm::dvec3 cross = glm::cross(b - a, c - a);

    double nx = cross.x;
    double ny = cross.y;
    double nz = cross.z;

    double x0 = a.x, x1p = b.x, x2p = c.x;
    double y0 = a.y, y1p = b.y, y2p = c.y;
    double z0 = a.z, z1p = b.z, z2p = c.z;

    // Moments and products integrals
    T0 += nx * f1(x0, x1p, x2p);
    T1 += ny * f1(y0, y1p, y2p);
    T2 += nz * f1(z0, z1p, z2p);

    TP0 += nx * f2(x0, x1p, x2p);
    TP1 += ny * f2(y0, y1p, y2p);
    TP2 += nz * f2(z0, z1p, z2p);

    TQ0 += nx * f3(x0, x1p, x2p);
    TQ1 += ny * f3(y0, y1p, y2p);
    TQ2 += nz * f3(z0, z1p, z2p);
}


glm::dmat3 ComputeInertiaTensor(
    const ModelInstance& inst,
    const glm::dvec3& cg,
    double density)
{
    const auto& verts = inst.model.vertexCoords;   // vector<float>
    const auto& tris = inst.model.elementArray;   // vector<unsigned int>

    // Mirtich integral accumulators
    double T0 = 0, T1 = 0, T2 = 0;
    double TP0 = 0, TP1 = 0, TP2 = 0;
    double TQ0 = 0, TQ1 = 0, TQ2 = 0;

    for (size_t i = 0; i < tris.size(); i += 3) {
        unsigned i0 = tris[i + 0];
        unsigned i1 = tris[i + 1];
        unsigned i2 = tris[i + 2];

        glm::dvec3 v0(
            verts[i0 * 3 + 0],
            verts[i0 * 3 + 1],
            verts[i0 * 3 + 2]);

        glm::dvec3 v1(
            verts[i1 * 3 + 0],
            verts[i1 * 3 + 1],
            verts[i1 * 3 + 2]);

        glm::dvec3 v2(
            verts[i2 * 3 + 0],
            verts[i2 * 3 + 1],
            verts[i2 * 3 + 2]);

        PolyInertia_AddTriangle(
            v0, v1, v2,
            T0, T1, T2,
            TP0, TP1, TP2,
            TQ0, TQ1, TQ2);
    }


    // Final mass moments
    double mass = density * T0 / 6.0;

    // Inertia about origin
    double Ixx = density * (TP1 + TP2) / 60.0;
    double Iyy = density * (TP2 + TP0) / 60.0;
    double Izz = density * (TP0 + TP1) / 60.0;

    double Ixy = -density * TQ0 / 120.0;
    double Iyz = -density * TQ1 / 120.0;
    double Izx = -density * TQ2 / 120.0;

    glm::dmat3 I_origin = {
        { Ixx, Ixy, Izx },
        { Ixy, Iyy, Iyz },
        { Izx, Iyz, Izz }
    };

    // Shift to center of gravity (parallel axis theorem)
    glm::dvec3 r = cg;

    glm::dmat3 P = mass * glm::dmat3(
        r.y * r.y + r.z * r.z, -r.x * r.y, -r.x * r.z,
        -r.y * r.x, r.x * r.x + r.z * r.z, -r.y * r.z,
        -r.z * r.x, -r.z * r.y, r.x * r.x + r.y * r.y
    );

    glm::dmat3 I_cg = I_origin - P;

    return I_cg;
}
