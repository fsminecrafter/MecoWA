#pragma once
#include <vector>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/quaternion.hpp>

struct ModelInstance;
struct SceneCollider;

// legacy
void Physics_AddDynamic_Box(ModelInstance& inst);
void Physics_AddStatic_TriangleMesh(ModelInstance& inst);
void Physics_AddDynamic_ConvexHull(ModelInstance& inst);

// new
// colliders  = list of SceneCollider (may be empty -> single bounding-box fallback)
// isStatic   = true for static world geometry
// mass       = kg, ignored when isStatic=true
// friction / restitution = surface properties
void Physics_AddWithColliders(
    ModelInstance& inst,
    const std::vector<SceneCollider>& colliders,
    bool                              isStatic,
    float                             mass = 1.f,
    float                             friction = 0.5f,
    float                             restitution = 0.1f);

// Sync Jolt body transforms back to ModelInstances
void Physics_SyncToEngine();

// Fixed-step physics tick  (called by PhysicsTick_Accumulate)
void Physics_Update(float dt);