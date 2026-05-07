#include "jolt_world.h"
#include "jolt_init.h"
#include "jolt_shape_cache.h"
#include "scene_file.h"   // SceneCollider, ColliderShape

#include <vector>
#include <iostream>

// Jolt core
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

// Shapes
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>

#include "engine.h"
#include "jolt_layers.h"

using namespace JPH;

//  Internal registry
struct PhysicalEntry
{
    ModelInstance* inst;
    BodyID         body;
    glm::vec3      renderOffset; // model origin -> physics center (world)
};

static std::vector<PhysicalEntry> gBodies;

//  Helper: build a single shape from a SceneCollider descriptor
static RefConst<Shape> BuildColliderShape(const SceneCollider& col,
    const ModelInstance& inst)
{
    switch (col.shape)
    {
    case ColliderShape::Box:
        return new BoxShape(Vec3(
            std::max(col.sx * inst.scale.x, 0.001f),
            std::max(col.sy * inst.scale.y, 0.001f),
            std::max(col.sz * inst.scale.z, 0.001f)));

    case ColliderShape::Sphere:
        return new SphereShape(std::max(col.sx * inst.scale.x, 0.001f));

    case ColliderShape::Capsule:
        return new CapsuleShape(
            std::max(col.sy * inst.scale.y, 0.001f),  // half-height
            std::max(col.sx * inst.scale.x, 0.001f)); // radius

    case ColliderShape::ConvexHull:
    {
        // Build from mesh vertices scaled by instance scale
        std::vector<Float3> verts;
        verts.reserve(inst.model.vertexCoords.size() / 3);
        for (size_t i = 0; i + 2 < inst.model.vertexCoords.size(); i += 3)
            verts.push_back(Float3(
                inst.model.vertexCoords[i + 0] * inst.scale.x,
                inst.model.vertexCoords[i + 1] * inst.scale.y,
                inst.model.vertexCoords[i + 2] * inst.scale.z));
        if (verts.empty()) return nullptr;
        return CreateConvexHullShape(verts);
    }

    case ColliderShape::TriangleMesh:
    {
        std::vector<Float3> verts;
        std::vector<uint32_t> idx;
        for (size_t i = 0; i + 2 < inst.model.vertexCoords.size(); i += 3)
            verts.push_back(Float3(
                inst.model.vertexCoords[i + 0] * inst.scale.x,
                inst.model.vertexCoords[i + 1] * inst.scale.y,
                inst.model.vertexCoords[i + 2] * inst.scale.z));
        idx = std::vector<uint32_t>(inst.model.elementArray.begin(),
            inst.model.elementArray.end());
        if (verts.empty() || idx.empty()) return nullptr;
        return GetTriangleMeshShapeCached(verts, idx);
    }
    }
    return nullptr;
}

//  Helper: wrap a shape with an offset/rotation transform if needed
static RefConst<Shape> WrapWithOffset(RefConst<Shape> inner, const SceneCollider& col)
{
    if (!inner) return nullptr;

    bool hasPos = (col.ox != 0.f || col.oy != 0.f || col.oz != 0.f);
    bool hasRot = (col.rx != 0.f || col.ry != 0.f || col.rz != 0.f);

    if (!hasPos && !hasRot) return inner;

    Quat rot = Quat::sEulerAngles(Vec3(
        glm::radians(col.rx),
        glm::radians(col.ry),
        glm::radians(col.rz)));

    RotatedTranslatedShapeSettings rts(
        Vec3(col.ox, col.oy, col.oz),
        rot,
        inner);

    auto res = rts.Create();
    if (res.HasError())
    {
        std::cerr << "[JoltWorld] RotatedTranslatedShape error: " << res.GetError() << "\n";
        return inner; // fallback – use without offset
    }
    return res.Get();
}

//  Build the final compound/single shape for an instance + its colliders
static RefConst<Shape> BuildCompoundShape(
    const ModelInstance& inst,
    const std::vector<SceneCollider>& colliders,
    bool                              isStatic)
{
    // Filter active, non-trigger colliders
    std::vector<const SceneCollider*> active;
    for (auto& c : colliders)
        if (c.enabled && !c.isTrigger) active.push_back(&c);

    if (active.empty())
    {
        // Fallback: single bounding box
        SceneCollider fb;
        fb.shape = ColliderShape::Box;
        fb.sx = inst.scale.x * 0.5f;
        fb.sy = inst.scale.y * 0.5f;
        fb.sz = inst.scale.z * 0.5f;
        return BuildColliderShape(fb, inst);
    }

    if (active.size() == 1)
    {
        // Single shape – no compound wrapper needed
        auto inner = BuildColliderShape(*active[0], inst);
        return WrapWithOffset(inner, *active[0]);
    }

    // Multiple shapes -> StaticCompoundShape (or MutableCompoundShape for dynamic)
    if (isStatic)
    {
        StaticCompoundShapeSettings compound;
        for (auto* col : active)
        {
            auto inner = BuildColliderShape(*col, inst);
            if (!inner) continue;

            Quat rot = Quat::sEulerAngles(Vec3(
                glm::radians(col->rx),
                glm::radians(col->ry),
                glm::radians(col->rz)));

            compound.AddShape(Vec3(col->ox, col->oy, col->oz), rot, inner);
        }
        auto res = compound.Create();
        if (res.HasError())
        {
            std::cerr << "[JoltWorld] StaticCompound error: " << res.GetError() << "\n";
            return nullptr;
        }
        return res.Get();
    }
    else
    {
        MutableCompoundShapeSettings compound;
        for (auto* col : active)
        {
            auto inner = BuildColliderShape(*col, inst);
            if (!inner) continue;

            Quat rot = Quat::sEulerAngles(Vec3(
                glm::radians(col->rx),
                glm::radians(col->ry),
                glm::radians(col->rz)));

            compound.AddShape(Vec3(col->ox, col->oy, col->oz), rot, inner);
        }
        auto res = compound.Create();
        if (res.HasError())
        {
            std::cerr << "[JoltWorld] MutableCompound error: " << res.GetError() << "\n";
            return nullptr;
        }
        return res.Get();
    }
}

//  Legacy single-shape helpers (kept for backward compat)
void Physics_AddDynamic_Box(ModelInstance& inst)
{
    SceneCollider col;
    col.shape = ColliderShape::Box;
    col.sx = inst.scale.x * 0.5f;
    col.sy = inst.scale.y * 0.5f;
    col.sz = inst.scale.z * 0.5f;

    auto shape = BuildColliderShape(col, inst);
    if (!shape) return;

    BodyInterface& bi = gPhysics->GetBodyInterface();
    BodyCreationSettings bcs(
        shape,
        Vec3(inst.position.x, inst.position.y, inst.position.z),
        Quat::sIdentity(),
        EMotionType::Dynamic,
        Layers::MOVING);
    bcs.mFriction = 0.8f;
    bcs.mRestitution = 0.1f;

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::Activate);
    gBodies.push_back({ &inst, body, glm::vec3(0) });
}

void Physics_AddStatic_TriangleMesh(ModelInstance& inst)
{
    std::vector<Float3>   verts;
    std::vector<uint32_t> indices;

    for (size_t i = 0; i + 2 < inst.model.vertexCoords.size(); i += 3)
        verts.push_back(Float3(
            inst.model.vertexCoords[i + 0] * inst.scale.x,
            inst.model.vertexCoords[i + 1] * inst.scale.y,
            inst.model.vertexCoords[i + 2] * inst.scale.z));

    indices = std::vector<uint32_t>(
        inst.model.elementArray.begin(), inst.model.elementArray.end());

    auto shape = GetTriangleMeshShapeCached(verts, indices);
    if (!shape) return;

    BodyInterface& bi = gPhysics->GetBodyInterface();
    BodyCreationSettings bcs(
        shape,
        Vec3(inst.position.x, inst.position.y, inst.position.z),
        Quat::sIdentity(),
        EMotionType::Static,
        Layers::NON_MOVING);

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::DontActivate);
    gBodies.push_back({ &inst, body, glm::vec3(0) });
}

void Physics_AddDynamic_ConvexHull(ModelInstance& inst)
{
    std::vector<Float3> verts;
    for (size_t i = 0; i + 2 < inst.model.vertexCoords.size(); i += 3)
        verts.push_back(Float3(
            inst.model.vertexCoords[i + 0] * inst.scale.x,
            inst.model.vertexCoords[i + 1] * inst.scale.y,
            inst.model.vertexCoords[i + 2] * inst.scale.z));

    if (verts.empty()) return;
    auto shape = CreateConvexHullShape(verts);
    if (!shape) return;

    BodyInterface& bi = gPhysics->GetBodyInterface();
    BodyCreationSettings bcs(
        shape,
        Vec3(inst.position.x, inst.position.y, inst.position.z),
        Quat::sIdentity(),
        EMotionType::Dynamic,
        Layers::MOVING);
    bcs.mFriction = 0.6f;

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::Activate);
    gBodies.push_back({ &inst, body, glm::vec3(0) });
}


// Register an instance with an explicit collider list
void Physics_AddWithColliders(
    ModelInstance& inst,
    const std::vector<SceneCollider>& colliders,
    bool                              isStatic,
    float                             mass,
    float                             friction,
    float                             restitution)
{
    auto shape = BuildCompoundShape(inst, colliders, isStatic);
    if (!shape)
    {
        std::cerr << "[JoltWorld] Physics_AddWithColliders: shape build failed for object.\n";
        return;
    }

    BodyInterface& bi = gPhysics->GetBodyInterface();

    EMotionType   motionType = isStatic ? EMotionType::Static : EMotionType::Dynamic;
    ObjectLayer   layer = isStatic ? Layers::NON_MOVING : Layers::MOVING;
    EActivation   activation = isStatic ? EActivation::DontActivate : EActivation::Activate;

    BodyCreationSettings bcs(
        shape,
        Vec3(inst.position.x, inst.position.y, inst.position.z),
        Quat::sIdentity(),
        motionType,
        layer);

    bcs.mFriction = friction;
    bcs.mRestitution = restitution;

    if (!isStatic && mass > 0.f)
    {
        bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        bcs.mMassPropertiesOverride.mMass = mass;
    }

    BodyID body = bi.CreateAndAddBody(bcs, activation);
    if (body.IsInvalid())
    {
        std::cerr << "[JoltWorld] Physics_AddWithColliders: body creation failed.\n";
        return;
    }

    gBodies.push_back({ &inst, body, glm::vec3(0) });

    std::cout << "[JoltWorld] Registered body with "
        << colliders.size() << " collider(s), static=" << isStatic
        << ", mass=" << mass << "\n";
}

//  Sync Jolt body transforms -> engine ModelInstances
void Physics_SyncToEngine()
{
    if (!gPhysics) return;
    BodyInterface& bi = gPhysics->GetBodyInterface();

    for (auto& entry : gBodies)
    {
        if (!entry.inst || entry.body.IsInvalid()) continue;

        // Only sync active (awake or just-sleeping) dynamic bodies
        EBodyType type = bi.GetBodyType(entry.body);
        if (type != EBodyType::RigidBody) continue;

        RVec3 pos = bi.GetCenterOfMassPosition(entry.body);
        Quat  rot = bi.GetRotation(entry.body);

        entry.inst->position = glm::vec3((float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ());

        // Convert Jolt quaternion -> GLM Euler (degrees)
        glm::quat q((float)rot.GetW(), (float)rot.GetX(), (float)rot.GetY(), (float)rot.GetZ());
        entry.inst->rotation = glm::degrees(glm::eulerAngles(q));
    }
}

//  Physics_Update  (called by the fixed-timestep accumulator)
void Physics_Update(float dt)
{
    if (!gPhysics || !gJobs) return;

    // 10 MB temp allocator – plenty for 120 Hz steps
    TempAllocatorImpl temp_allocator(10 * 1024 * 1024);

    // collisionSteps=1: we're already at a small fixed dt
    gPhysics->Update(dt, 1, &temp_allocator, gJobs);

    Physics_SyncToEngine();
} 