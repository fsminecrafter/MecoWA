#include "jolt_world.h"
#include "jolt_init.h"
#include "jolt_shape_cache.h"

#include <vector>

// Jolt core
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

// Shapes
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>

#include "engine.h"

using namespace JPH;

// === STUB IMPLEMENTATION FOR WATER/BUOYANCY (NOT USED) ===
// This is required because ConvexShape::GetSubmergedVolume is pure virtual
// and our Jolt library was compiled without full implementation.
// Since we're not using water physics, we provide empty stubs.
namespace JPH {
    void ConvexShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane& inSurface, float& outTotalVolume, float& outSubmergedVolume, Vec3& outCenterOfBuoyancy
#ifdef JPH_DEBUG_RENDERER
        , RVec3Arg inBaseOffset
#endif
    ) const
    {
        // Not implementing water physics - return zero submersion
        outTotalVolume = 0.0f;
        outSubmergedVolume = 0.0f;
        outCenterOfBuoyancy = Vec3::sZero();
    }
}
// === END STUB ===

struct PhysicalEntry
{
    ModelInstance* inst;
    BodyID body;
};

static std::vector<PhysicalEntry> gBodies;

////////////////////////////////////////////////////////////
// Dynamic box
////////////////////////////////////////////////////////////
void Physics_AddDynamic_Box(ModelInstance& inst)
{
    BodyInterface& bi = gPhysics->GetBodyInterface();

    RefConst<Shape> shape = new BoxShape(
        Vec3(inst.scale.x * 0.5f,
            inst.scale.y * 0.5f,
            inst.scale.z * 0.5f));

    BodyCreationSettings bcs(
        shape,
        Vec3(inst.position.x, inst.position.y, inst.position.z),
        Quat::sIdentity(),
        EMotionType::Dynamic,
        0
    );

    bcs.mFriction = 0.8f;
    bcs.mRestitution = 0.1f;

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::Activate);
    gBodies.push_back({ &inst, body });
}

////////////////////////////////////////////////////////////
// Static triangle mesh
////////////////////////////////////////////////////////////
void Physics_AddStatic_TriangleMesh(ModelInstance& inst)
{
    std::vector<Float3> verts;
    std::vector<uint32_t> indices;
    
    // Convert vertex data to Float3 format
    for (size_t i = 0; i < inst.vertexData.size(); i += 3)
    {
        verts.push_back(Float3(inst.vertexData[i], inst.vertexData[i+1], inst.vertexData[i+2]));
    }
    
    // Copy element data
    indices = inst.elementData;

    RefConst<Shape> shape = GetTriangleMeshShapeCached(verts, indices);

    BodyInterface& bi = gPhysics->GetBodyInterface();

    BodyCreationSettings bcs(
        shape,
        Vec3(inst.position.x, inst.position.y, inst.position.z),
        Quat::sIdentity(),
        EMotionType::Static,
        0
    );

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::DontActivate);
    gBodies.push_back({ &inst, body });
}

////////////////////////////////////////////////////////////
// Dynamic convex hull
////////////////////////////////////////////////////////////
void Physics_AddDynamic_ConvexHull(ModelInstance& inst)
{
    std::vector<Float3> verts;
    
    // Convert vertex data to Float3 format
    for (size_t i = 0; i < inst.vertexData.size(); i += 3)
    {
        verts.push_back(Float3(inst.vertexData[i], inst.vertexData[i+1], inst.vertexData[i+2]));
    }

    Array<Vec3> points;
    points.reserve(verts.size());
    for (const auto& v : verts)
        points.push_back(Vec3(v[0], v[1], v[2]));

    ConvexHullShapeSettings hullSettings(points);
    ShapeSettings::ShapeResult result = hullSettings.Create();

    if (result.HasError())
        return;

    RefConst<Shape> shape = result.Get();

    BodyInterface& bi = gPhysics->GetBodyInterface();

    BodyCreationSettings bcs(
        shape,
        Vec3(inst.position.x, inst.position.y, inst.position.z),
        Quat::sIdentity(),
        EMotionType::Dynamic,
        0
    );

    bcs.mFriction = 0.6f;

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::Activate);
    gBodies.push_back({ &inst, body });
}

////////////////////////////////////////////////////////////
// Physics update -> scene sync
////////////////////////////////////////////////////////////
void Physics_Update(float dt)
{
    TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
    gPhysics->Update(dt, 1, &temp_allocator, gJobs);

    BodyInterface& bi = gPhysics->GetBodyInterface();

    for (auto& e : gBodies)
    {
        BodyLockRead lock(gPhysics->GetBodyLockInterface(), e.body);
        if (!lock.Succeeded())
            continue;

        const Body& body = lock.GetBody();

        Vec3 p = body.GetPosition();
        Quat q = body.GetRotation();

        e.inst->position = glm::vec3(p[0], p[1], p[2]);

        glm::quat rot(q.GetW(), q.GetX(), q.GetY(), q.GetZ());
        e.inst->rotation = glm::degrees(glm::eulerAngles(rot));
    }
}
