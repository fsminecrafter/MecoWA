#include "jolt_world.h"
#include "jolt_init.h"
#include "jolt_shape_cache.h"

#include <Jolt/Physics/Body/BodyInterface.h>

struct PhysicalEntry {
    ModelInstance* inst;
    JPH::BodyID body;
};

static std::vector<PhysicalEntry> bodies;


void Physics_AddDynamic_Box(ModelInstance& inst)
{
    auto* bi = &gPhysics->GetBodyInterface();

    JPH::RefConst<JPH::Shape> shape = new JPH::BoxShape(
        JPH::Vec3(inst.scale.x * 0.5f, inst.scale.y * 0.5f, inst.scale.z * 0.5f)
    );

    JPH::BodyCreationSettings bcs(
        shape,
        JPH::Vec3(inst.position.x, inst.position.y, inst.position.z),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Dynamic,
        0 // object layer
    );

    auto mp = shape->GetMassProperties();
    mp.ScaleToMass(inst.mass);
    bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
    bcs.mMassPropertiesOverride = mp;

    auto body = bi->CreateBody(bcs);
    bi->AddBody(body->GetID(), JPH::EActivation::Activate);

    bodies.push_back({ &inst, body->GetID() });
}




void Physics_AddStatic_TriangleMesh(ModelInstance& inst)
{
    std::vector<JPH::Float3> verts;
    std::vector<uint32_t> idx;

    inst.GetTriangleMesh(verts, idx);

    auto shape = GetTriangleMeshShapeCached(verts, idx);

    auto* bi = &gPhysics->GetBodyInterface();

    JPH::BodyCreationSettings bcs(
        shape,
        JPH::Vec3(inst.position.x, inst.position.y, inst.position.z),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Static,
        0);

    auto body = bi->CreateBody(bcs);
    bi->AddBody(body->GetID(), JPH::EActivation::DontActivate);

    bodies.push_back({ &inst, body->GetID() });
}



void Physics_AddDynamic_ConvexHull(ModelInstance& inst)
{
    std::vector<JPH::Float3> verts;
    inst.GetConvexVertices(verts);

    auto shape = CreateConvexHullShape(verts);

    auto* bi = &gPhysics->GetBodyInterface();

    JPH::BodyCreationSettings bcs(
        shape,
        JPH::Vec3(inst.position.x, inst.position.y, inst.position.z),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Dynamic,
        0);

    auto body = bi->CreateBody(bcs);
    bi->AddBody(body->GetID(), JPH::EActivation::Activate);

    bodies.push_back({ &inst, body->GetID() });
}



void Physics_Update(float dt)
{
    gPhysics->Update(dt, 1, gJobs);

    auto& ro = gPhysics->GetBodyInterfaceReadOnly();

    for (auto& e : bodies)
    {
        auto* b = ro.GetBody(e.body);

        auto p = b->GetPosition();
        auto q = b->GetRotation();

        e.inst->position = glm::vec3(p.GetX(), p.GetY(), p.GetZ());
        glm::quat qq(q.GetW(), q.GetX(), q.GetY(), q.GetZ());
        e.inst->rotation = glm::degrees(glm::eulerAngles(qq));
    }
}
