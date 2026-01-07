#include "jolt_init.h"
#include "jolt_layers.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>

using namespace JPH;

PhysicsSystem* gPhysics = nullptr;
JobSystemThreadPool* gJobs = nullptr;

void Jolt_Init()
{
    RegisterDefaultAllocator();
    Factory::sInstance = new Factory();
    RegisterTypes();

    gPhysics = new PhysicsSystem();

    const uint maxBodies = 65536;
    const uint numBodyMutexes = 0;
    const uint maxBodyPairs = 65536;
    const uint maxContactConstraints = 10240;

    gPhysics->Init(
        maxBodies,
        numBodyMutexes,
        maxBodyPairs,
        maxContactConstraints,
        BroadPhaseLayerInterfaceImpl(),
        ObjectVsBroadPhaseLayerFilterImpl(),
        ObjectLayerPairFilterImpl()
    );

    gJobs = new JobSystemThreadPool(
        cMaxPhysicsJobs,
        cMaxPhysicsBarriers,
        std::thread::hardware_concurrency() - 1
    );
}


void ShutdownJolt()
{
    delete gPhysics;
    delete gJobs;
    delete JPH::Factory::sInstance;
}
