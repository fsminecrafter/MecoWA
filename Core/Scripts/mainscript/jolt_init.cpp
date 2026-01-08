#include "jolt_init.h"
#include "jolt_layers.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>

#include "mecowa.h"

using namespace JPH;

PhysicsSystem* gPhysics = nullptr;
JobSystemThreadPool* gJobs = nullptr;

void Jolt_Init()
{
    RegisterDefaultAllocator();

    Factory::sInstance = new Factory();
    RegisterTypes();

    // --- Job system FIRST ---
    uint32_t hw_threads = std::thread::hardware_concurrency();
    uint32_t worker_threads = hw_threads > 1 ? hw_threads - 1 : 1;

    gJobs = new JobSystemThreadPool(
        cMaxPhysicsJobs,
        cMaxPhysicsBarriers,
        worker_threads
    );

    if (!gJobs)
		joltinitsuccess = false;

    // --- Physics system AFTER ---
    gPhysics = new PhysicsSystem();

    if (!gPhysics)
        joltinitsuccess = false;

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

    joltinitsuccess = true;
}

void ShutdownJolt()
{
    delete gPhysics;
    gPhysics = nullptr;

    delete gJobs;
    gJobs = nullptr;

    delete Factory::sInstance;
    Factory::sInstance = nullptr;
}
