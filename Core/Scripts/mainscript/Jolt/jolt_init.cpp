#include "jolt_init.h"

JPH::PhysicsSystem* gPhysics = nullptr;
JPH::JobSystemThreadPool* gJobs = nullptr;

void InitJolt()
{
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    gJobs = new JPH::JobSystemThreadPool(4, 4);

    gPhysics = new JPH::PhysicsSystem();
    gPhysics->Init(
        50'000,
        0,
        20'000,
        20'000,
        nullptr
    );
}

void ShutdownJolt()
{
    delete gPhysics;
    delete gJobs;
    delete JPH::Factory::sInstance;
}
