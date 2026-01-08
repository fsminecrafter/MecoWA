#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

// Include the headers that define ObjectLayer / BroadPhaseLayer
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

#include <iostream>
#include <cstdarg>
#include <thread>

#include "mecowa.h"

using namespace JPH;

// Global physics system and job pool
inline PhysicsSystem* gPhysics = nullptr;
inline JobSystemThreadPool* gJobs = nullptr;

namespace Layers
{
    inline constexpr ObjectLayer NON_MOVING = 0;
    inline constexpr ObjectLayer MOVING = 1;
    inline constexpr ObjectLayer NUM_LAYERS = 2;
};

// Determines if two object layers should collide
class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
    bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
    {
        switch (inObject1)
        {
        case Layers::NON_MOVING: return inObject2 == Layers::MOVING;
        case Layers::MOVING:     return true;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};

namespace BroadPhaseLayers
{
    inline constexpr BroadPhaseLayer NON_MOVING{ 0 };
    inline constexpr BroadPhaseLayer MOVING{ 1 };
    inline constexpr uint NUM_LAYERS = 2;
};

// Maps object layers to broadphase layers
class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
    BPLayerInterfaceImpl()
    {
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }

    uint GetNumBroadPhaseLayers() const override
    {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
    {
        JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
    {
        switch ((BroadPhaseLayer::Type)inLayer)
        {
        case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING: return "NON_MOVING";
        case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:     return "MOVING";
        default:                                                   JPH_ASSERT(false); return "INVALID";
        }
    }
#endif

private:
    BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

// Determines if object layer collides with broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
    {
        switch (inLayer1)
        {
        case Layers::NON_MOVING: return inLayer2 == BroadPhaseLayers::MOVING;
        case Layers::MOVING:     return true;
        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};

static void TraceImpl(const char* inFMT, ...)
{
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);
    std::cout << "[Jolt] " << buffer << std::endl;
}

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailedImpl(const char* inExpression, const char* inMessage, const char* inFile, uint inLine)
{
    std::cout << "[Jolt Assert] " << inFile << ":" << inLine
        << ": (" << inExpression << ") "
        << (inMessage ? inMessage : "") << std::endl;
    return true;
}
#endif

void Jolt_Init()
{
    RegisterDefaultAllocator();

    Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;);

    if (!Factory::sInstance)
        Factory::sInstance = new Factory();

    RegisterTypes();

    uint32_t hw_threads = std::thread::hardware_concurrency();
    uint32_t worker_threads = hw_threads > 1 ? hw_threads - 1 : 1;
    gJobs = new JobSystemThreadPool(cMaxPhysicsJobs, cMaxPhysicsBarriers, worker_threads);

    static BPLayerInterfaceImpl bpl;
    static ObjectVsBroadPhaseLayerFilterImpl ovbpl;
    static ObjectLayerPairFilterImpl olpf;

    gPhysics = new PhysicsSystem();
    gPhysics->Init(
        65536,  // max bodies
        0,      // max body pairs
        65536,  // max body mutexes
        10240,  // max contact constraints
        bpl,
        ovbpl,
        olpf
    );

    std::cout << "[Jolt] Physics system initialized successfully.\n";
}

inline void ShutdownJolt()
{
    delete gPhysics;
    gPhysics = nullptr;

    delete gJobs;
    gJobs = nullptr;

    delete Factory::sInstance;
    Factory::sInstance = nullptr;
}
