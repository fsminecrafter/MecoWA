#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Body/Body.h>

#include "engine.h"

using namespace JPH;

struct PhysicsLink
{
    ModelInstance* model;
    BodyID body;
    glm::vec3 renderOffset; // model origin -> physics center
};

static std::vector<PhysicsLink> gPhysicsLinks;
extern JPH::PhysicsSystem* gPhysics;