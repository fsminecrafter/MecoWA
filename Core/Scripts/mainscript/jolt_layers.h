#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

using namespace JPH;

namespace Layers
{
    inline constexpr ObjectLayer NON_MOVING = 0;
    inline constexpr ObjectLayer MOVING = 1;
    inline constexpr ObjectLayer NUM_LAYERS = 2;
};