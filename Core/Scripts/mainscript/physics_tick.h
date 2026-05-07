#pragma once
// physics_tick.h

#include <algorithm>

inline constexpr float PHYSICS_HZ = 120.f;   // simulation rate (Hz)
inline constexpr float PHYSICS_DT = 1.f / PHYSICS_HZ;
inline constexpr int   MAX_SUBSTEPS_FRAME = 8;        // spiral-of-death guard

void Physics_Update(float dt);

namespace PhysTickState {
    inline float accumulator = 0.f;
}

// Call once per rendered frame.
// rawDt     = real wall-clock delta (seconds)
// timeScale = from DebugUI_GetTimeScale()
inline void PhysicsTick_Accumulate(float rawDt, float timeScale)
{
    // Clamp raw delta to avoid huge spikes (e.g. debugger pause)
    float clampedDt = std::min(rawDt, 0.25f);

    PhysTickState::accumulator += clampedDt * timeScale;

    int steps = 0;
    while (PhysTickState::accumulator >= PHYSICS_DT && steps < MAX_SUBSTEPS_FRAME)
    {
        Physics_Update(PHYSICS_DT);
        PhysTickState::accumulator -= PHYSICS_DT;
        ++steps;
    }
    // Leftover stays in accumulator for the next frame.
}