#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>

inline JPH::PhysicsSystem* gPhysics = nullptr;
inline JPH::JobSystemThreadPool* gJobs = nullptr;

void Jolt_Init();
void ShutdownJolt();
