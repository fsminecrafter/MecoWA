#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>

extern JPH::PhysicsSystem* gPhysics;
extern JPH::JobSystemThreadPool* gJobs;

void Jolt_Init();
void ShutdownJolt();
