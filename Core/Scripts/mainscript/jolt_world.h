#pragma once
#include <vector>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/quaternion.hpp>

struct ModelInstance;

void Physics_AddDynamic_Box(ModelInstance& inst);
void Physics_AddStatic_TriangleMesh(ModelInstance& inst);

void Physics_AddDynamic_ConvexHull(ModelInstance& inst);

void Physics_Update(float dt);
