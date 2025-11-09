#pragma once
#include "engine.h"
#include "physicsDefine.h"
#include <vector>

struct PhysicalModel {
    ModelInstance* instance;
    PhysicalEntity physics;
};

extern std::vector<PhysicalModel> physicalModels;

void RegisterPhysicalModel(ModelInstance& instance, const Material& mat);
void UpdatePhysics(float deltaTime);
void PrintPhysicsState();
void UpdateDrag(const Camera& cam, double mouseX, double mouseY, int windowWidthz, int windowHeighty);