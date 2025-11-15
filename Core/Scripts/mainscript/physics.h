#pragma once
#include "engine.h"
#include "physicsDefine.h"
#include <vector>

struct PhysicalModel {
    ModelInstance* instance;
    PhysicalEntity physics;
};

extern std::vector<PhysicalModel> physicalModels;

void RegisterPhysicalModel(ModelInstance& instance, const Material& mat, bool isstatic);
void UpdatePhysics(float deltaTime);
void PrintPhysicsState();

// Collision / impulse helpers
struct Contact {
    PhysicalModel* A;
    PhysicalModel* B;
    glm::vec3 point;      // world-space contact point (on surface)
    glm::vec3 normal;     // from A towards B (A->B)
    float penetration;    // positive penetration depth (or small negative tolerance)
};

std::vector<Contact> BroadphaseGeneratePairs();
std::vector<Contact> NarrowphaseGenerateContacts(PhysicalModel* A, PhysicalModel* B);
void ResolveContactImpulse(const Contact& c);
void ApplyImpulse(PhysicalModel& pm, const glm::vec3& impulse, const glm::vec3& contactPoint);
float ComputeKineticEnergy(const PhysicalModel& pm);
void UpdateDrag(const Camera& cam, double mouseX, double mouseY, int windowWidthz, int windowHeighty);
void OnRightClickPressed(const Camera& cam, double mouseX, double mouseY, int windowWidthz, int windowHeighty);
void CreateStaticFloor(float yPos, const Material& material);
void OnRightClickReleased();