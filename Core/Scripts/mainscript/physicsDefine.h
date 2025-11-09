#pragma once
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include "engine.h"  // so we can use ModelInstance

// 
// MATERIAL DEFINITION
// 
struct Material {
    std::string name = "Default";
    float density = 1.0f;             // kg/m³ or arbitrary density unit
    float friction = 0.5f;            // for later physics
    float restitution = 0.1f;         // bounciness
    std::string texturePath = "";     // future texture path
};

// 
// ANCHOR POINTS (for mechanical joints, hinges, etc.)
// 
struct AnchorPoint {
    glm::vec3 localPosition;  // relative to model origin
    glm::vec3 direction;      // useful for hinge or piston
    bool active = true;
};

// 
// PHYSICAL ENTITY (extends ModelInstance with physics properties)
// 
struct PhysicalEntity {
    ModelInstance instance;           // includes position, rotation, scale, etc.

    // Physics attributes
    glm::vec3 velocity = glm::vec3(0.0f);
    glm::vec3 angularVelocity = glm::vec3(0.0f);
    glm::vec3 forces = glm::vec3(0.0f);

    float mass = 1.0f;
    bool isStatic = false;

    Material material;
    std::vector<AnchorPoint> anchors;

    // 
    // Physics utility methods
    // 
    inline glm::vec3 GetPosition() const { return instance.position; }
    inline void SetPosition(const glm::vec3& pos) { instance.position = pos; }

    inline glm::vec3 GetRotation() const { return instance.rotation; }
    inline void SetRotation(const glm::vec3& rot) { instance.rotation = rot; }

    inline void ApplyForce(const glm::vec3& f) { forces += f; }
};