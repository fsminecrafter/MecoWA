#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <cmath>
#include "physicsDefine.h"
#include "engine.h"

class MaterialRegistry {
public:
    // Register or override a material
    static void Register(const Material& mat) {
        materials()[mat.name] = mat;

#ifdef _DEBUG
        std::cout << "[MaterialRegistry] Registered material: "
            << mat.name
            << " | Density=" << mat.density
            << " | Drag=" << mat.dragCoefficient
            << " | Friction=" << mat.friction
            << " | Restitution=" << mat.restitution
            << std::endl;
#endif
    }

    static float Apply(const ModelInstance& object, const std::string& materialName)
    {
        const Material& mat = MaterialRegistry::Get(materialName);

        const OBJData& mesh = object.model;

        float volume = 0.0f;

        for (size_t i = 0; i < mesh.elementArray.size(); i += 3)
        {
            uint32_t i0 = mesh.elementArray[i + 0] * 3;
            uint32_t i1 = mesh.elementArray[i + 1] * 3;
            uint32_t i2 = mesh.elementArray[i + 2] * 3;

            glm::vec3 a(
                mesh.vertexCoords[i0 + 0],
                mesh.vertexCoords[i0 + 1],
                mesh.vertexCoords[i0 + 2]
            );

            glm::vec3 b(
                mesh.vertexCoords[i1 + 0],
                mesh.vertexCoords[i1 + 1],
                mesh.vertexCoords[i1 + 2]
            );

            glm::vec3 c(
                mesh.vertexCoords[i2 + 0],
                mesh.vertexCoords[i2 + 1],
                mesh.vertexCoords[i2 + 2]
            );

            // apply instance scale
            a *= object.scale;
            b *= object.scale;
            c *= object.scale;

            glm::vec3 ab = b - a;
            glm::vec3 ac = c - a;

            volume += glm::dot(a, glm::cross(ab, ac)) / 6.0f;
        }

        volume = std::abs(volume);

        float mass = volume * mat.density;

        // round to 1 decimal (0.1)
        return std::round(mass * 10.0f) / 10.0f;
    }

    // Check if exists
    static bool Exists(const std::string& name) {
        return materials().count(name) > 0;
    }

    // Retrieve material or fallback to "Default"
    static const Material& Get(const std::string& name) {
        auto& mats = materials();
        auto it = mats.find(name);

        if (it != mats.end())
            return it->second;

#ifdef _DEBUG
        std::cout << "[MaterialRegistry] WARNING: Material '" << name
            << "' not found. Using Default.\n";
#endif

        return mats["Default"];
    }

    // List all materials (debug)
    static void Dump() {

#ifdef _DEBUG
        std::cout << "\n=== Material Registry Dump ===\n";
        for (auto& [name, mat] : materials()) {
            std::cout << name
                << " | density=" << mat.density
                << " | Cd=" << mat.dragCoefficient
                << " | friction=" << mat.friction
                << " | rest=" << mat.restitution
                << "\n";
        }
        std::cout << "===============================\n";
#endif
    }

private:
    // Stored here
    static std::unordered_map<std::string, Material>& materials() {
        static std::unordered_map<std::string, Material> instance = {
            { "Default", Material{} }
        };
        return instance;
    }
};
