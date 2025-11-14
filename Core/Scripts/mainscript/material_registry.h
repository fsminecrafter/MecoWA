#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include "physicsDefine.h"

class MaterialRegistry {
public:
    // Register or override a material
    static void Register(const Material& mat) {
        materials()[mat.name] = mat;

#ifdef Debug
        std::cout << "[MaterialRegistry] Registered material: "
            << mat.name
            << " | Density=" << mat.density
            << " | Drag=" << mat.dragCoefficient
            << " | Friction=" << mat.friction
            << " | Restitution=" << mat.restitution
            << std::endl;
#endif
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

#ifdef Debug
        std::cout << "[MaterialRegistry] WARNING: Material '" << name
            << "' not found. Using Default.\n";
#endif

        return mats["Default"];
    }

    // List all materials (debug)
    static void Dump() {
#ifdef Debug
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
