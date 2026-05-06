#pragma once

#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
//  SceneObject  –  one entry in a .scn file
// ─────────────────────────────────────────────────────────────────────────────
struct SceneObject
{
    // Line 1 – identity
    std::string name;
    std::string parent;       // "none" means no parent

    // Line 2 – assets
    std::string modelPath;    // relative to project root
    std::string texturePath;  // "none" means untextured

    // Line 3 – transform
    float x  = 0.f, y  = 0.f, z  = 0.f;   // position
    float rx = 0.f, ry = 0.f, rz = 0.f;   // rotation  (degrees)
    float sx = 1.f, sy = 1.f, sz = 1.f;   // scale

    // Line 4 – physics
    bool  isStatic = false;
    float weight   = 0.f;    // kg; 0 = auto-compute from material/volume
};

// ─────────────────────────────────────────────────────────────────────────────
//  SceneFile  –  parse / serialise a .scn file
// ─────────────────────────────────────────────────────────────────────────────
struct SceneFile
{
    std::vector<SceneObject> objects;

    // Returns true on success.  Clears objects first.
    bool Load(const std::string& path);

    // Returns true on success.
    bool Save(const std::string& path) const;

    void Clear() { objects.clear(); }
};
