#pragma once

#include <string>
#include <vector>
#include <cstdint>

// ─────────────────────────────────────────────────────────────────────────────
//  ColliderShape  –  type of collider attached to an object
// ─────────────────────────────────────────────────────────────────────────────
enum class ColliderShape
{
    Box,
    Sphere,
    Capsule,
    ConvexHull,
    TriangleMesh,
};

inline const char* ColliderShapeName(ColliderShape s)
{
    switch (s)
    {
    case ColliderShape::Box:          return "Box";
    case ColliderShape::Sphere:       return "Sphere";
    case ColliderShape::Capsule:      return "Capsule";
    case ColliderShape::ConvexHull:   return "ConvexHull";
    case ColliderShape::TriangleMesh: return "TriangleMesh";
    default:                          return "Box";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SceneCollider  –  one collider, child of a SceneObject
// ─────────────────────────────────────────────────────────────────────────────
struct SceneCollider
{
    std::string   name        = "Collider";
    ColliderShape shape       = ColliderShape::Box;

    // Local offset from parent object origin
    float ox = 0.f, oy = 0.f, oz = 0.f;          // offset position
    float rx = 0.f, ry = 0.f, rz = 0.f;          // offset rotation (deg)

    // Shape-specific parameters
    // Box:     sx/sy/sz = half-extents
    // Sphere:  sx = radius
    // Capsule: sx = radius, sy = half-height
    // Hull / Mesh: ignored (built from parent mesh)
    float sx = 0.5f, sy = 0.5f, sz = 0.5f;

    // Material overrides (negative = inherit from parent)
    float friction    = -1.f;
    float restitution = -1.f;

    bool  isTrigger   = false;   // sensor / no collision response
    bool  enabled     = true;
};

// ─────────────────────────────────────────────────────────────────────────────
//  SceneObject  –  one entry in a .scn file
// ─────────────────────────────────────────────────────────────────────────────
struct SceneObject
{
    // Identity
    std::string name;
    std::string parent = "none";    // "none" = root

    // Assets
    std::string modelPath;
    std::string texturePath = "none";

    // Transform
    float x  = 0.f, y  = 0.f, z  = 0.f;
    float rx = 0.f, ry = 0.f, rz = 0.f;
    float sx = 1.f, sy = 1.f, sz = 1.f;

    // Physics
    bool  isStatic = false;
    float weight   = 0.f;          // kg; 0 = auto

    // Colliders (children)
    std::vector<SceneCollider> colliders;
};

// ─────────────────────────────────────────────────────────────────────────────
//  SceneFile
// ─────────────────────────────────────────────────────────────────────────────
struct SceneFile
{
    std::vector<SceneObject> objects;

    // Returns true on success.
    bool Load(const std::string& path);
    bool Save(const std::string& path) const;
    void Clear() { objects.clear(); }

    // Helpers
    bool         IsModified()  const { return m_modified; }
    void         MarkClean()         { m_modified = false; }
    void         MarkDirty()         { m_modified = true; }

    std::string  currentPath;        // last loaded / saved path

private:
    bool m_modified = false;
};
