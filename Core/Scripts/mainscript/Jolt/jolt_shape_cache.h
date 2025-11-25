#pragma once
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <vector>
#include <unordered_map>

struct TriangleMeshKey {
    uint64_t hash;
};

JPH::ShapeRefC GetTriangleMeshShapeCached(const std::vector<JPH::Float3>& verts,
    const std::vector<uint32_t>& idx);

JPH::ShapeRefC CreateConvexHullShape(const std::vector<JPH::Float3>& verts);
