#include "jolt_shape_cache.h"
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <functional>

static std::unordered_map<uint64_t, JPH::ShapeRefC> triangleShapeCache;

static uint64_t HashVerts(const std::vector<JPH::Float3>& verts)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (const auto& v : verts)
    {
        h ^= std::hash<float>()(v.GetX()); h *= 0x100000001b3ull;
        h ^= std::hash<float>()(v.GetY()); h *= 0x100000001b3ull;
        h ^= std::hash<float>()(v.GetZ()); h *= 0x100000001b3ull;
    }
    return h;
}


JPH::ShapeRefC GetTriangleMeshShapeCached(const std::vector<JPH::Float3>& verts,
    const std::vector<uint32_t>& idx)
{
    const uint64_t h = HashVerts(verts);

    if (triangleShapeCache.contains(h))
        return triangleShapeCache[h];

    JPH::MeshShapeSettings mesh(verts, idx);
    auto created = mesh.Create().Get();

    triangleShapeCache[h] = created;
    return created;
}


JPH::ShapeRefC CreateConvexHullShape(const std::vector<JPH::Float3>& verts)
{
    JPH::ConvexHullShapeSettings hull(verts);
    return hull.Create().Get();
}
