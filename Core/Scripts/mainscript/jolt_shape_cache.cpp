#include "Jolt/Jolt.h"
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
        h ^= std::hash<float>()(v[0]); h *= 0x100000001b3ull;
        h ^= std::hash<float>()(v[1]); h *= 0x100000001b3ull;
        h ^= std::hash<float>()(v[2]); h *= 0x100000001b3ull;
    }
    return h;
}


JPH::ShapeRefC GetTriangleMeshShapeCached(const std::vector<JPH::Float3>& verts,
    const std::vector<uint32_t>& idx)
{
    const uint64_t h = HashVerts(verts);

    if (triangleShapeCache.contains(h))
        return triangleShapeCache[h];

    JPH::VertexList vertices;
    vertices.reserve(verts.size());
    for (const auto& v : verts)
        vertices.push_back(v);
    
    JPH::IndexedTriangleList indices;
    indices.reserve(idx.size() / 3);
    for (size_t i = 0; i < idx.size(); i += 3)
        indices.push_back(JPH::IndexedTriangle(idx[i], idx[i+1], idx[i+2]));
    
    JPH::MeshShapeSettings mesh(vertices, indices);
    auto created = mesh.Create().Get();

    triangleShapeCache[h] = created;
    return created;
}


JPH::ShapeRefC CreateConvexHullShape(const std::vector<JPH::Float3>& verts)
{
    JPH::Array<JPH::Vec3> points;
    points.reserve(verts.size());
    for (const auto& v : verts)
        points.push_back(JPH::Vec3(v[0], v[1], v[2]));
    
    JPH::ConvexHullShapeSettings hull(points);
    return hull.Create().Get();
}
