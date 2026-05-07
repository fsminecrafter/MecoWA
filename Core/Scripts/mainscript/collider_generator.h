#pragma once
// collider_generator.h

#include "scene_file.h"   // SceneCollider, ColliderShape
#include "objloader.h"    // OBJData

#include <glm/glm/glm.hpp>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <string>
#include <numeric>
#include <random>

namespace ColGen
{

    struct AABB {
        glm::vec3 mn{ 1e30f };
        glm::vec3 mx{ -1e30f };

        void expand(const glm::vec3& p) {
            mn = glm::min(mn, p);
            mx = glm::max(mx, p);
        }
        glm::vec3 center()   const { return (mn + mx) * 0.5f; }
        glm::vec3 halfext()  const { return (mx - mn) * 0.5f; }
        bool valid() const { return mx.x >= mn.x; }
    };

    // Compute per-vertex AABB from raw float coords
    inline AABB MeshAABB(const OBJData& mesh)
    {
        AABB box;
        for (size_t i = 0; i + 2 < mesh.vertexCoords.size(); i += 3)
            box.expand({ mesh.vertexCoords[i], mesh.vertexCoords[i + 1], mesh.vertexCoords[i + 2] });
        return box;
    }

    // Face centroid list
    inline std::vector<glm::vec3> FaceCentroids(const OBJData& mesh)
    {
        std::vector<glm::vec3> cs;
        cs.reserve(mesh.elementArray.size() / 3);
        for (size_t i = 0; i + 2 < mesh.elementArray.size(); i += 3)
        {
            auto idx = [&](int k) { return mesh.elementArray[i + k]; };
            auto v = [&](unsigned id) -> glm::vec3 {
                return { mesh.vertexCoords[id * 3], mesh.vertexCoords[id * 3 + 1], mesh.vertexCoords[id * 3 + 2] };
                };
            cs.push_back((v(idx(0)) + v(idx(1)) + v(idx(2))) / 3.f);
        }
        return cs;
    }

    // Vertices belonging to faces in a cluster index set
    inline std::vector<glm::vec3> ClusterVerts(
        const OBJData& mesh,
        const std::vector<int>& faceLabels,
        int label)
    {
        std::vector<glm::vec3> verts;
        for (size_t f = 0; f < faceLabels.size(); ++f)
        {
            if (faceLabels[f] != label) continue;
            size_t base = f * 3;
            for (int k = 0; k < 3; ++k)
            {
                unsigned id = mesh.elementArray[base + k];
                verts.push_back({ mesh.vertexCoords[id * 3], mesh.vertexCoords[id * 3 + 1], mesh.vertexCoords[id * 3 + 2] });
            }
        }
        return verts;
    }

    // Simple k-means on 3-D points (Lloyd's, fixed iterations)
    inline std::vector<int> KMeans(const std::vector<glm::vec3>& pts, int k, int iters = 12)
    {
        if (pts.empty() || k <= 0) return {};
        k = std::min(k, (int)pts.size());

        std::mt19937 rng(42);
        std::vector<glm::vec3> centers(k);
        std::vector<int>       labels(pts.size(), 0);

        // KMeans++ style init: pick spread seeds
        std::vector<int> seedIdx;
        {
            std::uniform_int_distribution<int> dist(0, (int)pts.size() - 1);
            seedIdx.push_back(dist(rng));
            for (int c = 1; c < k; ++c)
            {
                // pick farthest from nearest existing center
                float bestD = -1.f; int bestI = 0;
                for (int i = 0; i < (int)pts.size(); ++i)
                {
                    float minD = 1e30f;
                    for (int s : seedIdx) minD = std::min(minD, glm::length(pts[i] - pts[s]));
                    if (minD > bestD) { bestD = minD; bestI = i; }
                }
                seedIdx.push_back(bestI);
            }
        }
        for (int c = 0; c < k; ++c) centers[c] = pts[seedIdx[c]];

        for (int it = 0; it < iters; ++it)
        {
            // Assign
            for (int i = 0; i < (int)pts.size(); ++i)
            {
                float best = 1e30f; int bl = 0;
                for (int c = 0; c < k; ++c)
                {
                    float d = glm::length(pts[i] - centers[c]);
                    if (d < best) { best = d; bl = c; }
                }
                labels[i] = bl;
            }
            // Recompute centers
            std::vector<glm::vec3> sum(k, glm::vec3(0));
            std::vector<int> cnt(k, 0);
            for (int i = 0; i < (int)pts.size(); ++i)
            {
                sum[labels[i]] += pts[i];
                cnt[labels[i]]++;
            }
            for (int c = 0; c < k; ++c)
                if (cnt[c] > 0) centers[c] = sum[c] / (float)cnt[c];
        }
        return labels;
    }

    // Build a SceneCollider box from an AABB
    inline SceneCollider BoxFromAABB(const AABB& box, const std::string& name)
    {
        SceneCollider c;
        c.name = name;
        c.shape = ColliderShape::Box;
        glm::vec3 ctr = box.center();
        glm::vec3 he = box.halfext();
        c.ox = ctr.x; c.oy = ctr.y; c.oz = ctr.z;
        c.sx = std::max(he.x, 0.001f);
        c.sy = std::max(he.y, 0.001f);
        c.sz = std::max(he.z, 0.001f);
        c.enabled = true;
        return c;
    }

    // Build a SceneCollider convex hull (shape param unused - built from verts at spawn)
    inline SceneCollider ConvexFromAABB(const AABB& box, const std::string& name)
    {
        SceneCollider c = BoxFromAABB(box, name);
        c.shape = ColliderShape::ConvexHull;
        return c;
    }

} // namespace ColGen

// Simple mode: up to maxColliders axis-aligned boxes that decompose the mesh
// along its longest axis (recursive slab split).
// For very simple meshes (or maxColliders==1) this degenerates to a single
// bounding box.
inline std::vector<SceneCollider> GenerateColliders_Simple(
    const OBJData& mesh,
    int maxColliders = 4)
{
    using namespace ColGen;
    std::vector<SceneCollider> result;

    if (mesh.vertexCoords.empty()) return result;

    maxColliders = std::max(1, std::min(maxColliders, 16));

    // Collect all vertex positions
    size_t vcount = mesh.vertexCoords.size() / 3;
    std::vector<glm::vec3> verts(vcount);
    for (size_t i = 0; i < vcount; ++i)
        verts[i] = { mesh.vertexCoords[i * 3], mesh.vertexCoords[i * 3 + 1], mesh.vertexCoords[i * 3 + 2] };

    // Recursive slab split: split along longest axis, recurse
    // Uses a queue of vertex index subsets
    struct Subset { std::vector<int> idx; };
    std::vector<Subset> queue;
    {
        Subset all;
        all.idx.resize(vcount);
        std::iota(all.idx.begin(), all.idx.end(), 0);
        queue.push_back(std::move(all));
    }

    while ((int)queue.size() < maxColliders && !queue.empty())
    {
        // Find the subset with the largest volume to split
        int splitIdx = 0;
        float bestVol = -1.f;
        for (int i = 0; i < (int)queue.size(); ++i)
        {
            AABB b;
            for (int vi : queue[i].idx) b.expand(verts[vi]);
            glm::vec3 s = b.mx - b.mn;
            float vol = s.x * s.y * s.z;
            if (vol > bestVol) { bestVol = vol; splitIdx = i; }
        }

        Subset& sub = queue[splitIdx];
        AABB b;
        for (int vi : sub.idx) b.expand(verts[vi]);
        glm::vec3 sz = b.mx - b.mn;

        // longest axis
        int axis = 0;
        if (sz.y > sz[axis]) axis = 1;
        if (sz.z > sz[axis]) axis = 2;

        float mid = b.mn[axis] + sz[axis] * 0.5f;

        Subset left, right;
        for (int vi : sub.idx)
        {
            if (verts[vi][axis] <= mid) left.idx.push_back(vi);
            else                        right.idx.push_back(vi);
        }

        if (left.idx.empty() || right.idx.empty()) break; // can't split further

        queue.erase(queue.begin() + splitIdx);
        queue.push_back(std::move(left));
        queue.push_back(std::move(right));
    }

    // Build one box per subset
    for (int i = 0; i < (int)queue.size(); ++i)
    {
        AABB b;
        for (int vi : queue[i].idx) b.expand(verts[vi]);
        if (!b.valid()) continue;
        result.push_back(BoxFromAABB(b, "Auto_Box_" + std::to_string(i)));
    }

    return result;
}

// Complex mode: k-means clustering of face centroids -> one ConvexHull per cluster.
// maxColliders directly = number of clusters.
inline std::vector<SceneCollider> GenerateColliders_Complex(
    const OBJData& mesh,
    int maxColliders = 8)
{
    using namespace ColGen;
    std::vector<SceneCollider> result;

    if (mesh.elementArray.empty()) return result;

    maxColliders = std::max(1, std::min(maxColliders, 32));

    int numFaces = (int)(mesh.elementArray.size() / 3);
    int k = std::min(maxColliders, numFaces);

    auto centroids = FaceCentroids(mesh);
    if (centroids.empty()) return result;

    auto labels = KMeans(centroids, k);

    for (int c = 0; c < k; ++c)
    {
        auto cverts = ClusterVerts(mesh, labels, c);
        if (cverts.empty()) continue;

        AABB box;
        for (auto& v : cverts) box.expand(v);
        if (!box.valid()) continue;

        // Store as ConvexHull – offset to cluster center, size = half-extents
        SceneCollider col = ConvexFromAABB(box, "Auto_Hull_" + std::to_string(c));
        result.push_back(col);
    }

    return result;
}