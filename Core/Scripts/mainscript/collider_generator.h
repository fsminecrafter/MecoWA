#pragma once
/*  collider_generator.h
 *
 *  Auto-generates SceneCollider entries from an OBJData mesh.
 *
 *  GenerateColliders_Simple  – axis-aligned box decomposition
 *      Clusters vertices spatially and fits a box per cluster.
 *      Respects maxColliders; uses a single bounding box when 1 is requested.
 *
 *  GenerateColliders_Complex – convex-hull decomposition
 *      K-means style spatial clustering, one ConvexHull collider per cluster.
 *      Respects maxColliders.
 */

#include "scene_file.h"
#include "objloader.h"

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <limits>
#include <glm/glm/glm.hpp>

// ─────────────────────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace ColliderGen
{
    // Extract unique vertex positions from OBJData
    static std::vector<glm::vec3> ExtractVerts(const OBJData& mesh)
    {
        std::vector<glm::vec3> verts;
        size_t n = mesh.vertexCoords.size() / 3;
        verts.reserve(n);
        for (size_t i = 0; i < n; ++i)
            verts.push_back({
                mesh.vertexCoords[i * 3 + 0],
                mesh.vertexCoords[i * 3 + 1],
                mesh.vertexCoords[i * 3 + 2]
            });
        return verts;
    }

    // Compute AABB of a set of points
    struct AABB {
        glm::vec3 mn{ std::numeric_limits<float>::max() };
        glm::vec3 mx{ std::numeric_limits<float>::lowest() };

        void expand(const glm::vec3& p) {
            mn = glm::min(mn, p);
            mx = glm::max(mx, p);
        }
        glm::vec3 center()  const { return (mn + mx) * 0.5f; }
        glm::vec3 size()    const { return mx - mn; }
        glm::vec3 halfExt() const { return size() * 0.5f; }
        bool valid()        const { return mn.x <= mx.x; }
    };

    static AABB ComputeAABB(const std::vector<glm::vec3>& pts,
                             const std::vector<int>& indices)
    {
        AABB box;
        for (int i : indices) box.expand(pts[i]);
        return box;
    }

    static AABB ComputeAABB(const std::vector<glm::vec3>& pts)
    {
        AABB box;
        for (const auto& p : pts) box.expand(p);
        return box;
    }

    // ── K-means spatial clustering ────────────────────────────────────────────
    // Returns cluster assignment per vertex (size == pts.size())
    static std::vector<int> KMeans(const std::vector<glm::vec3>& pts,
                                    int k, int maxIter = 20)
    {
        if (pts.empty() || k <= 0) return {};
        k = std::min(k, (int)pts.size());

        // Seed centroids with k-means++
        std::mt19937 rng(42);
        std::vector<glm::vec3> centroids;
        centroids.reserve(k);

        // Pick first centroid randomly
        std::uniform_int_distribution<int> pick(0, (int)pts.size() - 1);
        centroids.push_back(pts[pick(rng)]);

        for (int c = 1; c < k; ++c)
        {
            // Compute D² distances to nearest existing centroid
            std::vector<float> dist(pts.size());
            float total = 0.f;
            for (size_t i = 0; i < pts.size(); ++i)
            {
                float best = std::numeric_limits<float>::max();
                for (const auto& cc : centroids)
                {
                    float d = glm::length(pts[i] - cc);
                    best = std::min(best, d * d);
                }
                dist[i] = best;
                total  += best;
            }
            // Weighted random selection
            std::uniform_real_distribution<float> wheel(0.f, total);
            float r = wheel(rng);
            float accum = 0.f;
            int chosen = (int)pts.size() - 1;
            for (size_t i = 0; i < pts.size(); ++i) {
                accum += dist[i];
                if (accum >= r) { chosen = (int)i; break; }
            }
            centroids.push_back(pts[chosen]);
        }

        std::vector<int> assign(pts.size(), 0);

        for (int iter = 0; iter < maxIter; ++iter)
        {
            // Assignment step
            bool changed = false;
            for (size_t i = 0; i < pts.size(); ++i)
            {
                int best = 0;
                float bestD = std::numeric_limits<float>::max();
                for (int c = 0; c < k; ++c)
                {
                    float d = glm::length(pts[i] - centroids[c]);
                    if (d < bestD) { bestD = d; best = c; }
                }
                if (assign[i] != best) { assign[i] = best; changed = true; }
            }
            if (!changed) break;

            // Update step
            std::vector<glm::vec3> newC(k, glm::vec3(0.f));
            std::vector<int> cnt(k, 0);
            for (size_t i = 0; i < pts.size(); ++i) {
                newC[assign[i]] += pts[i];
                cnt[assign[i]]++;
            }
            for (int c = 0; c < k; ++c)
                if (cnt[c] > 0) centroids[c] = newC[c] / (float)cnt[c];
        }

        return assign;
    }

    // Build a SceneCollider (Box) from an AABB
    static SceneCollider MakeBoxCollider(const AABB& box, int index)
    {
        SceneCollider c;
        c.name   = "AutoBox_" + std::to_string(index);
        c.shape  = ColliderShape::Box;
        c.enabled = true;
        c.isTrigger = false;

        glm::vec3 ctr  = box.center();
        glm::vec3 half = glm::max(box.halfExt(), glm::vec3(0.001f));

        c.ox = ctr.x;  c.oy = ctr.y;  c.oz = ctr.z;
        c.rx = 0.f;    c.ry = 0.f;    c.rz = 0.f;
        c.sx = half.x; c.sy = half.y; c.sz = half.z;

        c.friction    = -1.f;  // inherit
        c.restitution = -1.f;
        return c;
    }

    // Build a SceneCollider (ConvexHull) from cluster center
    static SceneCollider MakeConvexCollider(const AABB& box, int index)
    {
        SceneCollider c = MakeBoxCollider(box, index);
        c.name  = "AutoConvex_" + std::to_string(index);
        c.shape = ColliderShape::ConvexHull;
        return c;
    }

    // Eliminate near-duplicate or zero-size clusters
    static bool IsDegenerate(const AABB& box, float minSize = 1e-4f)
    {
        glm::vec3 s = box.size();
        return s.x < minSize && s.y < minSize && s.z < minSize;
    }

} // namespace ColliderGen

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────

// Simple: spatial clustering -> one Box per cluster.
// maxColliders is strictly respected.
inline std::vector<SceneCollider> GenerateColliders_Simple(const OBJData& mesh,
                                                            int maxColliders)
{
    using namespace ColliderGen;

    std::vector<SceneCollider> result;
    if (mesh.vertexCoords.empty()) return result;

    std::vector<glm::vec3> pts = ExtractVerts(mesh);

    // Clamp k to the number of distinct points
    int k = std::max(1, std::min(maxColliders, (int)pts.size()));

    if (k == 1)
    {
        // Single bounding box
        AABB box = ComputeAABB(pts);
        if (box.valid() && !IsDegenerate(box))
            result.push_back(MakeBoxCollider(box, 0));
        return result;
    }

    // K-means clustering
    std::vector<int> assign = KMeans(pts, k);

    // Build per-cluster AABBs
    std::vector<AABB> clusterBoxes(k);
    for (size_t i = 0; i < pts.size(); ++i)
        clusterBoxes[assign[i]].expand(pts[i]);

    // Emit a collider for each non-degenerate cluster
    int idx = 0;
    for (int c = 0; c < k; ++c)
    {
        if (!clusterBoxes[c].valid() || IsDegenerate(clusterBoxes[c])) continue;
        result.push_back(MakeBoxCollider(clusterBoxes[c], idx++));
        if ((int)result.size() >= maxColliders) break;
    }

    // If clustering produced fewer colliders than requested that is fine —
    // we never pad with duplicates just to hit the max.
    return result;
}

// Complex: spatial clustering -> one ConvexHull per cluster.
// maxColliders is strictly respected.
inline std::vector<SceneCollider> GenerateColliders_Complex(const OBJData& mesh,
                                                             int maxColliders)
{
    using namespace ColliderGen;

    std::vector<SceneCollider> result;
    if (mesh.vertexCoords.empty()) return result;

    std::vector<glm::vec3> pts = ExtractVerts(mesh);

    int k = std::max(1, std::min(maxColliders, (int)pts.size()));

    if (k == 1)
    {
        AABB box = ComputeAABB(pts);
        if (box.valid() && !IsDegenerate(box))
            result.push_back(MakeConvexCollider(box, 0));
        return result;
    }

    std::vector<int> assign = KMeans(pts, k);

    std::vector<AABB> clusterBoxes(k);
    for (size_t i = 0; i < pts.size(); ++i)
        clusterBoxes[assign[i]].expand(pts[i]);

    int idx = 0;
    for (int c = 0; c < k; ++c)
    {
        if (!clusterBoxes[c].valid() || IsDegenerate(clusterBoxes[c])) continue;
        result.push_back(MakeConvexCollider(clusterBoxes[c], idx++));
        if ((int)result.size() >= maxColliders) break;
    }

    return result;
}
