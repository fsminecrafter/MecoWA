#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Renderer/DebugRenderer.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <glm/glm/glm.hpp>
#include <vector>

struct DebugLine
{
    glm::vec3 from;
    glm::vec3 to;
    glm::vec3 color;
};

class JoltDebugRenderer : public JPH::DebugRenderer
{
public:
    JoltDebugRenderer();
    ~JoltDebugRenderer();

    void DrawLine(JPH::RVec3Arg from, JPH::RVec3Arg to, JPH::ColorArg color) override;
    void DrawTriangle(JPH::RVec3Arg v1, JPH::RVec3Arg v2, JPH::RVec3Arg v3,
        JPH::ColorArg color, ECastShadow castShadow) override;

    void Clear();
    void Render(const glm::mat4& viewProj);

private:
    std::vector<DebugLine> mLines;
    unsigned int mVAO = 0;
    unsigned int mVBO = 0;
    std::vector<float> mVertexBuffer;
};

