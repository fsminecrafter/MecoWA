#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Renderer/DebugRenderer.h>

class OpenGLDebugRenderer final : public JPH::DebugRenderer
{
public:
    OpenGLDebugRenderer();
    ~OpenGLDebugRenderer() override;

    void DrawLine(JPH::RVec3Arg from, JPH::RVec3Arg to, JPH::ColorArg color) override;
    void DrawTriangle(JPH::RVec3Arg v1, JPH::RVec3Arg v2, JPH::RVec3Arg v3,
        JPH::ColorArg color, ECastShadow castShadow) override;
    void DrawText3D(JPH::RVec3Arg position, const std::string_view& text,
        JPH::ColorArg color, float height) override;

    Batch CreateTriangleBatch(const Vertex* vertices, int vertexCount,
        const JPH::uint32* indices, int indexCount) override;

    Batch CreateTriangleBatch(const Triangle* triangles, int triangleCount) override;

    void DrawGeometry(JPH::RMat44Arg modelMatrix,
        const JPH::AABox& worldBounds,
        float lodScaleSq,
        JPH::ColorArg modelColor,
        const GeometryRef& geometry,
        ECullMode cullMode,
        ECastShadow castShadow,
        EDrawMode drawMode) override;

    void SetCameraPosition(float x, float y, float z);

private:
    JPH::Vec3 _cameraPos{ 0, 0, 0 };
};
