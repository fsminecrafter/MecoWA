#include "debugrender.h"

#include <glad/include/glad/glad.h>
#include <glfw/include/GLFW/glfw3.h>

#include <glm/glm/mat4x4.hpp>
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/type_ptr.hpp>
#include <glm/glm/gtc/matrix_transform.hpp>

#include <vector>
#include <cstddef>
#include <atomic>


// ------------------------------------------------------------
// OpenGLBatch
// ------------------------------------------------------------

class OpenGLBatch final : public JPH::RefTargetVirtual
{
public:
    OpenGLBatch(const JPH::DebugRenderer::Vertex* inVertices,
        int inVertexCount,
        const JPH::uint32* inIndices,
        int inIndexCount);

    OpenGLBatch(const JPH::DebugRenderer::Triangle* inTriangles,
        int inTriangleCount);

    virtual ~OpenGLBatch() override;

    virtual void AddRef() override;
    virtual void Release() override;

    void Draw(JPH::ColorArg inColor);

private:
    std::atomic<uint32_t> _refCount{ 0 };
    GLuint _vbo = 0;
    GLuint _ebo = 0;
    int _indexCount = 0;
};

void OpenGLDebugRenderer::SetViewProjection(const glm::mat4& view,
    const glm::mat4& projection)
{
    _view = view;
    _projection = projection;
}


void OpenGLBatch::AddRef()
{
    _refCount.fetch_add(1, std::memory_order_relaxed);
}

void OpenGLBatch::Release()
{
    if (_refCount.fetch_sub(1, std::memory_order_acq_rel) == 1)
        delete this;
}


OpenGLBatch::OpenGLBatch(const JPH::DebugRenderer::Vertex* inVertices,
    int inVertexCount,
    const JPH::uint32* inIndices,
    int inIndexCount)
{
    _indexCount = (inIndexCount > 0) ? inIndexCount : inVertexCount;

    glGenBuffers(1, &_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER,
        inVertexCount * sizeof(JPH::DebugRenderer::Vertex),
        inVertices,
        GL_STATIC_DRAW);

    glGenBuffers(1, &_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);

    if (inIndices)
    {
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
            _indexCount * sizeof(GLuint),
            inIndices,
            GL_STATIC_DRAW);
    }
    else
    {
        std::vector<GLuint> indices(_indexCount);
        for (int i = 0; i < _indexCount; ++i)
            indices[i] = i;

        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
            _indexCount * sizeof(GLuint),
            indices.data(),
            GL_STATIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

OpenGLBatch::OpenGLBatch(const JPH::DebugRenderer::Triangle* inTriangles,
    int inTriangleCount)
{
    const int vertexCount = inTriangleCount * 3;
    _indexCount = vertexCount;

    std::vector<JPH::DebugRenderer::Vertex> vertices;
    vertices.reserve(vertexCount);

    for (int i = 0; i < inTriangleCount; ++i)
        for (int j = 0; j < 3; ++j)
            vertices.push_back(inTriangles[i].mV[j]);

    std::vector<GLuint> indices(vertexCount);
    for (int i = 0; i < vertexCount; ++i)
        indices[i] = i;

    glGenBuffers(1, &_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER,
        vertices.size() * sizeof(JPH::DebugRenderer::Vertex),
        vertices.data(),
        GL_STATIC_DRAW);

    glGenBuffers(1, &_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        indices.size() * sizeof(GLuint),
        indices.data(),
        GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

OpenGLBatch::~OpenGLBatch()
{
    glDeleteBuffers(1, &_vbo);
    glDeleteBuffers(1, &_ebo);
}

void OpenGLBatch::Draw(JPH::ColorArg inColor)
{
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT,
        sizeof(JPH::DebugRenderer::Vertex),
        (void*)offsetof(JPH::DebugRenderer::Vertex, mPosition));

    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT,
        sizeof(JPH::DebugRenderer::Vertex),
        (void*)offsetof(JPH::DebugRenderer::Vertex, mNormal));

    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT,
        sizeof(JPH::DebugRenderer::Vertex),
        (void*)offsetof(JPH::DebugRenderer::Vertex, mUV));

    glColor4ub(inColor.r, inColor.g, inColor.b, inColor.a);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo);
    glDrawElements(GL_TRIANGLES, _indexCount, GL_UNSIGNED_INT, nullptr);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

// ------------------------------------------------------------
// OpenGLDebugRenderer
// ------------------------------------------------------------

OpenGLDebugRenderer::OpenGLDebugRenderer()
{
    Initialize(); // REQUIRED by Jolt
}

OpenGLDebugRenderer::~OpenGLDebugRenderer() = default;

void OpenGLDebugRenderer::DrawLine(JPH::RVec3Arg from,
    JPH::RVec3Arg to,
    JPH::ColorArg color)
{
    glColor4ub(color.r, color.g, color.b, color.a);
    glBegin(GL_LINES);
    glVertex3d(from.GetX(), from.GetY(), from.GetZ());
    glVertex3d(to.GetX(), to.GetY(), to.GetZ());
    glEnd();
}

void OpenGLDebugRenderer::DrawTriangle(JPH::RVec3Arg v1,
    JPH::RVec3Arg v2,
    JPH::RVec3Arg v3,
    JPH::ColorArg color,
    ECastShadow)
{
    glColor4ub(color.r, color.g, color.b, color.a);
    glBegin(GL_TRIANGLES);
    glVertex3d(v1.GetX(), v1.GetY(), v1.GetZ());
    glVertex3d(v2.GetX(), v2.GetY(), v2.GetZ());
    glVertex3d(v3.GetX(), v3.GetY(), v3.GetZ());
    glEnd();
}

void OpenGLDebugRenderer::DrawText3D(JPH::RVec3Arg,
    const std::string_view&,
    JPH::ColorArg,
    float)
{
    // Optional – no text rendering implemented
}

JPH::DebugRenderer::Batch
OpenGLDebugRenderer::CreateTriangleBatch(const Vertex* vertices,
    int vertexCount,
    const JPH::uint32* indices,
    int indexCount)
{
    return new OpenGLBatch(vertices, vertexCount, indices, indexCount);
}

JPH::DebugRenderer::Batch
OpenGLDebugRenderer::CreateTriangleBatch(const Triangle* triangles,
    int triangleCount)
{
    return new OpenGLBatch(triangles, triangleCount);
}

void OpenGLDebugRenderer::DrawGeometry(JPH::RMat44Arg modelMatrix,
    const JPH::AABox& worldBounds,
    float lodScaleSq,
    JPH::ColorArg modelColor,
    const GeometryRef& geometry,
    ECullMode,
    ECastShadow,
    EDrawMode drawMode)
{
    glPolygonMode(GL_FRONT_AND_BACK,
        drawMode == EDrawMode::Wireframe ? GL_LINE : GL_FILL);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixf(glm::value_ptr(_projection));

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // View * Model
    glm::mat4 model;
    float m[16];
    modelMatrix.StoreFloat4x4((JPH::Float4*)m);
    model = _view * glm::make_mat4(m);

    glLoadMatrixf(glm::value_ptr(model));


    const auto& lod =
        geometry->GetLOD(_cameraPos, worldBounds, lodScaleSq);

    auto* batch =
        static_cast<OpenGLBatch*>(lod.mTriangleBatch.GetPtr());

    batch->Draw(modelColor);

    glPopMatrix(); // MODELVIEW
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

}

void OpenGLDebugRenderer::SetCameraPosition(float x, float y, float z)
{
    _cameraPos = JPH::Vec3(x, y, z);
}
