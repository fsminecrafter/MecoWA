#include "jolt_debugrender.h"
#include <glm/glm/glm.hpp>
#include <glm/glm/gtc/type_ptr.hpp>
#include <glad/include/glad/glad.h>
#include <glfw/include/GLFW/glfw3.h>
#include <iostream>

JoltDebugRenderer::JoltDebugRenderer()
{
    glGenVertexArrays(1, &mVAO);
    glGenBuffers(1, &mVBO);
}

JoltDebugRenderer::~JoltDebugRenderer()
{
    glDeleteBuffers(1, &mVBO);
    glDeleteVertexArrays(1, &mVAO);
}

void JoltDebugRenderer::Clear()
{
    mLines.clear();
}

void JoltDebugRenderer::DrawLine(JPH::RVec3Arg from, JPH::RVec3Arg to, JPH::ColorArg color)
{
    mLines.push_back({
        glm::vec3(from.GetX(), from.GetY(), from.GetZ()),
        glm::vec3(to.GetX(), to.GetY(), to.GetZ()),
        glm::vec3(color.r / 255.0f, color.g / 255.0f, color.b / 255.0f)
        });
}

void JoltDebugRenderer::DrawTriangle(JPH::RVec3Arg v1, JPH::RVec3Arg v2, JPH::RVec3Arg v3,
    JPH::ColorArg color, ECastShadow)
{
    DrawLine(v1, v2, color);
    DrawLine(v2, v3, color);
    DrawLine(v3, v1, color);
}

// ----------------------------------------------------
// Send line data to GPU and render
// ----------------------------------------------------
void JoltDebugRenderer::Render(const glm::mat4& viewProj)
{
    if (mLines.empty())
        return;

    // Fill vertex buffer (pos.xyz + color.xyz)
    mVertexBuffer.clear();
    for (auto& line : mLines)
    {
        mVertexBuffer.insert(mVertexBuffer.end(), {
            line.from.x, line.from.y, line.from.z,
            line.color.r, line.color.g, line.color.b,
            line.to.x, line.to.y, line.to.z,
            line.color.r, line.color.g, line.color.b
            });
    }

    glBindVertexArray(mVAO);
    glBindBuffer(GL_ARRAY_BUFFER, mVBO);
    glBufferData(GL_ARRAY_BUFFER, mVertexBuffer.size() * sizeof(float),
        mVertexBuffer.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0); // pos
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1); // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(mVertexBuffer.size() / 6));

    glBindVertexArray(0);
}
