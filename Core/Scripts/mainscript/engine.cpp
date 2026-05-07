
// Cherry engine
// Made by Joel_minecrafter


#include "engine.h"
#include <iostream>
#include <vector>
#include "objloader.h"
#include <random>
#include <ctime>

#include "jolt_bridge.h"
#include "jolt_layers.h"
#include <glm/glm/glm.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Renderer/DebugRenderer.h>

std::vector<ObjectList> sceneModels;

//#define DEBUG

glm::vec3 ComputeMeshSize(const OBJData& obj)
{
    glm::vec3 min(FLT_MAX);
    glm::vec3 max(-FLT_MAX);

    for (size_t i = 0; i < obj.vertexCoords.size(); i += 3)
    {
        glm::vec3 v(
            obj.vertexCoords[i + 0],
            obj.vertexCoords[i + 1],
            obj.vertexCoords[i + 2]
        );

        min = glm::min(min, v);
        max = glm::max(max, v);
    }

    return max - min;
}

ModelInstance CreateModelInstance(const OBJData& model, glm::vec3 pos, glm::vec3 rot) {
    ModelInstance instance;
    instance.model = model;
    instance.position = pos;
    instance.rotation = rot;
    instance.scale = glm::vec3(1.0f);
    if (model.vertexCoords.size() % 3 == 0)
        instance.vertexCount = static_cast<unsigned>(model.vertexCoords.size() / 3);
    else {
        std::cerr << "[ModelInstance] Warning: vertexCoords not divisible by 3\n";
        instance.vertexCount = 0;
    }

    if (model.elementArray.size() % 3 == 0)
        instance.indiciesCount = static_cast<unsigned>(model.elementArray.size() / 3);
    else {
        std::cerr << "[ModelInstance] Warning: elementArray not divisible by 3\n";
        instance.indiciesCount = 0;
    }

    return instance;
}

glm::mat4 ComputeModelMatrix(const ModelInstance& instance) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, instance.position);
    model = glm::rotate(model, glm::radians(instance.rotation.x), glm::vec3(1, 0, 0));
    model = glm::rotate(model, glm::radians(instance.rotation.y), glm::vec3(0, 1, 0));
    model = glm::rotate(model, glm::radians(instance.rotation.z), glm::vec3(0, 0, 1));
    model = glm::scale(model, instance.scale);
    return model;
}

glm::mat4 getModelMatrix(const ModelInstance& instance) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, instance.position);
    model = glm::rotate(model, glm::radians(instance.rotation.x), glm::vec3(1, 0, 0));
    model = glm::rotate(model, glm::radians(instance.rotation.y), glm::vec3(0, 1, 0));
    model = glm::rotate(model, glm::radians(instance.rotation.z), glm::vec3(0, 0, 1));
    return model;
}


void DrawModel(const ModelInstance& instance, Shader& shader, GLuint VAO) {
    glm::mat4 modelMatrix = ComputeModelMatrix(instance);
    shader.setMat4("model", modelMatrix);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES,
        static_cast<GLsizei>(instance.model.elementArray.size()),
        GL_UNSIGNED_INT,
        0);

#ifdef DEBUG
    std::cout << "[DrawModel] Drew " << instance.model.elementArray.size() / 3
        << " triangles (" << instance.model.vertexCoords.size() / 3
        << " vertices)\n";
#endif
}

inline Vec3 ToJoltVec3(const glm::vec3& v)
{
    return Vec3(v.x, v.y, v.z);
}

inline Quat ToJoltQuat(const glm::vec3& eulerDeg)
{
    glm::quat q = glm::quat(glm::radians(eulerDeg));
    return Quat(q.x, q.y, q.z, q.w);
}

inline glm::vec3 FromJoltVec3(const Vec3& v)
{
    return glm::vec3(v.GetX(), v.GetY(), v.GetZ());
}

inline glm::vec3 FromJoltQuat(const Quat& q)
{
    glm::quat gq(q.GetW(), q.GetX(), q.GetY(), q.GetZ());
    return glm::degrees(glm::eulerAngles(gq));
}


void RegisterPhysics_Box(
    ModelInstance& inst,
    const OBJData& mesh,
    float mass,
    float friction,
    float restitution,
    bool originAtBottom,
	glm::vec3 boxsize
)
{
    BodyInterface& bi = gPhysics->GetBodyInterface();

    glm::vec3 worldSize;
    if (boxsize != glm::vec3(0.0f))
    {
        worldSize = boxsize;
    }
    else
    {
        glm::vec3 meshSize = ComputeMeshSize(mesh);
        worldSize = meshSize * inst.scale;
    }

    Vec3 halfExtent(
        worldSize.x * 0.5f,
        worldSize.y * 0.5f,
        worldSize.z * 0.5f
    );

    BoxShapeSettings boxSettings(halfExtent);
    auto result = boxSettings.Create();
    if (result.HasError())
        return;

    RefConst<Shape> shape = result.Get();

    EMotionType motion = mass > 0.0f
        ? EMotionType::Dynamic
        : EMotionType::Static;

    glm::vec3 renderOffset =
        originAtBottom ? glm::vec3(0, worldSize.y * 0.5f, 0)
        : glm::vec3(0);

    BodyCreationSettings bcs(
        shape,
        ToJoltVec3(inst.position + renderOffset),
        ToJoltQuat(inst.rotation),
        motion,
        Layers::NON_MOVING
    );

    if (motion == EMotionType::Dynamic)
    {
        bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        bcs.mMassPropertiesOverride.mMass = mass;
        bcs.mObjectLayer = Layers::MOVING;
    }

    bcs.mFriction = friction;
    bcs.mRestitution = restitution;

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::Activate);

    gPhysicsLinks.push_back({
        &inst,
        body,
        renderOffset
        });
}

void RegisterPhysics_Convex(
    ModelInstance& inst,
    float mass,
    float friction,
    float restitution,
    bool originAtBottom
)
{
    const OBJData& mesh = inst.model;
    BodyInterface& bi = gPhysics->GetBodyInterface();

    Array<Vec3> points;
    for (size_t i = 0; i < mesh.vertexCoords.size(); i += 3)
    {
        glm::vec3 v(
            mesh.vertexCoords[i + 0],
            mesh.vertexCoords[i + 1],
            mesh.vertexCoords[i + 2]
        );

        // Apply model scale
        v *= inst.scale;

        points.push_back(ToJoltVec3(v));
    }

    if (points.empty())
        return;

    ConvexHullShapeSettings hullSettings(points);
    auto result = hullSettings.Create();
    if (result.HasError())
        return;

    RefConst<Shape> shape = result.Get();

    EMotionType motion = mass > 0.0f
        ? EMotionType::Dynamic
        : EMotionType::Static;

    glm::vec3 bboxMin(FLT_MAX);
    glm::vec3 bboxMax(-FLT_MAX);
    for (auto& p : points)
    {
        bboxMin = glm::min(bboxMin, FromJoltVec3(p));
        bboxMax = glm::max(bboxMax, FromJoltVec3(p));
    }
    glm::vec3 size = bboxMax - bboxMin;

    glm::vec3 renderOffset =
        originAtBottom ? glm::vec3(0, size.y * 0.5f, 0) : glm::vec3(0);

    BodyCreationSettings bcs(
        shape,
        ToJoltVec3(inst.position + renderOffset),
        ToJoltQuat(inst.rotation),
        motion,
        Layers::NON_MOVING
    );

    if (motion == EMotionType::Dynamic)
    {
        bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        bcs.mMassPropertiesOverride.mMass = mass;
        bcs.mObjectLayer = Layers::MOVING;
    }

    bcs.mFriction = friction;
    bcs.mRestitution = restitution;

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::Activate);

    gPhysicsLinks.push_back({
        &inst,
        body,
        renderOffset
        });
}

Camera CreateCamera(glm::vec3 pos, glm::vec3 rot, float fov) {
    Camera cam;
    cam.position = pos;
    cam.rotation = rot;
    cam.fov = fov;
    return cam;
}

glm::mat4 GetViewMatrix(const Camera& camera) {
    // Start with identity
    glm::mat4 view = glm::mat4(1.0f);

    // Apply rotations (pitch, yaw, roll)
    view = glm::rotate(view, glm::radians(camera.rotation.x), glm::vec3(1, 0, 0)); // pitch
    view = glm::rotate(view, glm::radians(camera.rotation.y), glm::vec3(0, 1, 0)); // yaw
    view = glm::rotate(view, glm::radians(camera.rotation.z), glm::vec3(0, 0, 1)); // roll

    // Apply translation (move opposite of camera position)
    view = glm::translate(view, -camera.position);
    return view;
}

void Transform(ModelInstance& instance, const glm::vec3& rotation, const glm::vec3& position) {
    instance.rotation = rotation;
    instance.position = position;
}

void rotateObject(ModelInstance& instance, const glm::vec3& rotation) {
    instance.rotation += rotation;
}

void moveObject(ModelInstance& instance, const glm::vec3& position) {
    instance.position += position;
}

ModelInstance& CreateObject(const std::string& path, OBJData& objDataVar, std::string name,
    const glm::vec3& pos, const glm::vec3& rot, const glm::vec3& scale)
{
    if (!loadOBJ(path, objDataVar)) {
        std::cerr << "[Scene] Failed to load .obj: " << path << std::endl;
        static ModelInstance dummy;
        return dummy;
    }

    ModelInstance instance = CreateModelInstance(objDataVar, pos, rot);
    instance.scale = scale;

    size_t elementCount = objDataVar.elementArray.size();
    instance.vertexData.reserve(elementCount * 11); // pos(3) + color(3) + normal(3) + uv(2)

    std::mt19937 rng((unsigned int)time(nullptr));
    std::uniform_real_distribution<float> dist(0.3f, 1.0f);

    instance.vertexData.clear();
    instance.elementData.clear();

    for (size_t i = 0; i < elementCount; ++i) {
        unsigned idx = objDataVar.elementArray[i];

        // Check vertex bounds
        if (idx * 3 + 2 >= objDataVar.vertexCoords.size()) {
            std::cerr << "[Scene] Warning: invalid vertex index " << idx << " in element array, skipping\n";
            continue; // skip this element completely
        }

        // --- Position ---
        glm::vec3 posV(
            objDataVar.vertexCoords[idx * 3 + 0],
            objDataVar.vertexCoords[idx * 3 + 1],
            objDataVar.vertexCoords[idx * 3 + 2]);

        // --- Color ---
        glm::vec3 colV(1.0f);
        if (idx * 3 + 2 < objDataVar.vertexColors.size())
            colV = glm::vec3(
                objDataVar.vertexColors[idx * 3 + 0],
                objDataVar.vertexColors[idx * 3 + 1],
                objDataVar.vertexColors[idx * 3 + 2]);

        // --- Normal ---
        glm::vec3 normV(0.0f);
        if (idx * 3 + 2 < objDataVar.vertexNormals.size())
            normV = glm::vec3(
                objDataVar.vertexNormals[idx * 3 + 0],
                objDataVar.vertexNormals[idx * 3 + 1],
                objDataVar.vertexNormals[idx * 3 + 2]);

        // --- UV ---
        glm::vec2 uvV(0.0f);
        if (idx * 2 + 1 < objDataVar.vertexUVs.size())
            uvV = glm::vec2(
                objDataVar.vertexUVs[idx * 2 + 0],
                objDataVar.vertexUVs[idx * 2 + 1]);

        // Store vertex
        instance.vertexData.insert(instance.vertexData.end(),
            { posV.x, posV.y, posV.z,
              colV.x, colV.y, colV.z,
              normV.x, normV.y, normV.z,
              uvV.x, uvV.y });

        // Add corresponding element index (0-based for this new vertex)
        instance.elementData.push_back((unsigned int)(instance.vertexData.size() / 11 - 1));
    }


    // --- Upload to GPU ---
    glGenVertexArrays(1, &instance.VAO);
    glGenBuffers(1, &instance.VBO);
    glGenBuffers(1, &instance.EBO);

    glBindVertexArray(instance.VAO);
    glBindBuffer(GL_ARRAY_BUFFER, instance.VBO);
    glBufferData(GL_ARRAY_BUFFER, instance.vertexData.size() * sizeof(float), instance.vertexData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, instance.EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, instance.elementData.size() * sizeof(unsigned int), instance.elementData.data(), GL_STATIC_DRAW);

    // Vertex attributes: pos(3), color(3), normal(3), uv(2)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)(9 * sizeof(float)));
    glEnableVertexAttribArray(3);

	sceneModels.push_back({ name, instance });

    std::cout << "[Scene] Added object from " << path
        << " | Pos(" << pos.x << "," << pos.y << "," << pos.z << ")"
        << " Rot(" << rot.x << "," << rot.y << "," << rot.z << ")"
        << " Scale(" << scale.x << "," << scale.y << "," << scale.z << ")\n";

    return sceneModels.back().instance;
}

void PrintObjectPosition(const ModelInstance& instance, const std::string& name)
{
    if (!name.empty())
        std::cout << "[Object: " << name << "] ";

    std::cout << "Position = ("
        << instance.position.x << ", "
        << instance.position.y << ", "
        << instance.position.z << ")\n";
}

void PrintObjectTransform(const ModelInstance& instance, const std::string& name)
{
    if (!name.empty())
        std::cout << "[Object: " << name << "]\n";

    std::cout << "  Position: ("
        << instance.position.x << ", "
        << instance.position.y << ", "
        << instance.position.z << ")\n";

    std::cout << "  Rotation: ("
        << instance.rotation.x << ", "
        << instance.rotation.y << ", "
        << instance.rotation.z << ")\n";

    std::cout << "  Scale: ("
        << instance.scale.x << ", "
        << instance.scale.y << ", "
        << instance.scale.z << ")\n";
}

void RenderModels(Shader& shader) {
    for (auto& sceneObj : sceneModels) {
		ModelInstance& instance = sceneObj.instance;
        glm::mat4 modelMatrix = ComputeModelMatrix(instance);
        shader.setMat4("model", modelMatrix);
        glBindVertexArray(instance.VAO);
        glDrawElements(GL_TRIANGLES, (GLsizei)instance.elementData.size(), GL_UNSIGNED_INT, 0);
    }
}

bool RemoveObject(ModelInstance& instance) {
    // Delete GPU resources
    glDeleteVertexArrays(1, &instance.VAO);
    glDeleteBuffers(1, &instance.VBO);
    glDeleteBuffers(1, &instance.EBO);

    // Remove from sceneModels
    auto it = std::remove_if(sceneModels.begin(), sceneModels.end(),
        [&](const ObjectList& obj) { return &obj.instance == &instance; });

    if (it != sceneModels.end()) {
        sceneModels.erase(it, sceneModels.end());
        std::cout << "[Scene] Removed object from scene.\n";
        return true;
    }
    else {
        std::cerr << "[Scene] Warning: Attempted to remove object not in scene.\n";
        return false;
    }
}

ModelInstance& GetObjectByName(const std::string& name) {
    for (auto& sceneObj : sceneModels) {
        if (sceneObj.name == name) {
            return sceneObj.instance;
        }
    }
    static ModelInstance dummy;
    std::cerr << "[Scene] Warning: Object with name '" << name << "' not found.\n";
    return dummy;
}