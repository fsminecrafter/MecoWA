
// Cherry engine
// Made by Joel_minecrafter


#include "engine.h"
#include <iostream>
#include <vector>
#include "objloader.h"
#include <random>
#include <ctime>

#include "jolt_bridge.h"

std::vector<ModelInstance> sceneModels;

//#define DEBUG

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
    float mass,
    float friction,
    float restitution 
)
{
    BodyInterface& bi = gPhysics->GetBodyInterface();

    Vec3 halfExtent(inst.scale.x * 0.5f, inst.scale.y * 0.5f, inst.scale.z * 0.5f);

    BoxShapeSettings boxSettings(halfExtent, 0.0f); // convex radius = 0
    ShapeSettings::ShapeResult result = boxSettings.Create();

    if (result.HasError())
    {
        std::cerr << "[Jolt] Failed to create BoxShape\n";
        return;
    }

    RefConst<Shape> shape = result.Get();

    EMotionType motion = mass > 0.0f ? EMotionType::Dynamic : EMotionType::Static;

    BodyCreationSettings bcs(
        shape,
        ToJoltVec3(inst.position),
        ToJoltQuat(inst.rotation),
        motion,
        0
    );

    if (motion == EMotionType::Dynamic)
        bcs.mMassPropertiesOverride.mMass = mass;

    bcs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    bcs.mFriction = friction;
    bcs.mRestitution = restitution;

    BodyID body = bi.CreateAndAddBody(bcs, EActivation::Activate);

    gPhysicsLinks.push_back({ &inst, body });
}

void Physics_SyncToEngine()
{
    for (auto& link : gPhysicsLinks)
    {
        BodyLockRead lock(gPhysics->GetBodyLockInterface(), link.body);
        if (!lock.Succeeded())
            continue;

        const Body& body = lock.GetBody();

        link.model->position = FromJoltVec3(body.GetPosition());
        link.model->rotation = FromJoltQuat(body.GetRotation());
    }
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

ModelInstance& CreateObject(const std::string& path, OBJData& objDataVar,
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

    sceneModels.push_back(instance);

    std::cout << "[Scene] Added object from " << path
        << " | Pos(" << pos.x << "," << pos.y << "," << pos.z << ")"
        << " Rot(" << rot.x << "," << rot.y << "," << rot.z << ")"
        << " Scale(" << scale.x << "," << scale.y << "," << scale.z << ")\n";

    return sceneModels.back();
}

void RenderModels(Shader& shader) {
    for (auto& instance : sceneModels) {
        glm::mat4 modelMatrix = ComputeModelMatrix(instance);
        shader.setMat4("model", modelMatrix);
        glBindVertexArray(instance.VAO);
        glDrawElements(GL_TRIANGLES, (GLsizei)instance.elementData.size(), GL_UNSIGNED_INT, 0);
    }
}