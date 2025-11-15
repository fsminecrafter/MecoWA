#include "objloader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>
#include <limits>
#include <ctime>
#include <glm/glm.hpp>
#include <unordered_map>
#include <cassert>

bool loadOBJ(const std::string& path, OBJData& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[OBJLoader] Failed to open file: " << path << std::endl;
        return false;
    }

    std::vector<glm::vec3> temp_vertices;
    std::vector<glm::vec3> temp_normals;
    std::vector<unsigned int> vertexIndices;
    std::vector<unsigned int> normalIndices;
    std::vector<glm::vec2> temp_uvs;
    std::vector<unsigned int> uvIndices;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string prefix;
        ss >> prefix;

        if (prefix == "v") {
            glm::vec3 vertex;
            ss >> vertex.x >> vertex.y >> vertex.z;
            temp_vertices.push_back(vertex);
        }
        else if (prefix == "vn") {
            glm::vec3 normal;
            ss >> normal.x >> normal.y >> normal.z;
            temp_normals.push_back(normal);
        }
        else if (prefix == "vt") {
            glm::vec2 uv;
            ss >> uv.x >> uv.y;
            temp_uvs.push_back(uv);
        }
        else if (prefix == "f") {
            std::string vStr[3];
            ss >> vStr[0] >> vStr[1] >> vStr[2];

            for (int i = 0; i < 3; ++i) {
                unsigned int vIdx = 0, vtIdx = -1, vnIdx = -1;
                std::string& token = vStr[i];

                size_t firstSlash = token.find('/');
                size_t lastSlash = token.rfind('/');

                try {
                    if (firstSlash == std::string::npos) {
                        // v
                        vIdx = std::stoi(token) - 1;
                    }
                    else if (firstSlash == lastSlash) {
                        // v/vt
                        vIdx = std::stoi(token.substr(0, firstSlash)) - 1;
                        vtIdx = std::stoi(token.substr(firstSlash + 1)) - 1;
                    }
                    else {
                        // v/vt/vn or v//vn
                        vIdx = std::stoi(token.substr(0, firstSlash)) - 1;
                        if (lastSlash > firstSlash + 1)
                            vtIdx = std::stoi(token.substr(firstSlash + 1, lastSlash - firstSlash - 1)) - 1;
                        vnIdx = std::stoi(token.substr(lastSlash + 1)) - 1;
                    }
                }
                catch (...) {
                    std::cerr << "[OBJLoader] Warning: invalid face token '" << token << "'\n";
                    vIdx = -1; vtIdx = -1; vnIdx = -1;
                }

                vertexIndices.push_back(vIdx);
                uvIndices.push_back(vtIdx);
                normalIndices.push_back(vnIdx);
            }
        }
    }
    file.close();

    if (temp_vertices.empty()) {
        std::cerr << "[OBJLoader] No vertices found in " << path << std::endl;
        return false;
    }

    // Compute bounding box and center
    glm::vec3 minV(std::numeric_limits<float>::max());
    glm::vec3 maxV(std::numeric_limits<float>::lowest());
    for (const auto& v : temp_vertices) {
        minV = glm::min(minV, v);
        maxV = glm::max(maxV, v);
    }
    glm::vec3 center = (minV + maxV) * 0.5f;
    for (auto& v : temp_vertices) v -= center;

    // Compute normals if missing
    std::vector<glm::vec3> vertexNormals(temp_vertices.size(), glm::vec3(0.0f));
    if (temp_normals.empty()) {
        for (size_t i = 0; i < vertexIndices.size(); i += 3) {
            int i0 = vertexIndices[i];
            int i1 = vertexIndices[i + 1];
            int i2 = vertexIndices[i + 2];

            if (i0 < 0 || i1 < 0 || i2 < 0 ||
                i0 >= temp_vertices.size() ||
                i1 >= temp_vertices.size() ||
                i2 >= temp_vertices.size()) {
                std::cerr << "[OBJLoader] Warning: face with invalid vertex index, skipping\n";
                continue;
            }

            glm::vec3 v0 = temp_vertices[i0];
            glm::vec3 v1 = temp_vertices[i1];
            glm::vec3 v2 = temp_vertices[i2];

            glm::vec3 faceNormal = glm::normalize(glm::cross(v1 - v0, v2 - v0));

            vertexNormals[i0] += faceNormal;
            vertexNormals[i1] += faceNormal;
            vertexNormals[i2] += faceNormal;
        }
        for (auto& n : vertexNormals) n = glm::normalize(n);
    }
    else {
        for (size_t i = 0; i < vertexIndices.size(); ++i) {
            unsigned int vIdx = vertexIndices[i];
            unsigned int nIdx = normalIndices[i];
            if (vIdx < temp_vertices.size() && nIdx < temp_normals.size())
                vertexNormals[vIdx] = temp_normals[nIdx];
        }
    }

    // Reserve output vectors to avoid reallocations
    size_t estimatedVerts = temp_vertices.size();
    out.vertexCoords.reserve(estimatedVerts * 3);
    out.vertexNormals.reserve(estimatedVerts * 3);
    out.vertexUVs.reserve(estimatedVerts * 2);
    out.vertexColors.reserve(estimatedVerts * 3);
    out.elementArray.reserve(vertexIndices.size());

    // Unique vertex map
    struct VertexKey { unsigned v, vt, vn; bool operator==(const VertexKey& o) const noexcept { return v == o.v && vt == o.vt && vn == o.vn; } };
    struct VertexKeyHasher { size_t operator()(const VertexKey& k) const noexcept { return ((size_t)k.v * 73856093) ^ ((size_t)k.vt * 19349663) ^ ((size_t)k.vn * 83492791); } };
    std::unordered_map<VertexKey, unsigned, VertexKeyHasher> uniqueMap;

    std::mt19937 rng((unsigned int)time(nullptr));
    std::uniform_real_distribution<float> dist(0.3f, 1.0f);

    for (size_t i = 0; i < vertexIndices.size(); ++i) {
        unsigned vIdx = vertexIndices[i];
        unsigned vtIdx = i < uvIndices.size() ? uvIndices[i] : (unsigned)-1;
        unsigned vnIdx = i < normalIndices.size() ? normalIndices[i] : (unsigned)-1;

        // Skip invalid indices
        if (vIdx >= temp_vertices.size()) continue;
        if (vtIdx != (unsigned)-1 && vtIdx >= temp_uvs.size()) vtIdx = (unsigned)-1;
        if (vnIdx != (unsigned)-1 && vnIdx >= temp_normals.size()) vnIdx = (unsigned)-1;

        VertexKey key{ vIdx, vtIdx, vnIdx };
        auto it = uniqueMap.find(key);
        if (it == uniqueMap.end()) {
            const glm::vec3& pos = temp_vertices[vIdx];
            glm::vec3 norm = (vnIdx != (unsigned)-1) ? temp_normals[vnIdx] : vertexNormals[vIdx];

            out.vertexCoords.insert(out.vertexCoords.end(), { pos.x,pos.y,pos.z });
            out.vertexNormals.insert(out.vertexNormals.end(), { norm.x,norm.y,norm.z });
            if (vtIdx != (unsigned)-1) {
                const glm::vec2& uv = temp_uvs[vtIdx];
                out.vertexUVs.insert(out.vertexUVs.end(), { uv.x,uv.y });
            }
            else out.vertexUVs.insert(out.vertexUVs.end(), { 0.0f,0.0f });

            out.vertexColors.insert(out.vertexColors.end(), { dist(rng), dist(rng), dist(rng) });

            unsigned newIndex = (unsigned)(out.vertexCoords.size() / 3 - 1);
            uniqueMap[key] = newIndex;
            out.elementArray.push_back(newIndex);
        }
        else out.elementArray.push_back(it->second);
    }

    std::cout << "[OBJLoader] Loaded "
        << temp_vertices.size() << " vertices, "
        << temp_uvs.size() << " uvs, "
        << temp_normals.size() << " normals, "
        << vertexIndices.size() / 3 << " faces from " << path << std::endl;

    std::cout << "[OBJLoader] Centered at origin. Bounding box: min("
        << minV.x << "," << minV.y << "," << minV.z << "), max("
        << maxV.x << "," << maxV.y << "," << maxV.z << ")\n";

    return true;
}
