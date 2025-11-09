#include "objloader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>
#include <limits>
#include <ctime>
#include <glm/glm.hpp>
#include <unordered_map>

bool loadOBJ(const std::string& path, OBJData& out)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[OBJLoader] Failed to open file: " << path << std::endl;
        return false;
    }

    std::vector<glm::vec3> temp_vertices;
    std::vector<glm::vec3> temp_normals;
    std::vector<unsigned int> vertexIndices;
    std::vector<unsigned int> normalIndices;
    std::vector<glm::vec2> temp_uvs; // texture coordinates
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
        else if (prefix == "f") {
            std::string vStr[3];
            ss >> vStr[0] >> vStr[1] >> vStr[2];

            for (int i = 0; i < 3; ++i) {
                unsigned int vIdx = 0, vtIdx = 0, vnIdx = 0;
                std::string& token = vStr[i];

                size_t firstSlash = token.find('/');
                size_t lastSlash = token.rfind('/');

                if (firstSlash == std::string::npos) {
                    // Format: v
                    vIdx = std::stoi(token) - 1;
                }
                else if (firstSlash == lastSlash) {
                    // Format: v/vt
                    vIdx = std::stoi(token.substr(0, firstSlash)) - 1;
                    vtIdx = std::stoi(token.substr(firstSlash + 1)) - 1;
                }
                else {
                    // Format: v/vt/vn or v//vn
                    vIdx = std::stoi(token.substr(0, firstSlash)) - 1;
                    if (lastSlash > firstSlash + 1)
                        vtIdx = std::stoi(token.substr(firstSlash + 1, lastSlash - firstSlash - 1)) - 1;
                    vnIdx = std::stoi(token.substr(lastSlash + 1)) - 1;
                }

                vertexIndices.push_back(vIdx);
                uvIndices.push_back(vtIdx);
                normalIndices.push_back(vnIdx);
            }
        }
        else if (prefix == "vt") {
            glm::vec2 uv;
            ss >> uv.x >> uv.y;
            temp_uvs.push_back(uv);
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

    // --- Compute per-vertex normals if not provided ---
    std::vector<glm::vec3> vertexNormals(temp_vertices.size(), glm::vec3(0.0f));

    if (temp_normals.empty()) {
        // Compute normals from faces
        for (size_t i = 0; i < vertexIndices.size(); i += 3) {
            unsigned int i0 = vertexIndices[i + 0];
            unsigned int i1 = vertexIndices[i + 1];
            unsigned int i2 = vertexIndices[i + 2];

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
        // Use loaded normals
        for (size_t i = 0; i < vertexIndices.size(); ++i) {
            unsigned int vIdx = vertexIndices[i];
            unsigned int nIdx = normalIndices[i];
            if (nIdx < temp_normals.size())
                vertexNormals[vIdx] = temp_normals[nIdx];
        }
    }

    // --- Combine position/normal/uv into unique vertices ---
    struct VertexKey {
        unsigned v, vt, vn;
        bool operator==(const VertexKey& o) const noexcept {
            return v == o.v && vt == o.vt && vn == o.vn;
        }
    };
    struct VertexKeyHasher {
        size_t operator()(const VertexKey& k) const noexcept {
            return ((size_t)k.v * 73856093) ^ ((size_t)k.vt * 19349663) ^ ((size_t)k.vn * 83492791);
        }
    };

    std::unordered_map<VertexKey, unsigned, VertexKeyHasher> uniqueMap;

    out.vertexCoords.clear();
    out.vertexNormals.clear();
    out.vertexColors.clear();
    out.vertexUVs.clear();
    out.elementArray.clear();

    std::mt19937 rng((unsigned int)time(nullptr));
    std::uniform_real_distribution<float> dist(0.3f, 1.0f);

    for (size_t i = 0; i < vertexIndices.size(); ++i) {
        VertexKey key{ vertexIndices[i],
                        i < uvIndices.size() ? uvIndices[i] : (unsigned)-1,
                        i < normalIndices.size() ? normalIndices[i] : (unsigned)-1 };

        auto it = uniqueMap.find(key);
        if (it == uniqueMap.end()) {
            // position
            const glm::vec3& pos = temp_vertices[key.v];
            out.vertexCoords.insert(out.vertexCoords.end(), { pos.x, pos.y, pos.z });

            // normal
            glm::vec3 n(0);
            if (key.vn != (unsigned)-1 && key.vn < temp_normals.size())
                n = temp_normals[key.vn];
            else if (!temp_normals.empty())
                n = temp_normals[key.v % temp_normals.size()];
            out.vertexNormals.insert(out.vertexNormals.end(), { n.x, n.y, n.z });

            // uv
            if (key.vt != (unsigned)-1 && key.vt < temp_uvs.size()) {
                const glm::vec2& uv = temp_uvs[key.vt];
                out.vertexUVs.insert(out.vertexUVs.end(), { uv.x, uv.y });
            }
            else {
                out.vertexUVs.insert(out.vertexUVs.end(), { 0.0f, 0.0f });
            }

            // random color
            out.vertexColors.insert(out.vertexColors.end(), { dist(rng), dist(rng), dist(rng) });

            unsigned newIndex = (unsigned)(out.vertexCoords.size() / 3 - 1);
            uniqueMap[key] = newIndex;
            out.elementArray.push_back(newIndex);
        }
        else {
            out.elementArray.push_back(it->second);
        }
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