#pragma once
#include <vector>
#include <string>
#include <glm/glm/glm.hpp>

struct OBJData {
    std::vector<float> vertexCoords;   // x, y, z
    std::vector<float> vertexNormals;  // nx, ny, nz
    std::vector<float> vertexColors;   // r, g, b
    std::vector<unsigned int> elementArray; // indices
	std::vector<float> vertexUVs; // vertex uv stuff

};


bool loadOBJ(const std::string& path, OBJData& out);
