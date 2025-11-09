#pragma once
#include <vector>
#include <string>

struct ROBJData {
    std::vector<float> vertexCoords;
    std::vector<float> vertexColors;
    std::vector<int> elementArray;
};

bool loadROBJ(const std::string& path, ROBJData& outData);
