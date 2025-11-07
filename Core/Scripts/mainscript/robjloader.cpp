#include "robjloader.h"
#include <fstream>
#include <sstream>
#include <regex>
#include <iostream>

static std::vector<float> parseFloatArray(const std::string& text, const std::string& name) {
    std::regex pattern(name + R"(\s*\[.*?\]\s*=\s*\{([^}]*)\})");
    std::smatch match;
    std::vector<float> values;

    if (std::regex_search(text, match, pattern)) {
        std::string arrayData = match[1];
        std::stringstream ss(arrayData);
        std::string num;
        while (std::getline(ss, num, ',')) {
            try {
                values.push_back(std::stof(num));
            }
            catch (...) {}
        }
    }
    else {
        std::cerr << "[ROBJ Loader] Missing float array: " << name << std::endl;
    }
    return values;
}

static std::vector<int> parseIntArray(const std::string& text, const std::string& name) {
    std::regex pattern(name + R"(\s*\[.*?\]\s*=\s*\{([^}]*)\})");
    std::smatch match;
    std::vector<int> values;

    if (std::regex_search(text, match, pattern)) {
        std::string arrayData = match[1];
        std::stringstream ss(arrayData);
        std::string num;
        while (std::getline(ss, num, ',')) {
            try {
                values.push_back(std::stoi(num));
            }
            catch (...) {}
        }
    }
    else {
        std::cerr << "[ROBJ Loader] Missing int array: " << name << std::endl;
    }
    return values;
}

bool loadROBJ(const std::string& path, ROBJData& outData) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ROBJ Loader] Failed to open file: " << path << std::endl;
        return false;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());
    file.close();

    outData.vertexCoords = parseFloatArray(content, "vertexCoords");
    outData.vertexColors = parseFloatArray(content, "vertexColors");
    outData.elementArray = parseIntArray(content, "elementArray");

    if (outData.vertexCoords.empty() || outData.vertexColors.empty() || outData.elementArray.empty()) {
        std::cerr << "[ROBJ Loader] One or more arrays failed to load from " << path << std::endl;
        return false;
    }

    std::cout << "[ROBJ Loader] Loaded " << path << " successfully.\n";
    std::cout << " - vertexCoords: " << outData.vertexCoords.size() << " floats\n";
    std::cout << " - vertexColors: " << outData.vertexColors.size() << " floats\n";
    std::cout << " - elementArray: " << outData.elementArray.size() << " ints\n";

    return true;
}
