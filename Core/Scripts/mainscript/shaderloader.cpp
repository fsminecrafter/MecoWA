#include "shaderloader.h"
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_set>
#include <string>
#include <algorithm>

// ===========================================================
// Utility Functions
// ===========================================================

// Get directory of a path
static std::string GetDirectoryFromPath(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    return (pos == std::string::npos) ? "." : path.substr(0, pos);
}

// Normalize path separators
static std::string NormalizePath(const std::string& path) {
    std::string result = path;
    std::replace(result.begin(), result.end(), '\\', '/');
    return result;
}

// Check if file exists
static bool FileExists(const std::string& path) {
    std::ifstream f(path.c_str());
    return f.good();
}

static std::string LoadShaderWithIncludes(
    const std::string& filePath,
    std::unordered_set<std::string>& includedFiles,
    bool& versionInserted,
    bool isMainFile = true
)
{
    versionInserted = false;
    std::string absPath = NormalizePath(filePath);

    if (includedFiles.count(absPath)) {
        std::cerr << "// Skipping duplicate include: " << absPath << "\n";
        return "";
    }
    includedFiles.insert(absPath);

    if (!FileExists(absPath)) {
        std::cerr << "ERROR: Shader file not found: " << absPath << "\n";
        return "// Missing include: " + absPath + "\n";
    }

    std::ifstream file(absPath.c_str());
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open shader file: " << absPath << "\n";
        return "";
    }

    std::stringstream result;
    std::string line;
    std::string currentDir = GetDirectoryFromPath(absPath);

    while (std::getline(file, line)) {
        std::string trimmed = line;
        trimmed.erase(0, trimmed.find_first_not_of(" \t"));

        // Handle #include
        if (trimmed.rfind("#include", 0) == 0) {
            size_t q1 = trimmed.find('"');
            size_t q2 = trimmed.find_last_of('"');
            if (q1 != std::string::npos && q2 != std::string::npos && q2 > q1) {
                std::string includeFile = trimmed.substr(q1 + 1, q2 - q1 - 1);
                std::string includePath = NormalizePath(currentDir + "/" + includeFile);

                if (!FileExists(includePath)) {
                    std::cerr << "ERROR: Included shader not found: " << includePath << "\n";
                    result << "// Missing include: " << includePath << "\n";
                }
                else {
                    std::string includedCode = LoadShaderWithIncludes(includePath, includedFiles, versionInserted, false);
                    result << "\n// Begin include: " << includeFile << "\n"
                        << includedCode
                        << "\n// End include: " << includeFile << "\n";
                }
            }
            else {
                std::cerr << "WARNING: Malformed #include in " << absPath << ": " << line << "\n";
                result << "// Malformed include ignored: " << line << "\n";
            }
            continue;
        }

        // Strip in/out only for included files
        if (!isMainFile && (trimmed.rfind("out ", 0) == 0 || trimmed.rfind("in ", 0) == 0)) {
            result << "// Ignored in/out line in include: " << trimmed << "\n";
            continue;
        }

        result << line << "\n";
    }

    return result.str();
}

// ===========================================================
// Shader class implementation
// ===========================================================

#define DEBUG_SHADER_INCLUDE 0  // set to 1 to see resolved shader source

Shader::Shader(const char* vertexPath, const char* fragmentPath) {
    std::unordered_set<std::string> included;
    bool versionInserted = false;

    std::string vertexCode = LoadShaderWithIncludes(vertexPath, included, versionInserted);
    std::string fragmentCode = LoadShaderWithIncludes(fragmentPath, included, versionInserted);

#if DEBUG_SHADER_INCLUDE
    std::cout << "----- Resolved Vertex Shader -----\n";
    std::cout << vertexCode << "\n";
    std::cout << "----- Resolved Fragment Shader -----\n";
    std::cout << fragmentCode << "\n";
#endif

    const char* vShaderSource = vertexCode.c_str();
    const char* fShaderSource = fragmentCode.c_str();

    unsigned int vertex, fragment;
    int success;
    char infoLog[1024];

    // Vertex Shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderSource, NULL);
    glCompileShader(vertex);
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertex, 1024, NULL, infoLog);
        std::cerr << "ERROR: Vertex shader compilation failed\n" << infoLog << std::endl;
    }

    // Fragment Shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderSource, NULL);
    glCompileShader(fragment);
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragment, 1024, NULL, infoLog);
        std::cerr << "ERROR: Fragment shader compilation failed\n" << infoLog << std::endl;
    }

    // Shader Program
    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);
    glGetProgramiv(ID, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(ID, 1024, NULL, infoLog);
        std::cerr << "ERROR: Shader linking failed\n" << infoLog << std::endl;
    }

    glDeleteShader(vertex);
    glDeleteShader(fragment);
}


void Shader::use() const {
    glUseProgram(ID);
}

void Shader::setBool(const std::string& name, bool value) const {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
}

void Shader::setInt(const std::string& name, int value) const {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setFloat(const std::string& name, float value) const {
    glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}

void Shader::setMat4(const std::string& name, const glm::mat4& mat) const {
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

void Shader::setVec3(const std::string& name, const glm::vec3& value) const {
    glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(value));
}
