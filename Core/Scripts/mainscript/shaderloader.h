#pragma once
#include <glad/include/glad/glad.h>
#include <glm/glm/glm.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glm/glm/gtc/type_ptr.hpp>


class Shader {
public:
    unsigned int ID;

    // Constructor — load and compile the shader
    Shader(const char* vertexPath, const char* fragmentPath);

    // Activate the shader
    void use() const;

    // Utility uniform functions
    void setBool(const std::string& name, bool value) const;
    void setInt(const std::string& name, int value) const;
    void setFloat(const std::string& name, float value) const;
    void setMat4(const std::string& name, const glm::mat4& mat) const;
    void setVec3(const std::string& name, const glm::vec3& value) const;
};