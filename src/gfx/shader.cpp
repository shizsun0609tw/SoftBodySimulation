#include "shader.h"

#include <fstream>

namespace gfx {

Shader::Shader(util::fs::path filePath, GLenum shaderType) {
    id = glCreateShader(shaderType);
    auto shaderCode = loadFromFile(filePath);
    auto shaderCodePointer = shaderCode.c_str();
    glShaderSource(id, 1, &shaderCodePointer, nullptr);
    glCompileShader(id);
    GLint success;
    GLchar infoLog[1024];
    glGetShaderiv(id, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(id, 1024, nullptr, infoLog);
        puts("Shader compilation error!");
        puts(infoLog);
    }
}

Shader::~Shader() { glDeleteShader(id); }

GLuint Shader::getID() const { return id; }

std::string Shader::loadFromFile(util::fs::path filename) {
    std::ifstream shaderFile(filename);
    if (!shaderFile.is_open()) {
        puts("Failed to open shader file!");
        return "";
    }
    auto shaderCode = std::string(std::istreambuf_iterator<char>(shaderFile), std::istreambuf_iterator<char>());
    shaderFile.close();
    return shaderCode;
}

Program::Program() { id = glCreateProgram(); }

Program::~Program() { glDeleteProgram(id); }

void Program::linkShader(const Shader& shader) {
    glAttachShader(id, shader.getID());
    glLinkProgram(id);

    GLint success;
    GLchar infoLog[1024];
    glGetProgramiv(id, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(id, 1024, nullptr, infoLog);
        puts("Failed to link shader program!");
        puts(infoLog);
    }
}

GLuint Program::getID() const { return id; }

int Program::getUniformLocation(const char* name) const { return glGetUniformLocation(id, name); }

void Program::use() const { glUseProgram(id); }

void Program::setUniform(const char* name, int i1) { glUniform1i(getUniformLocation(name), i1); }

void Program::setUniform(const char* name, const Eigen::Matrix4f& mat4) {
    glUniformMatrix4fv(getUniformLocation(name), 1, GL_FALSE, mat4.data());
}

void Program::setUniform(const char* name, const Eigen::Matrix3f& mat3) {
    glUniformMatrix3fv(getUniformLocation(name), 1, GL_FALSE, mat3.data());
}

void Program::setUniform(const char* name, const Eigen::Vector3f& vec3) {
    glUniform3fv(getUniformLocation(name), 1, vec3.data());
}

}  // namespace gfx
