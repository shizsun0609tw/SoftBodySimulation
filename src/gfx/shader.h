#pragma once
#include <string>

#include "Eigen/Dense"
#include "glad/glad.h"

#include "../util/filesystem.h"

namespace gfx {
class Shader final {
 public:
    Shader(util::fs::path shader, GLenum shaderType);
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    ~Shader();

    GLuint getID() const;

 private:
    std::string loadFromFile(util::fs::path filename);
    GLuint id;
};

class Program final {
 public:
    Program();
    Program(const Program&) = delete;
    Program& operator=(const Program&) = delete;
    ~Program();

    void linkShader(const Shader& shader);
    template <typename... Args>
    void linkShader(const Shader& shader, Args&&... args) {
        glAttachShader(id, shader.getID());
        linkShader(args...);
    }

    GLuint getID() const;
    int getUniformLocation(const char* name) const;
    void use() const;
    void setUniform(const char* name, int i1);
    void setUniform(const char* name, const Eigen::Matrix4f& mat4);
    void setUniform(const char* name, const Eigen::Matrix3f& mat3);
    void setUniform(const char* name, const Eigen::Vector3f& vec3);

 private:
    GLuint id;
};
}  // namespace gfx
