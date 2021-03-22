#include "graphic.h"

#include <algorithm>

#include "Eigen/Geometry"
#include "glad/glad.h"

#include "../simulation/particle.h"
#include "../util/helper.h"

namespace {
constexpr int g_SphereSectors = 50;
constexpr int g_SphereStacks = 50;
constexpr int g_SphereRadius = 1;
}  // namespace

namespace gfx {

std::vector<GLfloat> Plane::vertices;
std::vector<GLfloat> Sphere::vertices;
std::vector<GLuint> Sphere::fillIndices = std::vector<GLuint>();
std::vector<GLuint> Sphere::wireIndices = std::vector<GLuint>();
std::vector<GLfloat> SkyBox::vertices = {-1.0f, -1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f,
                                         -1.0f, -1.0f, 1.0f,  1.0f, -1.0f, 1.0f,  1.0f, 1.0f, 1.0f,  -1.0f, 1.0f, 1.0f};
std::vector<GLuint> SkyBox::indices = {0, 1, 3, 3, 1, 2, 1, 5, 2, 2, 5, 6, 5, 4, 6, 6, 4, 7,
                                       4, 0, 7, 7, 0, 3, 3, 2, 7, 7, 2, 6, 4, 5, 0, 0, 5, 1};

Rigidbody::Rigidbody() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // Vertex
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), nullptr);
    // normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), reinterpret_cast<void*>(3 * sizeof(GLfloat)));
    // Texture
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), reinterpret_cast<void*>(6 * sizeof(GLfloat)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

Rigidbody::~Rigidbody() {
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
}

void Rigidbody::setModelMatrix(const Eigen::Matrix4f& _modelMatrix) {
    modelMatrix = _modelMatrix;
    inverseTransposeModel = modelMatrix.block(0, 0, 3, 3).inverse().transpose();
}

void Rigidbody::setTexture(const std::shared_ptr<Texture>& _texture) { texture = _texture; }

void Rigidbody::setTexture(const Eigen::Vector3f& color) { baseColor = color; }

void Rigidbody::initialize() {
    generateVertices();
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertexPointer->size() * sizeof(GLfloat), vertexPointer->data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

Plane::Plane() {
    if (vertexPointer == nullptr) {
        initialize();
    }
}

void Plane::generateVertices() {
    vertices = {-0.5f, 0.0f, -0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, -0.5f, 0.0f, 0.5f,  0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
                0.5f,  0.0f, 0.5f,  0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.5f,  0.0f, -0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    vertexPointer = &vertices;
}

void Plane::render(Program* shaderProgram) {
    if (texture) {
        shaderProgram->setUniform("useTexture", 1);
        shaderProgram->setUniform("diffuseTexture", texture->getIndex());
    } else {
        shaderProgram->setUniform("useTexture", 0);
        shaderProgram->setUniform("baseColor", baseColor);
    }
    shaderProgram->setUniform("model", modelMatrix);
    shaderProgram->setUniform("invtransmodel", inverseTransposeModel);
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
}

Sphere::Sphere() {
    glGenVertexArrays(1, &wireVAO);
    if (vertexPointer == nullptr) {
        initialize();
    }

    glBindVertexArray(wireVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // Vertex
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), nullptr);
    // normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), reinterpret_cast<void*>(3 * sizeof(GLfloat)));
    // Texture
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), reinterpret_cast<void*>(6 * sizeof(GLfloat)));

    glGenBuffers(1, &wireEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wireEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, wireIndices.size() * sizeof(GLuint), wireIndices.data(), GL_STATIC_DRAW);

    glBindVertexArray(vao);
    glGenBuffers(1, &fillEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, fillEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, fillIndices.size() * sizeof(GLuint), fillIndices.data(), GL_STATIC_DRAW);
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

Sphere::~Sphere() {
    glDeleteVertexArrays(1, &wireVAO);
    glDeleteBuffers(1, &wireEBO);
    glDeleteBuffers(1, &fillEBO);
}

void Sphere::setRenderMode(RenderMode _mode) { mode = _mode; }

void Sphere::generateVertices() {
    // http://www.songho.ca/opengl/gl_sphere.html#sphere
    vertices.reserve(8 * (g_SphereStacks + 1) * (g_SphereSectors + 1));
    fillIndices.reserve(6 * g_SphereStacks * g_SphereSectors);
    wireIndices.reserve(4 * g_SphereStacks * g_SphereSectors);

    float x, y, z, xy;                                    //  position
    float nx, ny, nz, lengthInv = 1.0f / g_SphereRadius;  //  normal
    float s, t;                                           //  texCoord

    constexpr float sectorStep = 2.0f * util::PI<float>() / g_SphereSectors;
    constexpr float stackStep = util::PI<float>() / g_SphereStacks;
    float sectorAngle, stackAngle;

    for (int i = 0; i <= g_SphereStacks; ++i) {
        stackAngle = util::PI<float>() / 2.0f - i * stackStep;  // [pi/2, -pi/2]
        xy = cosf(stackAngle);                                  // r * cos(u)
        z = sinf(stackAngle);                                   // r * sin(u)

        for (int j = 0; j <= g_SphereSectors; ++j) {
            sectorAngle = j * sectorStep;  // [0, 2pi]

            x = xy * cosf(sectorAngle);  // r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle);  // r * cos(u) * sin(v)

            // normalized vertex normal (nx, ny, nz)
            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;

            // vertex tex coord (s, t) range between [0, 1]
            s = static_cast<float>(j) / g_SphereSectors;
            t = static_cast<float>(i) / g_SphereStacks;

            vertices.insert(vertices.end(), {x, y, z, nx, ny, nz, s, t});
        }
    }
    vertexPointer = &vertices;
    unsigned int k1, k2;  // EBO index
    for (int i = 0; i < g_SphereStacks; ++i) {
        k1 = i * (g_SphereSectors + 1);  // beginning of current stack
        k2 = k1 + g_SphereSectors + 1;   // beginning of next stack
        for (int j = 0; j < g_SphereSectors; ++j, ++k1, ++k2) {
            wireIndices.insert(wireIndices.end(), {k1, k2});
            if (i != 0) {
                wireIndices.insert(wireIndices.end(), {k1, k1 + 1});
                fillIndices.insert(fillIndices.end(), {k1, k2, k1 + 1});
            }
            // k1+1 => k2 => k2+1
            if (i != (g_SphereStacks - 1)) {
                fillIndices.insert(fillIndices.end(), {k1 + 1, k2, k2 + 1});
            }
        }
    }
}

void Sphere::render(Program* shaderProgram) {
    if (texture) {
        shaderProgram->setUniform("useTexture", 1);
        shaderProgram->setUniform("diffuseTexture", texture->getIndex());
    } else {
        shaderProgram->setUniform("useTexture", 0);
        shaderProgram->setUniform("baseColor", baseColor);
    }
    shaderProgram->setUniform("model", modelMatrix);
    shaderProgram->setUniform("invtransmodel", inverseTransposeModel);
    if (mode == RenderMode::FILLED) {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(fillIndices.size()), GL_UNSIGNED_INT, nullptr);
    } else {
        glBindVertexArray(wireVAO);
        glDrawElements(GL_LINES, static_cast<GLsizei>(wireIndices.size()), GL_UNSIGNED_INT, nullptr);
    }
    glBindVertexArray(0);
}

SoftCube::SoftCube(simulation::Cube* _cube) {
    cube = _cube;

    glGenVertexArrays(6, cubeVAO);
    glGenVertexArrays(1, &structVAO);
    glGenVertexArrays(1, &shearVAO);
    glGenVertexArrays(1, &bendingVAO);
    glGenVertexArrays(1, &particleVAO);

    glGenBuffers(1, &vertexVBO);
    glGenBuffers(1, &textureCoordVBO);
    glGenBuffers(6, cubeVBO);
    glGenBuffers(6, normalVBO);

    glGenBuffers(1, &structEBO);
    glGenBuffers(1, &bendingEBO);
    glGenBuffers(1, &shearEBO);
    glGenBuffers(2, cubeEBO);

    allocateBuffers();
    calculateTextureCoords();
    calculateIndices();
    initializeBuffers();
    update();
}

SoftCube::~SoftCube() {
    glDeleteVertexArrays(6, cubeVAO);
    glDeleteVertexArrays(1, &structVAO);
    glDeleteVertexArrays(1, &shearVAO);
    glDeleteVertexArrays(1, &bendingVAO);
    glDeleteVertexArrays(1, &particleVAO);

    glDeleteBuffers(1, &vertexVBO);
    glDeleteBuffers(1, &textureCoordVBO);
    glDeleteBuffers(6, cubeVBO);
    glDeleteBuffers(6, normalVBO);

    glDeleteBuffers(1, &structEBO);
    glDeleteBuffers(1, &bendingEBO);
    glDeleteBuffers(1, &shearEBO);
    glDeleteBuffers(2, cubeEBO);
}

void SoftCube::allocateBuffers() {
    int edgeNum = cube->getNumAtEdge();
    int totalSize = edgeNum * edgeNum * sizeof(GLfloat);
    int vertexSize = 3 * cube->getParticleNum() * sizeof(GLfloat);
    int indexSize = 6 * edgeNum * edgeNum * sizeof(GLuint);
    // Temporary
    faceVertices = std::vector<GLfloat>(3 * edgeNum * edgeNum);
    normalBuffer = std::vector<float>(3 * edgeNum * edgeNum);
    fullVertices = std::vector<float>(3 * cube->getParticleNum());
    // All vertices
    glBindBuffer(GL_ARRAY_BUFFER, vertexVBO);
    glBufferData(GL_ARRAY_BUFFER, vertexSize, nullptr, GL_STREAM_DRAW);
    // Texture
    glBindBuffer(GL_ARRAY_BUFFER, textureCoordVBO);
    glBufferData(GL_ARRAY_BUFFER, 2 * totalSize, nullptr, GL_STATIC_DRAW);

    for (int face = 0; face < 6; ++face) {
        glBindBuffer(GL_ARRAY_BUFFER, cubeVBO[face]);
        glBufferData(GL_ARRAY_BUFFER, 3 * totalSize, nullptr, GL_STREAM_DRAW);
        // Normal
        glBindBuffer(GL_ARRAY_BUFFER, normalVBO[face]);
        glBufferData(GL_ARRAY_BUFFER, 3 * totalSize, nullptr, GL_STREAM_DRAW);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // Front and back of cube
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexSize, nullptr, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexSize, nullptr, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void SoftCube::calculateIndices() {
    int edgeNum = cube->getNumAtEdge();
    int dividor = edgeNum - 1;
    {
        std::vector<GLuint> temp(6 * (edgeNum - 1) * (edgeNum - 1));
        int currentPos = -1;
        for (int i = 0; i < edgeNum - 1; ++i) {
            for (int j = 0; j < edgeNum - 1; ++j) {
                unsigned int k1 = i * edgeNum + j;
                unsigned int k2 = (i + 1) * edgeNum + j;
                temp[++currentPos] = k2 + 1;
                temp[++currentPos] = k2;
                temp[++currentPos] = k1;
                temp[++currentPos] = k1 + 1;
                temp[++currentPos] = k2 + 1;
                temp[++currentPos] = k1;
            }
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO[0]);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, temp.size() * sizeof(GLuint), temp.data());
        currentPos = -1;
        for (int i = 0; i < edgeNum - 1; ++i) {
            for (int j = 0; j < edgeNum - 1; ++j) {
                unsigned int k1 = i * edgeNum + j;
                unsigned int k2 = (i + 1) * edgeNum + j;
                temp[++currentPos] = k1;
                temp[++currentPos] = k2;
                temp[++currentPos] = k2 + 1;
                temp[++currentPos] = k1;
                temp[++currentPos] = k2 + 1;
                temp[++currentPos] = k1 + 1;
            }
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO[1]);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, temp.size() * sizeof(GLuint), temp.data());
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
    for (int i = 0; i < cube->getSpringNum(); ++i) {
        std::vector<GLuint>* currentIndices = &structIndices;
        switch (cube->getSpring(i).getType()) {
            case simulation::Spring::SpringType::STRUCT:
                currentIndices = &structIndices;
                break;
            case simulation::Spring::SpringType::SHEAR:
                currentIndices = &shearIndices;
                break;
            case simulation::Spring::SpringType::BENDING:
                currentIndices = &bendingIndices;
                break;
        }
        currentIndices->emplace_back(cube->getSpring(i).getSpringStartID());
        currentIndices->emplace_back(cube->getSpring(i).getSpringEndID());
    }
    // Spring indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, structEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, structIndices.size() * sizeof(GLuint), structIndices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shearEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, shearIndices.size() * sizeof(GLuint), shearIndices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bendingEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, bendingIndices.size() * sizeof(GLuint), bendingIndices.data(),
                 GL_STATIC_DRAW);
}

void SoftCube::calculateTextureCoords() {
    int edgeNum = cube->getNumAtEdge();
    int dividor = edgeNum - 1;
    std::vector<GLfloat> temp(2 * edgeNum * edgeNum);
    int currentPos = -1;
    for (int i = 0; i < edgeNum; ++i) {
        for (int j = 0; j < edgeNum; ++j) {
            temp[++currentPos] = static_cast<GLfloat>(i) / dividor;
            temp[++currentPos] = static_cast<GLfloat>(j) / dividor;
        }
    }
    glBindBuffer(GL_ARRAY_BUFFER, textureCoordVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, temp.size() * sizeof(GLfloat), temp.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SoftCube::initializeBuffers() {
    for (int face = 0; face < 6; ++face) {
        glBindVertexArray(cubeVAO[face]);
        // Vertex
        glBindBuffer(GL_ARRAY_BUFFER, cubeVBO[face]);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
        // Normal
        glBindBuffer(GL_ARRAY_BUFFER, normalVBO[face]);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
        // Texture
        glBindBuffer(GL_ARRAY_BUFFER, textureCoordVBO);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), nullptr);
        // Index
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO[face & 1]);
    }

    // Spring - Struct
    glBindVertexArray(structVAO);
    glBindBuffer(GL_ARRAY_BUFFER, vertexVBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, structEBO);
    // Spring - Shear
    glBindVertexArray(shearVAO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shearEBO);
    // Spring - Bending
    glBindVertexArray(bendingVAO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bendingEBO);
    // Particles
    glBindVertexArray(particleVAO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
    // Unbind all
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void SoftCube::setTextures(const std::array<std::shared_ptr<Texture>, 6>& _textures) { textures = _textures; }

void SoftCube::update() {
    // All vertex
    int nParticle = cube->getParticleNum();
    auto particles = cube->getParticlePointer();
    int currentPos = -1;
    for (int i = 0; i < nParticle; ++i) {
        const auto& pos = (*particles)[i].getPosition();
        fullVertices[++currentPos] = pos[0];
        fullVertices[++currentPos] = pos[1];
        fullVertices[++currentPos] = pos[2];
    }
    glBindBuffer(GL_ARRAY_BUFFER, vertexVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, fullVertices.size() * sizeof(GLfloat), fullVertices.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    for (int face = 1; face <= 6; ++face) {
        updateSingleFace(face);
    }
}

void SoftCube::updateSingleFace(int face) {
    int edgeNum = cube->getNumAtEdge();
    // Vertex per face
    int currentPos = -1;
    for (int i = 0; i < edgeNum; ++i) {
        for (int j = 0; j < edgeNum; ++j) {
            int pos = 3 * cube->getPointMap(face, i, j);
            faceVertices[++currentPos] = fullVertices[pos++];
            faceVertices[++currentPos] = fullVertices[pos++];
            faceVertices[++currentPos] = fullVertices[pos];
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO[face - 1]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, faceVertices.size() * sizeof(GLfloat), faceVertices.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Reset to 0
    auto temp = std::vector<std::vector<Eigen::Vector3f>>(
        edgeNum, std::vector<Eigen::Vector3f>(edgeNum, Eigen::Vector3f::Zero()));
    // Use to flip normal
    float dFaceFactor = (face & 1) ? -1.0f : 1.0f;
    // Accumulate normals
    for (int i = 0; i < edgeNum - 1; ++i) {
        for (int j = 0; j < edgeNum - 1; ++j) {
            // V1   V4
            // V2   V3
            Eigen::Vector3f V1 = cube->getParticle(cube->getPointMap(face, i, j)).getPosition();
            Eigen::Vector3f V2 = cube->getParticle(cube->getPointMap(face, i + 1, j)).getPosition();
            Eigen::Vector3f V3 = cube->getParticle(cube->getPointMap(face, i + 1, j + 1)).getPosition();
            Eigen::Vector3f V4 = cube->getParticle(cube->getPointMap(face, i, j + 1)).getPosition();

            Eigen::Vector3f N = (V3 - V2).cross(V1 - V2);
            N *= dFaceFactor;
            N.normalize();
            temp[i][j] += N;
            temp[i + 1][j] += N;
            temp[i + 1][j + 1] += N;

            N = (V1 - V4).cross(V3 - V4);
            N *= dFaceFactor;
            N.normalize();
            temp[i][j] += N;
            temp[i + 1][j + 1] += N;
            temp[i][j + 1] += N;
        }
    }
    // Update normal buffer
    currentPos = -1;
    for (int i = 0; i < edgeNum; ++i) {
        for (int j = 0; j < edgeNum; ++j) {
            temp[i][j].normalize();
            normalBuffer[++currentPos] = temp[i][j][0];
            normalBuffer[++currentPos] = temp[i][j][1];
            normalBuffer[++currentPos] = temp[i][j][2];
        }
    }
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO[face - 1]);
    glBufferSubData(GL_ARRAY_BUFFER, 0, normalBuffer.size() * sizeof(GLfloat), normalBuffer.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SoftCube::renderCube(Program* shaderProgram) {
    shaderProgram->setUniform("model", modelMatrix);
    shaderProgram->setUniform("invtransmodel", inverseTransposeModel);
    shaderProgram->setUniform("useTexture", 1);
    int dividor = cube->getNumAtEdge() - 1;
    for (int face = 0; face < 6; ++face) {
        glBindVertexArray(cubeVAO[face]);
        shaderProgram->setUniform("diffuseTexture", textures[face]->getIndex());
        glDrawElements(GL_TRIANGLES, 6 * dividor * dividor, GL_UNSIGNED_INT, 0);
    }
    glBindVertexArray(0);
}

void SoftCube::renderLines(Program* shaderProgram, simulation::Spring::SpringType springType) {
    using SpringType = simulation::Spring::SpringType;
    shaderProgram->setUniform("useTexture", 0);
    shaderProgram->setUniform("model", modelMatrix);
    shaderProgram->setUniform("invtransmodel", inverseTransposeModel);
    std::size_t currentEBOSize = 0;
    switch (springType) {
        case SpringType::STRUCT:
            shaderProgram->setUniform("baseColor", Eigen::Vector3f(0.0f, 1.0f, 1.0f));
            glBindVertexArray(structVAO);
            currentEBOSize = structIndices.size();
            break;
        case SpringType::SHEAR:
            shaderProgram->setUniform("baseColor", Eigen::Vector3f(1.0f, 1.0f, 0.0f));
            glBindVertexArray(shearVAO);
            currentEBOSize = shearIndices.size();
            break;
        case SpringType::BENDING:
            shaderProgram->setUniform("baseColor", Eigen::Vector3f(0.0f, 1.0f, 0.0f));
            glBindVertexArray(bendingVAO);
            currentEBOSize = bendingIndices.size();
    }
    glDrawElements(GL_LINES, static_cast<GLsizei>(currentEBOSize), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void SoftCube::renderPoints(Program* shaderProgram) {
    shaderProgram->setUniform("model", modelMatrix);
    shaderProgram->setUniform("invtransmodel", inverseTransposeModel);
    shaderProgram->setUniform("useTexture", 0);
    shaderProgram->setUniform("baseColor", Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    glBindVertexArray(particleVAO);
    glDrawArrays(GL_POINTS, 0, cube->getParticleNum());
    glBindVertexArray(0);
}

SkyBox::SkyBox() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);
    glBindVertexArray(vao);
    // Vertex
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat), vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
    // Index
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);
    // Unbind all
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

SkyBox::~SkyBox() {
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);
}

void SkyBox::setTexture(const std::shared_ptr<CubeTexture>& _texture) { texture = _texture; }

void SkyBox::render(Program* shaderProgram) {
    glDepthFunc(GL_LEQUAL);  // Put on the background
    glBindVertexArray(vao);
    shaderProgram->setUniform("skybox", texture->getIndex());
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
    glDepthFunc(GL_LESS);
}
}  // namespace gfx
