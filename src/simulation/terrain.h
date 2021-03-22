#pragma once
#include <memory>

#include "Eigen/Dense"

#include "cube.h"

namespace simulation {
class Terrain;

enum class TerrainType : char { Plane, Sphere, Bowl, TiltedPlane };

class TerrainFactory final {
 public:
    // no instance, only static usage
    TerrainFactory() = delete;
    static std::unique_ptr<Terrain> CreateTerrain(TerrainType type);
};

// a virtual class
class Terrain {
 public:
    virtual ~Terrain() = default;
    Eigen::Matrix4f getModelMatrix();

    virtual TerrainType getType() = 0;
    virtual void handleCollision(const float delta_T, Cube& cube) = 0;

 protected:
    Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();
};

class PlaneTerrain final : public Terrain {
 public:
    friend class TerrainFactory;

    PlaneTerrain();

    TerrainType getType() override;
    void handleCollision(const float delta_T, Cube& cube) override;

 private:
    Eigen::Vector3f position = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
    Eigen::Vector3f normal = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
};

class SphereTerrain final : public Terrain {
 public:
    friend class TerrainFactory;

    SphereTerrain();

    TerrainType getType() override;
    void handleCollision(const float delta_T, Cube& cube) override;

 private:
    Eigen::Vector3f position = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
    float radius = 3.0f;
    float mass = 10.0f;
};

class BowlTerrain final : public Terrain {
 public:
    friend class TerrainFactory;

    BowlTerrain();

    TerrainType getType() override;
    void handleCollision(const float delta_T, Cube& cube) override;

 private:
    Eigen::Vector3f position = Eigen::Vector3f(2.0f, 7.0f, 1.0f);
    float radius = 7.0f;
    float mass = 10.0f;
};

class TiltedPlaneTerrain final : public Terrain {
 public:
    friend class TerrainFactory;

    TiltedPlaneTerrain();

    TerrainType getType() override;
    void handleCollision(const float delta_T, Cube& cube) override;

 private:
    Eigen::Vector3f position = Eigen::Vector3f(0.0, 0.0, 0.0);
    Eigen::Vector3f normal = Eigen::Vector3f(1.0, 1.0, 0.0);
};
}  // namespace simulation
