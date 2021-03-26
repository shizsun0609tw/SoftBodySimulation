#include "terrain.h"

#include <stdexcept>
#include <iostream>
#include <stdlib.h>

#include "particle.h"
#include "../util/helper.h"

namespace simulation {
// Factory
std::unique_ptr<Terrain> TerrainFactory::CreateTerrain(TerrainType type) {
    switch (type) {
        case simulation::TerrainType::Plane:
            return std::make_unique<PlaneTerrain>();
        case simulation::TerrainType::Sphere:
            return std::make_unique<SphereTerrain>();
        case simulation::TerrainType::Bowl:
            return std::make_unique<BowlTerrain>();
        case simulation::TerrainType::TiltedPlane:
            return std::make_unique<TiltedPlaneTerrain>();
        default:
            throw std::invalid_argument("TerrainFactory::CreateTerrain : invalid TerrainType");
            break;
    }
    return nullptr;
}
// Terrain

Eigen::Matrix4f Terrain::getModelMatrix() { return modelMatrix; }

// Note:
// You should update each particles' velocity (base on the equation in
// slide) and force (contact force : resist + friction) in handleCollision function

// PlaneTerrain //

PlaneTerrain::PlaneTerrain() { modelMatrix = util::translate(0.0f, position[1], 0.0f) * util::scale(60, 1, 60); }

TerrainType PlaneTerrain::getType() { return TerrainType::Plane; }

void PlaneTerrain::handleCollision(const float delta_T, Cube& cube) {
    constexpr float eEPSILON = 0.01f;
    constexpr float coefResist = 0.8f;
    constexpr float coefFriction = 0.7f;
   
    std::vector<simulation::Particle> * particles = cube.getParticlePointer();
    for (int i = 0; i < particles->size(); ++i) {
        if (fabs(this->normal.dot(this->position - (*particles)[i].getPosition())) < eEPSILON
            && this->normal.dot((*particles)[i].getVelocity()) < 0)
        {
            Eigen::Vector3f vn = (*particles)[i].getVelocity().dot(this->normal) / (this->normal).norm() * this->normal;
            Eigen::Vector3f vt = (*particles)[i].getVelocity() - vn;

            (*particles)[i].setPosition((*particles)[i].getPosition() + this->normal * eEPSILON);
            (*particles)[i].setVelocity(-coefResist * vn + vt);

            Eigen::Vector3f fc = Eigen::Vector3f::Zero();
            Eigen::Vector3f ff = Eigen::Vector3f::Zero();

            if (this->normal.dot((*particles)[i].getForce()) < 0) 
            {
                fc = -(this->normal.dot((*particles)[i].getForce())) * this->normal;
                ff = -coefFriction * (-this->normal.dot((*particles)[i].getForce())) * vt;
            }

            (*particles)[i].addForce(fc + ff);
        }
    }
}

// SphereTerrain //

SphereTerrain::SphereTerrain() { modelMatrix = util::translate(position) * util::scale(radius, radius, radius); }

TerrainType SphereTerrain::getType() { return TerrainType::Sphere; }

void SphereTerrain::handleCollision(const float delta_T, Cube& cube) {
    constexpr float eEPSILON = 0.01f;
    constexpr float coefResist = 0.8f;
    constexpr float coefFriction = 0.3f;
    
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    
    std::vector<simulation::Particle>* particles = cube.getParticlePointer();
    for (int i = 0; i < particles->size(); ++i) {
        normal = ((*particles)[i].getPosition() - this->position).normalized();
        if (fabs(normal.dot(this->position - (*particles)[i].getPosition())) < eEPSILON + radius &&
            normal.dot((*particles)[i].getVelocity()) < 0) {
            Eigen::Vector3f vn = (*particles)[i].getVelocity().dot(normal) / (normal).norm() * normal;
            Eigen::Vector3f vt = (*particles)[i].getVelocity() - vn;

            (*particles)[i].setPosition((*particles)[i].getPosition() + normal * eEPSILON);
            (*particles)[i].setVelocity(-coefResist * vn + vt);

            Eigen::Vector3f fc = Eigen::Vector3f::Zero();
            Eigen::Vector3f ff = Eigen::Vector3f::Zero();

            if (normal.dot((*particles)[i].getForce()) < 0) {
                fc = -(normal.dot((*particles)[i].getForce())) * normal;
                ff = -coefFriction * (-normal.dot((*particles)[i].getForce())) * vt;
            }

            (*particles)[i].addForce(fc + ff);
        }
    }
}

// BowlTerrain //

BowlTerrain::BowlTerrain() { modelMatrix = util::translate(position) * util::scale(radius, radius, radius); }

TerrainType BowlTerrain::getType() { return TerrainType::Bowl; }

void BowlTerrain::handleCollision(const float delta_T, Cube& cube) {
    constexpr float eEPSILON = 0.01f;
    constexpr float coefResist = 0.8f;
    constexpr float coefFriction = 0.3f;
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();

    std::vector<simulation::Particle>* particles = cube.getParticlePointer();
    for (int i = 0; i < particles->size(); ++i) {
        normal = (this->position - (*particles)[i].getPosition()).normalized();
        if (fabs(normal.dot(this->position - (*particles)[i].getPosition())) > radius + eEPSILON &&
            normal.dot((*particles)[i].getVelocity()) < 0) {
            Eigen::Vector3f vn = (*particles)[i].getVelocity().dot(normal) / (normal).norm() * normal;
            Eigen::Vector3f vt = (*particles)[i].getVelocity() - vn;

            (*particles)[i].setPosition((*particles)[i].getPosition() + normal * eEPSILON);
            (*particles)[i].setVelocity(-coefResist * vn + vt);

            Eigen::Vector3f fc = Eigen::Vector3f::Zero();
            Eigen::Vector3f ff = Eigen::Vector3f::Zero();

            if (normal.dot((*particles)[i].getForce()) < 0) {
                fc = -(normal.dot((*particles)[i].getForce())) * normal;
                ff = -coefFriction * (-normal.dot((*particles)[i].getForce())) * vt;
            }

            (*particles)[i].addForce(fc + ff);
        }
    }
}

// TiltedPlaneTerrain //

TiltedPlaneTerrain::TiltedPlaneTerrain() { modelMatrix = util::rotateDegree(0, 0, -45) * util::scale(60, 1, 60); }

TerrainType TiltedPlaneTerrain::getType() { return TerrainType::TiltedPlane; }

void TiltedPlaneTerrain::handleCollision(const float delta_T, Cube& cube) {
    constexpr float eEPSILON = 0.01f;
    constexpr float coefResist = 0.8f;
    constexpr float coefFriction = 0.3f;
   
    std::vector<simulation::Particle>* particles = cube.getParticlePointer();
    for (int i = 0; i < particles->size(); ++i) {
        if (fabs(this->normal.dot(this->position - (*particles)[i].getPosition())) < eEPSILON &&
            this->normal.dot((*particles)[i].getVelocity()) < 0) {
            Eigen::Vector3f vn = (*particles)[i].getVelocity().dot(this->normal) / (this->normal).norm() * this->normal;
            Eigen::Vector3f vt = (*particles)[i].getVelocity() - vn;

            (*particles)[i].setPosition((*particles)[i].getPosition() + this->normal * eEPSILON * 0.1);
            (*particles)[i].setVelocity(-coefResist * vn + vt);

            Eigen::Vector3f fc = Eigen::Vector3f::Zero();
            Eigen::Vector3f ff = Eigen::Vector3f::Zero();

            if (this->normal.dot((*particles)[i].getForce()) < 0) {
                fc = -(this->normal.dot((*particles)[i].getForce())) * this->normal;
                ff = -coefFriction * (-this->normal.dot((*particles)[i].getForce())) * vt;
            }

            (*particles)[i].addForce(fc + ff);
        }
    }
}
}  // namespace simulation
