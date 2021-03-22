#include "massSpringSystem.h"

#include <cmath>
#include <iostream>
#include <utility>

#include "integrator.h"
namespace simulation {
constexpr float g_cdDeltaT = 0.001f;
constexpr float g_cdK = 2000.0f;
constexpr float g_cdD = 60.0f;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor & Destructor
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MassSpringSystem::MassSpringSystem()
    : isDrawingCube(true),
      isDrawingStruct(false),
      isDrawingShear(false),
      isDrawingBending(false),
      isSimulating(false),

      integratorType(IntegratorType::ExplicitEuler),
      cubeCount(1),
      particleCountPerEdge(10),
      cubeID(1),

      deltaTime(g_cdDeltaT),
      springCoefStruct(g_cdK),
      springCoefShear(g_cdK),
      springCoefBending(g_cdK),
      damperCoefStruct(g_cdD),
      damperCoefShear(g_cdD),
      damperCoefBending(g_cdD),
      rotation(0.0),
      cubeLength(4.0),

      position(Eigen::Vector3f(0.0f, 5.0f, 0.0f)),
      gravity(Eigen::Vector3f(0.0f, -9.8f, 0.0f)) {
    initializeCube();
    reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set and Update
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MassSpringSystem::reset() {
    for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++) {
        cubes[cubeIdx].resetCube(position, rotation);
    }
}

void MassSpringSystem::setSpringCoef(const float springCoef, const Spring::SpringType springType) {
    for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++) {
        if (springType == Spring::SpringType::STRUCT) {
            springCoefStruct = springCoef;
            cubes[cubeIdx].setSpringCoef(springCoef, Spring::SpringType::STRUCT);
        } else if (springType == Spring::SpringType::SHEAR) {
            springCoefShear = springCoef;
            cubes[cubeIdx].setSpringCoef(springCoef, Spring::SpringType::SHEAR);
        } else if (springType == Spring::SpringType::BENDING) {
            springCoefBending = springCoef;
            cubes[cubeIdx].setSpringCoef(springCoef, Spring::SpringType::BENDING);
        } else {
            std::cout << "Error spring type in MassSpringSystem SetSpringCoef" << std::endl;
        }
    }
}

void MassSpringSystem::setDamperCoef(const float damperCoef, const Spring::SpringType springType) {
    for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++) {
        if (springType == Spring::SpringType::STRUCT) {
            damperCoefStruct = damperCoef;
            cubes[cubeIdx].setDamperCoef(damperCoef, Spring::SpringType::STRUCT);
        } else if (springType == Spring::SpringType::SHEAR) {
            damperCoefShear = damperCoef;
            cubes[cubeIdx].setDamperCoef(damperCoef, Spring::SpringType::SHEAR);
        } else if (springType == Spring::SpringType::BENDING) {
            damperCoefBending = damperCoef;
            cubes[cubeIdx].setDamperCoef(damperCoef, Spring::SpringType::BENDING);
        } else {
            std::cout << "Error spring type in CMassSpringSystme SetDamperCoef" << std::endl;
        }
    }
}

void MassSpringSystem::setTerrain(std::unique_ptr<Terrain>&& terrain) { this->terrain = std::move(terrain); }

void MassSpringSystem::setIntegrator(std::unique_ptr<Integrator>&& integrator) {
    this->integrator = std::move(integrator);
    this->integratorType = this->integrator->getType();
}

float MassSpringSystem::getSpringCoef(const Spring::SpringType springType) {
    if (springType == Spring::SpringType::STRUCT) {
        return springCoefStruct;
    } else if (springType == Spring::SpringType::SHEAR) {
        return springCoefShear;
    } else if (springType == Spring::SpringType::BENDING) {
        return springCoefBending;
    } else {
        std::cout << "Error spring type in CMassSpringSystme GetSpringCoef" << std::endl;
        throw std::invalid_argument("MassSpringSystem::GetSpringCoef : invalid springType");
    }
}
float MassSpringSystem::getDamperCoef(const Spring::SpringType springType) {
    if (springType == Spring::SpringType::STRUCT) {
        return damperCoefStruct;
    } else if (springType == Spring::SpringType::SHEAR) {
        return damperCoefShear;
    } else if (springType == Spring::SpringType::BENDING) {
        return damperCoefBending;
    } else {
        std::cout << "Error spring type in CMassSpringSystme GetDamperCoef" << std::endl;
        throw std::invalid_argument("MassSpringSystem::GetDamperCoef : invalid springType");
    }
}

int MassSpringSystem::getCubeCount() const { return cubeCount; }
Cube* MassSpringSystem::getCubePointer(int n) {
    if (n >= cubeCount) {
        throw std::runtime_error("Cube index out of range");
    }
    return &cubes[n];
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Simulation Part
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MassSpringSystem::checkStable() {
    for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++) {
        Cube testCube = cubes[cubeIdx];
        int springNum = testCube.getSpringNum();

        for (int springIdx = 0; springIdx < springNum; springIdx++) {
            Spring spring = testCube.getSpring(springIdx);
            float vel = testCube.getParticle(spring.getSpringStartID()).getVelocity().squaredNorm();

            if (std::isnan(vel) || vel > 1e6) return false;
        }
    }
    return true;
}

void MassSpringSystem::simulationOneTimeStep() {
    // std::cout << "deltaTime : " << this->deltaTime << std::endl;
    if (isSimulating) {
        integrate();
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MassSpringSystem::initializeCube() {
    for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++) {
        Cube NewCube(Eigen::Vector3f(0.0f, cubeLength, 0.0f - (2.0f * cubeLength * cubeIdx)), cubeLength,
                     particleCountPerEdge, springCoefStruct, damperCoefStruct);
        cubes.push_back(NewCube);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compute Force
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MassSpringSystem::computeAllForce() {
    for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++) {
        computeCubeForce(cubes[cubeIdx]);
    }
}

void MassSpringSystem::computeCubeForce(Cube& cube) {
    cube.addForceField(gravity);
    cube.computeInternalForce();
    // delegate to terrain to handle collision
    terrain->handleCollision(deltaTime, cube);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Integrator
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MassSpringSystem::integrate() {
    computeAllForce();
    integrator->integrate(*this);
}
}  // namespace simulation
