#include "integrator.h"

#include <vector>
#include <iostream>

namespace simulation {
// Factory
std::unique_ptr<Integrator> IntegratorFactory::CreateIntegrator(IntegratorType type) {
    switch (type) {
        case simulation::IntegratorType::ExplicitEuler:
            return std::make_unique<ExplicitEulerIntegrator>();
        case simulation::IntegratorType::ImplicitEuler:
            return std::make_unique<ImplicitEulerIntegrator>();
        case simulation::IntegratorType::MidpointEuler:
            return std::make_unique<MidpointEulerIntegrator>();
        case simulation::IntegratorType::RungeKuttaFourth:
            return std::make_unique<RungeKuttaFourthIntegrator>();
        default:
            throw std::invalid_argument("TerrainFactory::CreateTerrain : invalid TerrainType");
            break;
    }
    return nullptr;
}

//
// ExplicitEulerIntegrator
//

IntegratorType ExplicitEulerIntegrator::getType() { return IntegratorType::ExplicitEuler; }

void ExplicitEulerIntegrator::integrate(MassSpringSystem& particleSystem) {
    std::vector<Particle> * particles = particleSystem.getCubePointer(0)->getParticlePointer();
    Eigen::Vector3f offset = Eigen::Vector3f::Zero();

    for (int i = 0; i < particles->size(); ++i) {
        (*particles)[i].addVelocity((*particles)[i].getAcceleration() * particleSystem.deltaTime);
        (*particles)[i].addPosition((*particles)[i].getVelocity() * particleSystem.deltaTime);
    }
}

//
// ImplicitEulerIntegrator
//

IntegratorType ImplicitEulerIntegrator::getType() { return IntegratorType::ImplicitEuler; }

void ImplicitEulerIntegrator::integrate(MassSpringSystem& particleSystem) {
    Cube next_cube = *particleSystem.getCubePointer(0);
    std::vector<Particle> * next_particles = next_cube.getParticlePointer();
    std::vector<Particle> * particles = particleSystem.getCubePointer(0)->getParticlePointer();
    particleSystem.computeCubeForce(next_cube);

    for (int i = 0; i < particles->size(); ++i) {
        (*particles)[i].addVelocity((*next_particles)[i].getAcceleration() * particleSystem.deltaTime);
        (*particles)[i].addPosition((*particles)[i].getVelocity() * particleSystem.deltaTime);
    }
}

//
// MidpointEulerIntegrator
//

IntegratorType MidpointEulerIntegrator::getType() { return IntegratorType::MidpointEuler; }

void MidpointEulerIntegrator::integrate(MassSpringSystem& particleSystem) {
    // TODO
    // For midpoint euler, the deltaTime passed in is correct.
    // But this deltaTime is for a full step.
    // So you may need to adjust it before computing, but don't forget to restore original value.

    Cube mid_cube = *particleSystem.getCubePointer(0);
    std::vector<Particle> * mid_particles = mid_cube.getParticlePointer(); 
    std::vector<Particle> * particles = particleSystem.getCubePointer(0)->getParticlePointer();
    particleSystem.deltaTime /= 2;
    particleSystem.computeCubeForce(mid_cube);
    particleSystem.deltaTime *= 2;

    for (int i = 0; i < particles->size(); ++i) {
        (*particles)[i].addVelocity((*mid_particles)[i].getAcceleration() * particleSystem.deltaTime);
        (*particles)[i].addPosition((*particles)[i].getVelocity() * particleSystem.deltaTime);
    }
}

//
// RungeKuttaFourthIntegrator
//

IntegratorType RungeKuttaFourthIntegrator::getType() { return IntegratorType::RungeKuttaFourth; }

void RungeKuttaFourthIntegrator::integrate(MassSpringSystem& particleSystem) {
    struct StateStep {
        Eigen::Vector3f deltaVel;
        Eigen::Vector3f deltaPos;
    };
    // TODO
    // StateStep struct is just a hint, you can use whatever you want.
    Cube temp_cube = *particleSystem.getCubePointer(0);
    std::vector<Particle> k1_particles, k2_particles, k3_particles, k4_particles;
    std::vector<Particle> * particles = particleSystem.getCubePointer(0)->getParticlePointer();
    float time = particleSystem.deltaTime;

    k1_particles = *temp_cube.getParticlePointer();
    for (int i = 0; i < particles->size(); ++i) {
        k1_particles[i].addVelocity((*particles)[i].getAcceleration() * time);
        k1_particles[i].addPosition(k1_particles[i].getVelocity() * time);
    }

    particleSystem.deltaTime /= 2;
    particleSystem.computeCubeForce(temp_cube);
    k2_particles = *temp_cube.getParticlePointer();    
    for (int i = 0; i < particles->size(); ++i) {
        k2_particles[i].addVelocity(k1_particles[i].getAcceleration() * time / 2);
        k2_particles[i].addPosition(k2_particles[i].getVelocity() * time / 2);
    }

    particleSystem.computeCubeForce(temp_cube);
    k3_particles = *temp_cube.getParticlePointer();
    for (int i = 0; i < particles->size(); ++i) {
        k3_particles[i].addVelocity(k2_particles[i].getAcceleration() * time / 2);
        k3_particles[i].addPosition(k3_particles[i].getVelocity() * time / 2);
    }
    
    particleSystem.deltaTime = time;
    particleSystem.computeCubeForce(temp_cube);
    k4_particles = *temp_cube.getParticlePointer();
    for (int i = 0; i < particles->size(); ++i) {
        k4_particles[i].addVelocity(k3_particles[i].getAcceleration() * time);
        k4_particles[i].addPosition(k4_particles[i].getVelocity() * time);
    }
    
    for (int i = 0; i < particles->size(); ++i) {
        (*particles)[i].addVelocity((1.0f / 6.0f) *
                                    (k1_particles[i].getAcceleration() + 2 * k2_particles[i].getAcceleration() +
                                        2 * k3_particles[i].getAcceleration() + k4_particles[i].getAcceleration()) *
                                    particleSystem.deltaTime);
        (*particles)[i].addPosition((*particles)[i].getVelocity() * particleSystem.deltaTime);
    }
}
}  // namespace simulation
