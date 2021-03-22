#include "particle.h"

namespace simulation {
//==========================================
//  getter
//==========================================

float Particle::getMass() const { return mass; }
Eigen::Vector3f Particle::getPosition() const { return position; }
Eigen::Vector3f Particle::getVelocity() const { return velocity; }
Eigen::Vector3f Particle::getAcceleration() const { return force / mass; }
Eigen::Vector3f Particle::getForce() const { return force; }

//==========================================
//  setter
//==========================================
void Particle::setMass(const float _mass) { mass = _mass; }

void Particle::setPosition(const Eigen::Vector3f &_position) { position = _position; }

void Particle::setVelocity(const Eigen::Vector3f &_velocity) { velocity = _velocity; }

void Particle::setAcceleration(const Eigen::Vector3f &_acceleration) { force = _acceleration * mass; }

void Particle::setForce(const Eigen::Vector3f &_force) { force = _force; }

//==========================================
//  method
//==========================================

void Particle::addPosition(const Eigen::Vector3f &_position) { position += _position; }

void Particle::addVelocity(const Eigen::Vector3f &_velocity) { velocity += _velocity; }

void Particle::addAcceleration(const Eigen::Vector3f &_acceleration) { force += _acceleration * mass; }

void Particle::addForce(const Eigen::Vector3f &_force) { force += _force; }
}  // namespace simulation
