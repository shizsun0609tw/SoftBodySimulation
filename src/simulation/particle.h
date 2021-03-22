#pragma once
#include "Eigen/Dense"

namespace simulation {
class Particle {
 private:
    float mass = 1.0f;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    Eigen::Vector3f force = Eigen::Vector3f::Zero();

 public:
    //==========================================
    //  getter
    //==========================================

    float getMass() const;
    Eigen::Vector3f getPosition() const;
    Eigen::Vector3f getVelocity() const;
    Eigen::Vector3f getAcceleration() const;
    Eigen::Vector3f getForce() const;

    //==========================================
    //  setter
    //==========================================

    void setMass(const float _mass);
    void setPosition(const Eigen::Vector3f &_position);
    void setVelocity(const Eigen::Vector3f &_velocity);
    void setAcceleration(const Eigen::Vector3f &_acceleration);
    void setForce(const Eigen::Vector3f &_force);

    //==========================================
    //  method
    //==========================================

    void addPosition(const Eigen::Vector3f &_position);
    void addVelocity(const Eigen::Vector3f &_velocity);
    void addAcceleration(const Eigen::Vector3f &_acceleration);
    void addForce(const Eigen::Vector3f &_force);
};
}  // namespace simulation
