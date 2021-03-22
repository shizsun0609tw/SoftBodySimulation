#pragma once
#include <vector>

#include "particle.h"
#include "spring.h"

namespace simulation {
class Cube {
 public:
    Cube();
    Cube(const Eigen::Vector3f &a_kInitPos, const float cubeLength, const int numAtEdge, const float dSpringCoef,
         const float dDamperCoef);
    //==========================================
    //  getter
    //==========================================

    int getParticleNum() const;  // return number of particles in the cube
    int getSpringNum() const;    // return number of springs in the cube
    int getNumAtEdge() const;    // return number of particles at edge
    // return index used to access particle at face
    unsigned int getPointMap(const int a_ciSide, const int a_ciI, const int a_ciJ);
    // get a particle in container according to index
    Particle &getParticle(int particleIdx);
    std::vector<Particle> *getParticlePointer();
    // get a spring in container according to index
    Spring &getSpring(int springIdx);

    //==========================================
    //  setter
    //==========================================

    void setSpringCoef(const float springCoef, const Spring::SpringType springType);
    void setDamperCoef(const float damperCoef, const Spring::SpringType springType);

    //==========================================
    //  method
    //==========================================
    // set rotation and offset of the cube
    void resetCube(const Eigen::Vector3f &offset, const float &rotate);
    // add gravity
    void addForceField(const Eigen::Vector3f &force);
    void computeInternalForce();
    // delegate collision detection to terrain

 private:
    float springCoefStruct;  // spring coefficient
    float springCoefShear;
    float springCoefBending;
    float damperCoefStruct;
    float damperCoefShear;
    float damperCoefBending;

    int particleNumPerEdge;  // number of particles at cube's edge
    int particleNumPerFace;  // number of particles at cube's face
    float cubeLength;
    Eigen::Vector3f initialPosition;

    std::vector<Particle> particles;
    std::vector<Spring> springs;

    //==========================================
    //  internal method
    //==========================================
    void initializeParticle();
    void initializeSpring();

    void updateSpringCoef(const float springCoef, const Spring::SpringType springType);
    void updateDamperCoef(const float damperCoef, const Spring::SpringType springType);

    Eigen::Vector3f computeSpringForce(const Eigen::Vector3f &positionA, const Eigen::Vector3f &positionB,
                                       const float springCoef, const float restLength);
    Eigen::Vector3f computeDamperForce(const Eigen::Vector3f &positionA, const Eigen::Vector3f &positionB,
                                       const Eigen::Vector3f &velocityA, const Eigen::Vector3f &velocityB,
                                       const float damperCoef);
};
}  // namespace simulation
