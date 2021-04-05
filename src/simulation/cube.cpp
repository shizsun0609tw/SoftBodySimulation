#include "cube.h"

#include "Eigen/Dense"
#include <iostream>
#include "../util/helper.h"
namespace simulation {
constexpr float g_cdK = 2500.0f;
constexpr float g_cdD = 50.0f;

Cube::Cube()
    : particleNumPerEdge(10),
      cubeLength(2.0),
      initialPosition(Eigen::Vector3f(0.0, 0.0, 0.0)),
      springCoefStruct(g_cdK),
      springCoefShear(g_cdK),
      springCoefBending(g_cdK),
      damperCoefStruct(g_cdD),
      damperCoefShear(g_cdD),
      damperCoefBending(g_cdD) {
    particleNumPerFace = particleNumPerEdge * particleNumPerEdge;
    initializeParticle();
    initializeSpring();
}

Cube::Cube(const Eigen::Vector3f &a_kInitPos, const float cubeLength, const int numAtEdge, const float dSpringCoef,
           const float dDamperCoef)
    : particleNumPerEdge(numAtEdge),
      cubeLength(cubeLength),
      initialPosition(a_kInitPos),
      springCoefStruct(dSpringCoef),
      springCoefShear(dSpringCoef),
      springCoefBending(dSpringCoef),
      damperCoefStruct(dDamperCoef),
      damperCoefShear(dDamperCoef),
      damperCoefBending(dDamperCoef) {
    particleNumPerFace = numAtEdge * numAtEdge;
    initializeParticle();
    initializeSpring();
}

int Cube::getParticleNum() const { return static_cast<int>(particles.size()); }

int Cube::getSpringNum() const { return static_cast<int>(springs.size()); }

int Cube::getNumAtEdge() const { return particleNumPerEdge; }

unsigned int Cube::getPointMap(const int a_ciSide, const int a_ciI, const int a_ciJ) {
    int r = -1;

    switch (a_ciSide) {
        case 1:  // [a_ciI][a_ciJ][0] bottom face
            r = particleNumPerFace * a_ciI + particleNumPerEdge * a_ciJ;
            break;
        case 6:  // [a_ciI][a_ciJ][9] top face
            r = particleNumPerFace * a_ciI + particleNumPerEdge * a_ciJ + particleNumPerEdge - 1;
            break;
        case 2:  // [a_ciI][0][a_ciJ] front face
            r = particleNumPerFace * a_ciI + a_ciJ;
            break;
        case 5:  // [a_ciI][9][a_ciJ] back face
            r = particleNumPerFace * a_ciI + particleNumPerEdge * (particleNumPerEdge - 1) + a_ciJ;
            break;
        case 3:  // [0][a_ciI][a_ciJ] left face
            r = particleNumPerEdge * a_ciI + a_ciJ;
            break;
        case 4:  // [9][a_ciI][a_ciJ] ra_ciIght face
            r = particleNumPerFace * (particleNumPerEdge - 1) + particleNumPerEdge * a_ciI + a_ciJ;
            break;
    }

    return r;
}

Particle &Cube::getParticle(int particleIdx) { return particles[particleIdx]; }

std::vector<Particle> *Cube::getParticlePointer() { return &particles; }

Spring &Cube::getSpring(int springIdx) { return springs[springIdx]; }

void Cube::setSpringCoef(const float springCoef, const Spring::SpringType springType) {
    if (springType == Spring::SpringType::STRUCT) {
        springCoefStruct = springCoef;
        updateSpringCoef(springCoef, Spring::SpringType::STRUCT);
    } else if (springType == Spring::SpringType::SHEAR) {
        springCoefShear = springCoef;
        updateSpringCoef(springCoef, Spring::SpringType::SHEAR);
    } else if (springType == Spring::SpringType::BENDING) {
        springCoefBending = springCoef;
        updateSpringCoef(springCoef, Spring::SpringType::BENDING);
    }
}

void Cube::setDamperCoef(const float damperCoef, const Spring::SpringType springType) {
    if (springType == Spring::SpringType::STRUCT) {
        damperCoefStruct = damperCoef;
        updateDamperCoef(damperCoef, Spring::SpringType::STRUCT);
    } else if (springType == Spring::SpringType::SHEAR) {
        damperCoefShear = damperCoef;
        updateDamperCoef(damperCoef, Spring::SpringType::SHEAR);
    } else if (springType == Spring::SpringType::BENDING) {
        damperCoefBending = damperCoef;
        updateDamperCoef(damperCoef, Spring::SpringType::BENDING);
    }
}

void Cube::resetCube(const Eigen::Vector3f &offset, const float &rotate) {
    float dTheta = util::radians(rotate);  //  change angle from degree to
                                           //  radian

    for (unsigned int uiI = 0; uiI < particles.size(); uiI++) {
        int i = uiI / particleNumPerFace;
        int j = (uiI / particleNumPerEdge) % particleNumPerEdge;
        int k = uiI % particleNumPerEdge;
        float offset_x = (float)((i - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
        float offset_y = (float)((j - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
        float offset_z = (float)((k - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));

        Eigen::Vector3f RotateVec(offset_x, offset_y,
                                  offset_z);  //  vector from center of cube to the particle

        Eigen::AngleAxis<float> rotation(dTheta, Eigen::Vector3f(1.0f, 0.0f, 1.0f).normalized());

        RotateVec = rotation * RotateVec;

        particles[uiI].setPosition(initialPosition + offset + RotateVec);
        particles[uiI].setForce(Eigen::Vector3f::Zero());
        particles[uiI].setVelocity(Eigen::Vector3f::Zero());
    }
}

void Cube::addForceField(const Eigen::Vector3f &force) {
    for (unsigned int uiI = 0; uiI < particles.size(); uiI++) {
        particles[uiI].setAcceleration(force);
    }
}

void Cube::computeInternalForce() {
    Eigen::Vector3f spring_force_a = Eigen::Vector3f::Zero();
    Eigen::Vector3f spring_force_b = Eigen::Vector3f::Zero();
    Eigen::Vector3f damper_force_a = Eigen::Vector3f::Zero();
    Eigen::Vector3f damper_force_b = Eigen::Vector3f::Zero();
    int start_id = 0;
    int end_id = 0;

    for (int i = 0; i < springs.size(); i++) 
    {
        start_id = springs[i].getSpringStartID();
        end_id = springs[i].getSpringEndID();

        spring_force_a = computeSpringForce(particles[start_id].getPosition(), particles[end_id].getPosition(), 
                                          springs[i].getSpringCoef(), springs[i].getSpringRestLength());
        damper_force_a = computeDamperForce(particles[start_id].getPosition(), particles[end_id].getPosition(), 
                                          particles[start_id].getVelocity(), particles[end_id].getVelocity(),
                                          springs[i].getDamperCoef());        
        spring_force_b = computeSpringForce(particles[end_id].getPosition(), particles[start_id].getPosition(),
                                          springs[i].getSpringCoef(), springs[i].getSpringRestLength());
        damper_force_b = computeDamperForce(particles[end_id].getPosition(), particles[start_id].getPosition(),
                                          particles[end_id].getVelocity(), particles[start_id].getVelocity(),
                                          springs[i].getDamperCoef());

        particles[start_id].addForce(spring_force_a + damper_force_a);
        particles[end_id].addForce(spring_force_b + damper_force_b);
    }
}

Eigen::Vector3f Cube::computeSpringForce(const Eigen::Vector3f &positionA, const Eigen::Vector3f &positionB,
                                         const float springCoef, const float restLength) {
    Eigen::Vector3f f = Eigen::Vector3f::Zero();

    f = -springCoef * 
        (fabs((positionA - positionB).norm()) - restLength) *
        ((positionA - positionB) / (fabs((positionA - positionB).norm())));

    return f;
}

Eigen::Vector3f Cube::computeDamperForce(const Eigen::Vector3f &positionA, const Eigen::Vector3f &positionB,
                                         const Eigen::Vector3f &velocityA, const Eigen::Vector3f &velocityB,
                                         const float damperCoef) {
    Eigen::Vector3f f = Eigen::Vector3f::Zero();

    f = -damperCoef * 
        (((velocityA - velocityB).dot(positionA - positionB)) / (fabs((positionA - positionB).norm()))) * 
        ((positionA - positionB) / (fabs((positionA - positionB).norm())));

    return f;
}

void Cube::initializeParticle() {
    for (int i = 0; i < particleNumPerEdge; i++) {
        for (int j = 0; j < particleNumPerEdge; j++) {
            for (int k = 0; k < particleNumPerEdge; k++) {
                Particle Particle;
                float offset_x = (float)((i - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
                float offset_y = (float)((j - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
                float offset_z = (float)((k - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
                Particle.setPosition(Eigen::Vector3f(initialPosition(0) + offset_x, initialPosition(1) + offset_y,
                                                     initialPosition(2) + offset_z));
                particles.push_back(Particle);
            }
        }
    }
}

void Cube::pushSpringLine(const int particleID, const int neighborID, Spring::SpringType springType) 
{
    float length = fabs((particles[particleID].getPosition() - particles[neighborID].getPosition()).norm());
    
    switch (springType) {
        case Spring::SpringType::STRUCT:
            springs.push_back(Spring(particleID, neighborID, length, springCoefStruct, damperCoefStruct,
                                     Spring::SpringType::STRUCT));
            break;
        case Spring::SpringType::BENDING:
            springs.push_back(Spring(particleID, neighborID, length, springCoefBending, damperCoefBending,
                                     Spring::SpringType::BENDING));
            break;
        case Spring::SpringType::SHEAR:
            springs.push_back(Spring(particleID, neighborID, length, springCoefShear, damperCoefShear,
                                     Spring::SpringType::SHEAR));
            break;
        default:
            break;
    }
}

void Cube::initalizeSpringBending()
{
    int iParticleID = 0;
    int iNeighborID = 0;
    Eigen::Vector3f Length = Eigen::Vector3f::Zero();
    
    for (int i = 0; i < particleNumPerEdge; i ++) {
        for (int j = 0; j < particleNumPerEdge; j ++) {
            for (int k = 0; k < particleNumPerEdge; k ++) {
                iParticleID = i * particleNumPerFace + j * particleNumPerEdge + k;
                // front
                if (k < particleNumPerEdge - 2) {
                
                    iNeighborID = iParticleID + 2;
                    pushSpringLine(iParticleID, iNeighborID, Spring::SpringType::BENDING);
                }
                
                // top
                if (j < particleNumPerEdge - 2) {
                    iNeighborID = iParticleID + particleNumPerEdge * 2;
                    pushSpringLine(iParticleID, iNeighborID, Spring::SpringType::BENDING);
                }
                // right
                if (i < particleNumPerEdge - 2) {
                    iNeighborID = iParticleID + particleNumPerFace * 2;
                    pushSpringLine(iParticleID, iNeighborID, Spring::SpringType::BENDING);
                }
            }
        }
    }
}

void Cube::initalizeSpringStruct() 
{
    int iParticleID = 0;
    int iNeighborID = 0;
    Eigen::Vector3f Length = Eigen::Vector3f::Zero();

    for (int i = 0; i < particleNumPerEdge; ++i) {
        for (int j = 0; j < particleNumPerEdge; ++j) {
            for (int k = 0; k < particleNumPerEdge; ++k) {
                iParticleID = i * particleNumPerFace + j * particleNumPerEdge + k;
                // front
                if (k != particleNumPerEdge - 1) {
                    iNeighborID = iParticleID + 1;
                    pushSpringLine(iParticleID, iNeighborID, Spring::SpringType::STRUCT);
                }
                // top
                if (j != particleNumPerEdge - 1) {
                    iNeighborID = iParticleID + particleNumPerEdge;
                    pushSpringLine(iParticleID, iNeighborID, Spring::SpringType::STRUCT);
                }
                // right
                if (i != particleNumPerEdge - 1) {
                    iNeighborID = iParticleID + particleNumPerFace;
                    pushSpringLine(iParticleID, iNeighborID, Spring::SpringType::STRUCT);
                }
            }
        }
    }
}

void Cube::initializeSpringShear()
{
    int iParticleID = 0;
    int iNeighborID = 0;
    Eigen::Vector3f Length = Eigen::Vector3f::Zero();

    for (int i = 0; i < particleNumPerEdge; ++i) {
        for (int j = 0; j < particleNumPerEdge; ++j) {
            for (int k = 0; k < particleNumPerEdge; ++k) {
                iParticleID = i * particleNumPerFace + j * particleNumPerEdge + k;

                initializeSpringShearLine(iParticleID, i, j, k);
            }
        }
    }
}

void Cube::initializeSpringShearLine(const int particleID, const int i, const int j, const int k) {
    /*
    
      4--------5
     /|       /|            j
    6--------7-|            |
    | 0------|-1            . - i
    |/       |/           /
    2--------3          k
    
    */

    int iNeighborID = 0;
    Eigen::Vector3f Length = Eigen::Vector3f::Zero();
    
    //  0-6
    if (k != particleNumPerEdge - 1 && j != particleNumPerEdge - 1) {
        iNeighborID = particleID + particleNumPerEdge + 1;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }

    //  0-7
    if (k != particleNumPerEdge -1 && j != particleNumPerEdge - 1 && i != particleNumPerEdge - 1) {
        iNeighborID = particleID + particleNumPerFace + particleNumPerEdge + 1;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }

    // 0-5
    if (j != particleNumPerEdge - 1 && i != particleNumPerEdge - 1)
    {
        iNeighborID = particleID + particleNumPerEdge + particleNumPerFace;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }

    // 0-3
    if (k != particleNumPerEdge - 1 && i != particleNumPerEdge - 1)
    {
        iNeighborID = particleID + particleNumPerFace + 1;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }

    // 2-4
    if (k != 0 && j != particleNumPerEdge - 1)
    {
        iNeighborID = particleID + particleNumPerEdge - 1;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }

    // 2-1
    if (k != 0 && i != particleNumPerEdge - 1)
    {
        iNeighborID = particleID + particleNumPerFace - 1;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }

    // 2-5
    if (k != 0 && j != particleNumPerEdge - 1 && i != particleNumPerEdge - 1)
    {
        iNeighborID = particleID + particleNumPerEdge + particleNumPerFace - 1;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }
    
    // 1-6
    if (k != particleNumPerEdge - 1 && j != particleNumPerEdge - 1 && i != 0) 
    {
        iNeighborID = particleID + particleNumPerEdge - particleNumPerFace + 1;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }
    
    // 4-3
    if (k != particleNumPerEdge - 1 && j != 0 && i != particleNumPerEdge - 1)
    {
        iNeighborID = particleID - particleNumPerEdge + particleNumPerFace + 1;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }
    
    // 4-1
    if (j != 0 && i != particleNumPerEdge - 1) 
    {
        iNeighborID = particleID - particleNumPerEdge + particleNumPerFace;
        pushSpringLine(particleID, iNeighborID, Spring::SpringType::SHEAR);
    }
}

void Cube::initializeSpring() {
    int iParticleID = 0;
    int iNeighborID = 0;
    Eigen::Vector3f SpringStartPos = particles[iParticleID].getPosition();
    Eigen::Vector3f SpringEndPos = particles[iNeighborID].getPosition();
    Eigen::Vector3f Length = SpringStartPos - SpringEndPos;

    enum {
        X_id = 1,
        Y_id = 2,
        Z_id = 3
    };
    
    initalizeSpringBending();
    initalizeSpringStruct();
    initializeSpringShear();
}

void Cube::updateSpringCoef(const float a_cdSpringCoef, const Spring::SpringType a_cSpringType) {
    for (unsigned int uiI = 0; uiI < springs.size(); uiI++) {
        if (springs[uiI].getType() == a_cSpringType) {
            springs[uiI].setSpringCoef(a_cdSpringCoef);
        }
    }
}

void Cube::updateDamperCoef(const float a_cdDamperCoef, const Spring::SpringType a_cSpringType) {
    for (unsigned int uiI = 0; uiI < springs.size(); uiI++) {
        if (springs[uiI].getType() == a_cSpringType) {
            springs[uiI].setDamperCoef(a_cdDamperCoef);
        }
    }
}
}  //  namespace simulation
