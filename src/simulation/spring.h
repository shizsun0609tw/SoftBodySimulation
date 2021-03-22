#pragma once
#include "Eigen/Dense"

#include "particle.h"

namespace simulation {
class Spring {
 public:
    enum class SpringType : char {
        STRUCT,
        SHEAR,
        BENDING,
    };

 private:
    int firstSpringIndex;
    int secondSpringIndex;
    float restLength;
    float springCoef;
    float damperCoef;
    SpringType type;

 public:
    //==========================================
    //  constructor/destructor
    //==========================================

    Spring(int springStartID, int springEndID, float restLength, float springCoef, float damperCoef, SpringType type);
    //==========================================
    //  getter
    //==========================================

    int getSpringStartID() const;
    int getSpringEndID() const;
    float getSpringRestLength() const;
    float getSpringCoef() const;
    float getDamperCoef() const;
    SpringType getType();

    //==========================================
    //  setter
    //==========================================

    void setSpringCoef(const float springCoef);
    void setDamperCoef(const float damperCoef);
};
}  // namespace simulation
