#include "spring.h"

namespace simulation {
Spring::Spring(int springStartID, int springEndID, float restLength, float springCoef, float damperCoef,
               SpringType type)
    : firstSpringIndex(springStartID),
      secondSpringIndex(springEndID),
      restLength(restLength),
      springCoef(springCoef),
      damperCoef(damperCoef),
      type(type) {}

//==========================================
//  getter
//==========================================

int Spring::getSpringStartID() const { return firstSpringIndex; }
int Spring::getSpringEndID() const { return secondSpringIndex; }
float Spring::getSpringRestLength() const { return restLength; }
float Spring::getSpringCoef() const { return springCoef; }
float Spring::getDamperCoef() const { return damperCoef; }
Spring::SpringType Spring::getType() { return type; }

//==========================================
//  setter
//==========================================
void Spring::setSpringCoef(const float springCoef) { this->springCoef = springCoef; }

void Spring::setDamperCoef(const float damperCoef) { this->damperCoef = damperCoef; }
}  // namespace simulation
