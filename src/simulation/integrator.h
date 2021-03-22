#pragma once
#include <memory>

#include "massSpringSystem.h"

namespace simulation {
class Integrator;

enum class IntegratorType : char { ExplicitEuler, ImplicitEuler, MidpointEuler, RungeKuttaFourth };

class IntegratorFactory final {
 public:
    // no instance, only static usage
    IntegratorFactory() = delete;

    static std::unique_ptr<Integrator> CreateIntegrator(IntegratorType type);
};

class Integrator {
 public:
    virtual ~Integrator() = default;
    virtual IntegratorType getType() = 0;
    virtual void integrate(MassSpringSystem& particleSystem) = 0;
};

class ExplicitEulerIntegrator final : public Integrator {
 public:
    ExplicitEulerIntegrator() = default;
    IntegratorType getType() override;
    void integrate(MassSpringSystem& particleSystem) override;
};

class ImplicitEulerIntegrator final : public Integrator {
 public:
    ImplicitEulerIntegrator() = default;
    IntegratorType getType() override;
    void integrate(MassSpringSystem& particleSystem) override;
};

class MidpointEulerIntegrator final : public Integrator {
 public:
    MidpointEulerIntegrator() = default;
    IntegratorType getType() override;
    void integrate(MassSpringSystem& particleSystem) override;
};

class RungeKuttaFourthIntegrator final : public Integrator {
 public:
    RungeKuttaFourthIntegrator() = default;
    IntegratorType getType() override;
    void integrate(MassSpringSystem& particleSystem) override;
};
}  // namespace simulation
