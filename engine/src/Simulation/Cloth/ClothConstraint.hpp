#pragma once

#include <vector>

#include "Simulation/Cloth/ClothParticle.hpp"

enum class ClothSolverMode {
    PBD,
    XPBD
};

class ClothConstraint {
public:
    virtual ~ClothConstraint() = default;
    virtual void ResetLambda() = 0;
    virtual void SolvePBD(
        std::vector<ClothParticle>& particles,
        float stiffness) = 0;
    virtual void SolveXPBD(
        std::vector<ClothParticle>& particles,
        float deltaTime) = 0;
};