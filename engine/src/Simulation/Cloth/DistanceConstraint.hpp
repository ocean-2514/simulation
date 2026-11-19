#pragma once

#include "Simulation/Cloth/ClothConstraint.hpp"

class DistanceConstraint : public ClothConstraint {
public:
    DistanceConstraint(int i, int j, float restLength) 
        : m_i(i), m_j(j), m_restLength(restLength) {}
        
    void ResetLambda() override {}
    void SolvePBD(
        std::vector<ClothParticle>& particles,
        float stiffness) override;
    void SolveXPBD(
        std::vector<ClothParticle>& particles,
        float deltaTime) override {}

private:
    int m_i, m_j;
    float m_restLength = 0.0f;
};