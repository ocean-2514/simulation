#pragma once

#include "Simulation/Cloth/Constraint/ClothConstraint.hpp"

#include <cstdint>

class DistanceConstraint : public ClothConstraint {
public:
    DistanceConstraint(uint32_t i, uint32_t j, float compliance, float restLength) 
        : m_i(i), m_j(j), m_compliance(compliance), m_restLength(restLength) {}
        
    void ResetLambda() override { m_lambda = 0.0f; }
    void SolvePBD(
        std::vector<ClothParticle>& particles,
        float stiffness) override;
    void SolveXPBD(
        std::vector<ClothParticle>& particles,
        float deltaTime) override;

private:
    uint32_t m_i, m_j;
    float m_restLength = 0.0f;
    float m_lambda = 0.0f;
    float m_compliance = 0.0001f;
};