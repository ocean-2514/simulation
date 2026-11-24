#pragma once

#include "Simulation/Cloth/Constraint/ClothConstraint.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

class GroundCollisionConstraint : public ClothConstraint {
public:
    GroundCollisionConstraint(uint32_t i, float y = 0.0f)
        : m_i(i), m_ground(y) {}

    void ResetLambda() override { m_lambda = 0.0f; }
    void SolvePBD(
        std::vector<ClothParticle>& particles,
        float) override {
        if (m_i >= particles.size()) return;
        auto& p = particles[m_i];
        if (!(p.invMass > 0.0f) || !std::isfinite(p.invMass)
            || !std::isfinite(p.position.y)
            || !std::isfinite(m_ground)) return;
        if (p.position.y < m_ground) p.position.y = m_ground;
    }
    void SolveXPBD(
        std::vector<ClothParticle>& particles,
        float) override {
        if (m_i >= particles.size()) return;
        auto& p = particles[m_i];
        if (!(p.invMass > 0.0f) || !std::isfinite(p.invMass)
            || !std::isfinite(p.position.y) || !std::isfinite(m_ground)) return;

        // C = y - ground >= 0 with an immovable ground (zero compliance).
        // Clamp the accumulated contact multiplier to keep it unilateral.
        const float constraintValue = p.position.y - m_ground;
        const float oldLambda = m_lambda;
        m_lambda = std::max(oldLambda - constraintValue / p.invMass, 0.0f);
        const float lambdaDelta = m_lambda - oldLambda;
        p.position.y += p.invMass * lambdaDelta;
    }

    void PostSolveVelocity(std::vector<ClothParticle>& particles) override {
        if (m_i >= particles.size()) return;
        auto& p = particles[m_i];
        if (!(p.invMass > 0.0f) || !std::isfinite(p.invMass)
            || !std::isfinite(p.position.y)
            || !std::isfinite(p.velocity.y) || !std::isfinite(m_ground)) return;

        constexpr float contactSlop = 1e-5f;
        if (p.position.y <= m_ground + contactSlop && p.velocity.y < 0.0f) {
            p.velocity.y = 0.0f;
        }
    }

private:
    uint32_t m_i;
    float m_ground = 0.0f;
    float m_lambda = 0.0f;
};
