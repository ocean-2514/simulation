#pragma once

#include "Simulation/Cloth/Constraint/ClothConstraint.hpp"

#include <cmath>
#include <cstdint>

class SphereCollisionConstraint : public ClothConstraint {
public:
    SphereCollisionConstraint(
        uint32_t idx,
        const glm::vec3& center = glm::vec3{0.0f},
        float radius = 1.0f)
        : m_i(idx), m_center(center), m_radius(radius) {}

    void ResetLambda() override {
        m_lambda = 0.0f;
    }

    void SolvePBD(
        std::vector<ClothParticle>& particles,
        float) override {
        if (m_i >= particles.size() || !HasValidSphere()) return;

        auto& particle = particles[m_i];
        if (!CanMove(particle)) return;

        const glm::vec3 offset = particle.position - m_center;
        const float distance = glm::length(offset);
        if (!std::isfinite(distance) || distance >= m_radius) return;

        const glm::vec3 normal = SafeNormal(offset, distance);
        particle.position = m_center + normal * (m_radius + kContactSlop);
    }

    void SolveXPBD(
        std::vector<ClothParticle>& particles,
        float) override {
        if (m_i >= particles.size() || !HasValidSphere()) return;

        auto& particle = particles[m_i];
        if (!CanMove(particle)) return;

        const glm::vec3 offset = particle.position - m_center;
        const float distance = glm::length(offset);
        if (!std::isfinite(distance)) return;

        // C = |x - center| - radius >= 0. The sphere is immovable and has
        // zero compliance, so the XPBD denominator is the particle weight.
        const glm::vec3 normal = SafeNormal(offset, distance);
        const float constraintValue = distance - m_radius - kContactSlop;
        const float oldLambda = m_lambda;
        const float candidateLambda = oldLambda - constraintValue / particle.invMass;
        if (!std::isfinite(candidateLambda)) return;

        m_lambda = candidateLambda > 0.0f ? candidateLambda : 0.0f;
        const float lambdaDelta = m_lambda - oldLambda;
        particle.position += particle.invMass * normal * lambdaDelta;
    }

    void PostSolveVelocity(
        std::vector<ClothParticle>& particles) override {
        if (m_i >= particles.size() || !HasValidSphere()) return;

        auto& particle = particles[m_i];
        if (!CanMove(particle)
            || !std::isfinite(particle.velocity.x)
            || !std::isfinite(particle.velocity.y)
            || !std::isfinite(particle.velocity.z)) return;

        const glm::vec3 offset = particle.position - m_center;
        const float distance = glm::length(offset);
        if (!std::isfinite(distance) || distance > m_radius + kContactSlop) return;

        const glm::vec3 normal = SafeNormal(offset, distance);
        const float inwardSpeed = glm::dot(particle.velocity, normal);
        if (inwardSpeed < 0.0f) {
            particle.velocity -= inwardSpeed * normal;
        }
    }

private:
    static constexpr float kDirectionEpsilon = 1e-6f;
    static constexpr float kContactSlop = 0.005f;

    bool HasValidSphere() const {
        return std::isfinite(m_center.x)
            && std::isfinite(m_center.y)
            && std::isfinite(m_center.z)
            && std::isfinite(m_radius)
            && m_radius >= 0.0f;
    }

    static bool CanMove(const ClothParticle& particle) {
        return std::isfinite(particle.invMass)
            && particle.invMass > 0.0f
            && std::isfinite(particle.position.x)
            && std::isfinite(particle.position.y)
            && std::isfinite(particle.position.z);
    }

    static glm::vec3 SafeNormal(const glm::vec3& offset, float distance) {
        return distance > kDirectionEpsilon
            ? offset / distance
            : glm::vec3{1.0f, 0.0f, 0.0f};
    }

    uint32_t m_i = 0;
    float m_lambda = 0.0f;
    glm::vec3 m_center{0.0f};
    float m_radius = 1.0f;
};
