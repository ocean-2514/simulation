#include "Simulation/Cloth/Constraint/DistanceConstraint.hpp"

#include <cmath>

void DistanceConstraint::SolvePBD(
    std::vector<ClothParticle>& particles,
    float stiffness) {
    if (m_i >= particles.size() || m_j >= particles.size()) return;

    auto& p0 = particles[m_i];
    auto& p1 = particles[m_j];

    if (p0.invMass <= 0.0f && p1.invMass <= 0.0f) return;

    const glm::vec3 dist = p1.position - p0.position;
    const float length = glm::length(dist);
    const glm::vec3 distNorm = 
        length <= 1e-6 ? glm::vec3{1.0f, 0.0f, 0.0f} : 
        dist / length;
    const float error = length - m_restLength;
    
    const float w0 = p0.invMass / (p0.invMass + p1.invMass);
    const float w1 = p1.invMass / (p0.invMass + p1.invMass);
    p0.position += w0 * error * stiffness * distNorm;
    p1.position -= w1 * error * stiffness * distNorm;
}

void DistanceConstraint::SolveXPBD(
    std::vector<ClothParticle>& particles,
    float deltaTime) {
    if (m_i >= particles.size() || m_j >= particles.size()) return;
    if (!(deltaTime > 0.0f) || !std::isfinite(deltaTime)) return;
    if (m_compliance < 0.0f || !std::isfinite(m_compliance)
        || m_restLength < 0.0f || !std::isfinite(m_restLength)) return;

    auto& p0 = particles[m_i];
    auto& p1 = particles[m_j];
    if (p0.invMass < 0.0f || p1.invMass < 0.0f) return;
    if (p0.invMass == 0.0f && p1.invMass == 0.0f) return;

    const glm::vec3 dist = p1.position - p0.position;
    const float length = glm::length(dist);
    if (!std::isfinite(length)) return;
    const glm::vec3 distNorm = length > 1e-6f
        ? dist / length
        : glm::vec3{1.0f, 0.0f, 0.0f};

    const float error = length - m_restLength;
    const float alphaTilde = m_compliance / (deltaTime * deltaTime);
    const float denominator = p0.invMass + p1.invMass + alphaTilde;
    if (!(denominator > 0.0f) || !std::isfinite(denominator)) return;

    const float lambdaDelta = -(error + alphaTilde * m_lambda) 
        / denominator;
    if (!std::isfinite(lambdaDelta)) return;
    m_lambda += lambdaDelta;

    // For C = |p1 - p0| - restLength, the gradients are -n for p0 and +n for p1.
    p0.position -= p0.invMass * lambdaDelta * distNorm;
    p1.position += p1.invMass * lambdaDelta * distNorm;
}
