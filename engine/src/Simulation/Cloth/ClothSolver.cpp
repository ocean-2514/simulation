#include "Simulation/Cloth/ClothSolver.hpp"
#include "Simulation/Cloth/Constraint/DistanceConstraint.hpp"
#include "Simulation/Cloth/Constraint/GroundCollisionConstraint.hpp"
#include "Simulation/Cloth/Constraint/SphereCollisionConstraint.hpp"

#include <cmath>

ClothSolver::ClothSolver(
    std::shared_ptr<ClothGrid> cloth, 
    ClothSolverSettings settings)
    : m_cloth(std::move(cloth)), m_settings(settings) {
    BuildConstraints();
}

void ClothSolver::Step(float deltaTime) {
    m_accumulator += deltaTime;
    uint32_t stepCount = 0;
    const float fixedDeltaTime = m_settings.fixedDeltaTime;
    while (m_accumulator >= fixedDeltaTime 
        && stepCount < m_settings.maxSubsteps) {
        SimulateSubstep(fixedDeltaTime);
        m_accumulator -= fixedDeltaTime;
        ++stepCount;
    }
    if (m_accumulator > fixedDeltaTime) {
        m_accumulator = std::fmod(m_accumulator, fixedDeltaTime);
    }
}

void ClothSolver::SimulateSubstep(float dt) {
    auto& particles = m_cloth->Particles();
    for (auto& particle : particles) {
        particle.oldPosition = particle.position;
        if (particle.invMass <= 0.0f) continue;
        particle.velocity *= std::exp(-m_settings.damping * dt);
        glm::vec3 accumulation = m_settings.gravity + 
            particle.force * particle.invMass;
        particle.velocity += accumulation * dt;
        particle.position += particle.velocity * dt;
    }

    if (m_settings.mode == ClothSolverMode::XPBD) {
        // XPBD multipliers accumulate across iterations within this substep,
        // but start fresh at the next substep.
        for (auto& constraint : m_constraints) {
            constraint->ResetLambda();
        }
    }

    for (uint32_t i = 0; i < m_settings.solverIterations; ++i) {
        for (auto& constraint : m_constraints) {
            if (m_settings.mode == ClothSolverMode::PBD) {
                constraint->SolvePBD(particles, m_settings.stiffness);
            } else if (m_settings.mode == ClothSolverMode::XPBD) {
                constraint->SolveXPBD(particles, dt);
            }
        }
    }

    for (auto& particle : particles) {
        particle.velocity = (particle.position - particle.oldPosition) / dt;
        particle.force = glm::vec3{0.0f};
    }
    for (auto& constraint : m_constraints) {
        constraint->PostSolveVelocity(particles);
    }
}

void ClothSolver::BuildConstraints()
{
    m_constraints.clear();

    const uint32_t cols = m_cloth->Columns();
    const uint32_t rows = m_cloth->Rows();

    const float spacingX = m_cloth->SpacingX();
    const float spacingY = m_cloth->SpacingY();
    const float diagLen  = std::sqrt(spacingX * spacingX + spacingY * spacingY);

    m_constraints.reserve(cols * rows * 6);

    auto addDistance = [&](uint32_t a, uint32_t b, float alpha, float rest) {
        auto c = std::make_unique<DistanceConstraint>(a, b, alpha, rest);
        m_constraints.push_back(std::move(c));
    };

    for (uint32_t y = 0; y < rows; ++y) {
        for (uint32_t x = 0; x < cols; ++x) {
            const uint32_t idx = y * cols + x;

            if (x + 1 < cols)
                addDistance(idx, idx + 1, 0.0001f, spacingX);
            if (y + 1 < rows)
                addDistance(idx, idx + cols, 0.0001f, spacingY);
            if (x + 1 < cols && y + 1 < rows) {
                addDistance(idx, idx + cols + 1, 0.0001f, diagLen); // ↘
                addDistance(idx + 1, idx + cols, 0.0001f, diagLen); // ↙
            }

            if (x + 2 < cols)
                addDistance(idx, idx + 2, 0.001f, spacingX * 2.0f);
            if (y + 2 < rows)
                addDistance(idx, idx + 2 * cols, 0.001f, spacingY * 2.0f);
        }
    }

    // Solve the unilateral ground constraints after all cloth constraints in
    // each iteration so later distance corrections cannot reintroduce penetration.
    const uint32_t particleCount = cols * rows;
    for (uint32_t idx = 0; idx < particleCount; ++idx) {
        m_constraints.push_back(
            std::make_unique<GroundCollisionConstraint>(idx, -1.0f));
        m_constraints.push_back(
            std::make_unique<SphereCollisionConstraint>(idx, glm::vec3{0.0f, -0.1f, -0.5f}, 0.3f)
        );
    }
}
