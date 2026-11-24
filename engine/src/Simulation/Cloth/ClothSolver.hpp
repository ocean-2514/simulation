#pragma once

#include "Simulation/Cloth/Constraint/ClothConstraint.hpp"
#include "Simulation/Cloth/ClothGrid.hpp"
#include <vector>

struct ClothSolverSettings {
    ClothSolverMode mode = ClothSolverMode::PBD;
    uint32_t maxSubsteps = 4;
    uint32_t solverIterations = 8;
    glm::vec3 gravity{0.0f, -9.8f, 0.0f};
    float damping = 0.5f;
    float stiffness = 0.5f;
    float fixedDeltaTime = 1.0f / 60.0f;
};

class ClothSolver
{
public:
    ClothSolver(std::shared_ptr<ClothGrid> cloth, 
        ClothSolverSettings settings = {});
    
    void Step(float deltaTime);
    
private:
    void SimulateSubstep(float deltaTime);
    void BuildConstraints();

    std::shared_ptr<ClothGrid> m_cloth;
    std::vector<std::unique_ptr<ClothConstraint>> m_constraints;
    ClothSolverSettings m_settings;
    float m_accumulator = 0.0f;
};