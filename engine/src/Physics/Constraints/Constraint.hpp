#pragma once

#include "Physics/RigidBodyUtils.hpp"

#include <glm/glm.hpp>

class PhysicsWorld;
class ContactCache;

struct SolverContext {
    float deltaTime;
    float inverseDeltaTime;
    glm::vec2 gravity;            // 用于推断休息速度等
    
    // 全局的摩擦系数、恢复系数等配置
    struct Settings {
        float baumgarte = 0.2f;   // 位置修正系数
        float slop = 0.005f;      // 允许的微小穿透容差
    } settings;

    // 可能需要访问世界以解决某些全局问题（如连续碰撞检测的 TOI）
    PhysicsWorld& world;
    ContactCache* contactCache = nullptr;
    uint32_t stepIndex = 0;
};

class Constraint {
public:
    virtual ~Constraint() = default;
    virtual void Prepare(SolverContext& context) = 0;
    virtual void WarmStart(SolverContext& context) = 0;
    virtual void SolveVelocity(SolverContext& context) = 0;
    virtual void SolvePosition(SolverContext& context) = 0;
};