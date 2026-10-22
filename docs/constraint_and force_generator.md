## 15. 约束与力生成器

约束系统应与碰撞检测分开设计。碰撞接触本身也是一种约束，但它由窄相检测动态产生；弹簧、距离关节和固定关节则由用户创建并跨时间步持续存在。

先区分两种不同机制：

- **力生成器（ForceGenerator）**：向刚体累加力或力矩，例如重力、持续推力和软弹簧。它参与外力积分，不直接修正位置或速度。
- **约束（Constraint / Joint）**：限制一个或多个自由度，例如保持两点距离或固定相对位置和角度。它由约束求解器迭代求解，通常通过冲量改变速度，再通过位置修正消除误差。

一个“弹簧”可以采用上述任一模型。软弹簧力实现简单，但刚度较大时显式积分容易振荡甚至发散；距离弹簧约束通过迭代冲量处理，通常更稳定，也更容易与碰撞一起求解。不要把 `ForceGenerator` 和 `Constraint` 合成一个职责含糊的基类。

### 15.1 建议的模块边界

建议在 `engine/src/Physics/` 下增加：

```text
Physics/
    Forces/
        ForceGenerator.hpp
        SpringForce.hpp
    Constraints/
        Constraint.hpp
        DistanceConstraint.hpp
        FixedConstraint.hpp
        ConstraintSolver.hpp
```

职责划分：

| 类型 | 职责 |
|---|---|
| `PhysicsWorld` | 持有刚体、力生成器和约束；协调每个物理子步的执行顺序 |
| `ForceGenerator` | 根据当前状态向刚体累加力/力矩，不直接改速度或位置 |
| `Constraint` | 保存 body ID、局部锚点和约束参数；提供准备与求解接口 |
| `ConstraintSolver` | 管理 warm start 和多轮迭代，统一求解接触与关节约束 |
| `RigidBody` | 提供质心速度、接触点速度、应用冲量等最小物理操作 |

约束只保存 `BodyId`，不要保存 `PhysicsBody&` 或指向 `std::vector` 元素的指针，因为刚体容器扩容可能使地址失效。世界锚点可以用无效 body ID 表示；该端固定在世界坐标，逆质量和逆转动惯量均为零。

### 15.2 力生成器接口

一个简单的接口草案：

```cpp
struct ForceContext
{
    PhysicsWorld& world;
};

class ForceGenerator
{
public:
    virtual ~ForceGenerator() = default;
    virtual void Apply(ForceContext& context, float deltaTime) = 0;
};
```

`PhysicsWorld` 可拥有 `std::unique_ptr<ForceGenerator>`，并提供注册/移除接口。每个生成器通过稳定的 `BodyId` 查找目标刚体。第一版也可以先不做多态基类：将重力作为世界级加速度，在 `PhysicsWorld` 中显式累加；需要弹簧和外力后再抽象生成器。

重力是加速度，弹簧施加的是力，积分时应区分：

```cpp
linearVelocity += (gravity + force * inverseMass) * deltaTime;
```

当前 `RigidBody::IntegrateForces` 中的 `(gravity + force) * inverseMass * deltaTime` 会把重力也乘以逆质量。实现力生成器前应先统一这个语义，否则同一重力会因物体质量不同而产生不同加速度。

### 15.3 软弹簧力

两刚体弹簧保存两个局部锚点、静止长度、刚度和阻尼：

```cpp
struct SpringForce
{
    BodyId bodyA;
    BodyId bodyB;
    glm::vec2 localAnchorA{0.0f};
    glm::vec2 localAnchorB{0.0f};
    float restLength = 1.0f;
    float stiffness = 20.0f;
    float damping = 1.0f;
};
```

每个物理子步中，将局部锚点转换到世界坐标：

```cpp
pA = WorldPoint(bodyA.transform, localAnchorA);
pB = WorldPoint(bodyB.transform, localAnchorB);
n = normalize(pB - pA);
extension = length(pB - pA) - restLength;
```

锚点速度必须包含角速度：

```cpp
vPoint = linearVelocity + angularVelocity * perp(p - worldCenterOfMass);
relativeSpeed = dot(vB - vA, n);
```

弹簧在 A 上的力可写为：

```cpp
forceA = n * (stiffness * extension + damping * relativeSpeed);
forceB = -forceA;
```

同时累加锚点产生的力矩：`torqueA += cross(rA, forceA)`，`torqueB += cross(rB, forceB)`。固定端弹簧只向动态刚体施力。注意力方向和相对速度符号应通过“拉长时弹簧拉回、缩短时阻尼抵抗运动”的小测试验证。

显式弹簧力对时间步和刚度敏感。调试初期使用较小刚度；需要很硬的弹簧时，优先改成迭代求解的距离约束，或再考虑隐式弹簧/soft constraint，而不是不断增大刚度。

### 15.4 约束基类与求解阶段

建议让约束公开一致的阶段接口：

```cpp
class Constraint
{
public:
    virtual ~Constraint() = default;
    virtual void Prepare(SolverContext& context) = 0;
    virtual void WarmStart(SolverContext& context) = 0;
    virtual void SolveVelocity(SolverContext& context) = 0;
    virtual void SolvePosition(SolverContext& context) = 0;
};
```

`Prepare` 在每个子步开始时计算世界锚点、杆臂、有效质量和偏差项；`WarmStart` 应用上一子步缓存的累计冲量；`SolveVelocity` 修正相对速度；`SolvePosition` 消除残留的位置误差。只有创建时可设置的参数（body ID、局部锚点、静止长度等）应属于用户接口，求解缓存应为内部状态。

推荐每个物理子步采用如下顺序：

```text
1. 清理力累加器，应用重力和 ForceGenerator
2. 积分外力，更新动态刚体速度
3. 根据当前变换计算 AABB，执行 broad phase 和 narrow phase
4. Prepare 所有接触约束和用户约束
5. WarmStart
6. 重复 velocityIterations 次：求解接触和用户约束的速度
7. 用已修正的速度积分位置和角度
8. 重复 positionIterations 次：修正接触穿透和用户约束的位置/角度误差
9. 保存累计冲量，进入下一子步
```

接触约束和关节约束应在同一个 velocity iteration 循环里交替求解；否则关节可能把刚体拉回碰撞体内部，或碰撞响应破坏关节速度。位置误差修正也应协调进行。求解次序会影响收敛，可先固定顺序，之后再考虑按约束图分岛并行。当前项目在碰撞检测前积分位置；加入约束后建议逐步调整到上述顺序。离散检测仍可能漏掉高速穿越，先增加物理子步，之后再考虑连续碰撞检测（CCD）。

当前 `PhysicsWorld::Step(deltaTime, iteration)` 将时间步切成多份并重复 `StepOneIteration`；这个 `iteration` 实际表示**物理子步数**，不是同一状态上的约束求解轮数。建议重命名为 `substepCount`，另设 `velocityIterations` 和 `positionIterations`。每个子步只应用一次外力生成器，再在该子步内部执行多轮约束求解。

### 15.5 距离约束

距离约束要求两个锚点之间保持长度 `L`：

```text
C = length(pB - pA) - L = 0
n = normalize(pB - pA)
```

这是一个标量约束，只沿 `n` 方向产生冲量。每轮求解使用两锚点的点速度差、逆质量和转动惯量计算有效质量，并更新累计冲量。冲量需要累计并限制，而不是每轮独立应用；这能避免迭代轮数变化时响应强度随之变化，也为 warm starting 提供基础。

建议第一版使用带 bias/softness 的 sequential impulse 方法，而不是用弹簧常数很大的显式力逼近刚性。位置求解可用小比例误差修正和 slop，避免一次把误差完全消除造成抖动。

### 15.6 固定约束

固定约束（weld/fixed joint）在创建时记录两个刚体的相对状态，之后保持两个局部锚点重合，并保持相对角度不变：

```text
pA = WorldPoint(A.transform, localAnchorA)
pB = WorldPoint(B.transform, localAnchorB)
positionError = pB - pA
angleError = wrapAngle((angleB - angleA) - referenceAngle)
```

在 2D 中它有三个标量自由度约束：世界 X、世界 Y 两个平移约束，以及一个角度约束。位置部分与距离约束类似，但需要沿两个独立轴求解；角度部分使用角速度差与逆转动惯量求解。先实现无旋转版本也可以，但应把平移和旋转求解拆成明确的行，避免之后改接口。

固定到世界的约束令 B 为世界端，保存创建时的世界锚点和参考角度；不要创建一个可移动但“质量很大”的假刚体来代替静态端。

### 15.7 PhysicsWorld 接口建议

最终可以在世界中提供类似接口：

```cpp
BodyId CreateBody(...);
ConstraintId CreateDistanceConstraint(const DistanceConstraintDef& def);
ConstraintId CreateFixedConstraint(const FixedConstraintDef& def);
bool DestroyConstraint(ConstraintId id);
bool AddForce(BodyId body, const glm::vec2& force);
```

约束定义（`Def`）与运行时约束实例分开：定义便于调用方创建和保存配置；实例持有累计冲量、索引和 warm-start 缓存。若使用 `unique_ptr<Constraint>`，要通过 generation handle 或稳定 ID 管理删除，避免 `vector` 移动使外部引用悬空。

### 15.8 推荐实现顺序与测试

按以下顺序增量实现，每一步都用无 OpenGL 依赖的物理测试验证：

1. 明确外力与加速度语义，修正 gravity/force 积分，并测试不同质量物体具有相同重力加速度。
2. 为动态、静态、运动学刚体补齐锚点位置、锚点速度、力矩和冲量等公共数学操作。
3. 添加一个固定到世界的软弹簧力生成器，测试拉伸恢复、阻尼、静态端和角度力矩。
4. 添加距离约束的 `Prepare`、warm start、速度求解和位置求解；先测试两个圆，再测试带偏心锚点的旋转刚体。
5. 添加固定约束的位置和角度行，并验证两个刚体发生碰撞时约束仍能保持。
6. 将接触与用户约束放入同一求解迭代，比较不同迭代数下的长度误差和能量变化。

至少覆盖：静止长度、拉伸和压缩、两个动态刚体、一个静态端、偏心锚点、旋转、零长度/退化输入、约束删除、多个约束组成的链，以及高速碰撞下的约束稳定性。测试应检查误差随迭代次数下降，而不只检查“物体看起来连在一起”。
