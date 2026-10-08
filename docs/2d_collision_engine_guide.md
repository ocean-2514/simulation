# 2D 碰撞检测实现指南

本文档针对当前项目的结构，说明如何从现有的 `Shape` 和 OpenGL 几何体逐步扩展出一个可测试、可维护的 2D 物理碰撞系统。

目标是让渲染和物理各自负责清晰的事情：

- `Shape`、`Circle`、`Polygon`：描述和绘制图形。
- `RigidBody`：描述物体的质量、位置、速度和旋转。
- `Collider`：描述碰撞形状和碰撞参数。
- `CollisionWorld`：查找可能碰撞的物体。
- `CollisionDetector`：计算碰撞结果。
- `Solver`：根据碰撞结果修正速度和位置。

不要在 `BufferGeometry` 或 `Shape` 中直接实现碰撞检测。OpenGL 的 VAO/VBO 只属于渲染层，碰撞检测不应该依赖 OpenGL Context。

---

## 1. 推荐目录结构

在 `engine/src/` 下增加以下目录：

```text
Physics/
    PhysicsTypes.hpp
    RigidBody.hpp
    Collider.hpp
    PhysicsWorld.hpp
    Collision/
        CollisionManifold.hpp
        CollisionDetector.hpp
        CircleCircle.hpp
        CirclePolygon.hpp
        PolygonPolygon.hpp
    BroadPhase/
        AABB.hpp
        SpatialHash.hpp
    Solver/
        ImpulseSolver.hpp
```

建议的职责边界：

| 类 | 职责 |
|---|---|
| `Shape` | 负责当前的 OpenGL 几何体和局部顶点数据 |
| `Collider` | 负责物理碰撞形状，不持有 VAO/VBO |
| `RigidBody` | 负责位置、旋转、速度和质量 |
| `CollisionDetector` | 只判断两个 Collider 是否相交，并生成碰撞信息 |
| `PhysicsWorld` | 管理刚体，执行 broad phase、narrow phase 和 solver |
| `ImpulseSolver` | 处理碰撞后的速度变化和位置修正 |

---

## 2. 先定义基础数学类型

当前项目已经使用 GLM，可以直接使用 `glm::vec2`。建议先定义统一的物理标量和变换类型：

```cpp
// Physics/PhysicsTypes.hpp
#pragma once

#include <glm/glm.hpp>

using PhysicsScalar = float;

struct Transform2D
{
    glm::vec2 position{0.0f};
    PhysicsScalar rotation = 0.0f;
};
```

不要把 `Shape::centroid` 直接当成世界坐标。它是局部坐标。世界质心应该通过刚体变换得到：

```cpp
glm::vec2 WorldPoint(const Transform2D& transform, const glm::vec2& localPoint)
{
    float c = glm::cos(transform.rotation);
    float s = glm::sin(transform.rotation);

    glm::vec2 rotated(
        c * localPoint.x - s * localPoint.y,
        s * localPoint.x + c * localPoint.y);

    return transform.position + rotated;
}
```

当前的 `Circle` 和 `Polygon` 都以局部坐标生成顶点，因此物理层也应该以局部坐标保存碰撞形状。

---

## 3. 定义碰撞形状 Collider

第一阶段只实现圆和凸多边形：

```cpp
// Physics/Collider.hpp
#pragma once

#include "Physics/PhysicsTypes.hpp"
#include <vector>

struct CircleCollider
{
    PhysicsScalar radius = 0.5f;
};

struct PolygonCollider
{
    std::vector<glm::vec2> vertices;
};

enum class ColliderType
{
    Circle,
    Polygon
};

struct Collider
{
    ColliderType type = ColliderType::Circle;
    CircleCollider circle;
    PolygonCollider polygon;
    Transform2D localTransform;
    PhysicsScalar restitution = 0.2f;
    PhysicsScalar friction = 0.5f;
};
```

建议让 Collider 只保存碰撞所需数据，不直接引用 `Circle`、`Polygon` 或 `BufferGeometry`。这样可以在没有窗口、没有 GLAD 的单元测试中验证碰撞算法。

### Shape 与 Collider 的关系

短期可以在创建渲染对象时同时创建对应的 Collider：

```cpp
struct PhysicsObject
{
    std::unique_ptr<Shape> renderShape;
    Collider collider;
    RigidBody body;
};
```

更长期可以用一个 ID 或实体组件系统关联它们，避免物理层持有渲染对象指针。

---

## 4. 定义刚体 RigidBody

刚体负责运动状态，不负责判断碰撞：

```cpp
// Physics/RigidBody.hpp
#pragma once

#include "Physics/PhysicsTypes.hpp"

class RigidBody
{
public:
    Transform2D transform;
    glm::vec2 linearVelocity{0.0f};
    PhysicsScalar angularVelocity = 0.0f;
    glm::vec2 force{0.0f};
    PhysicsScalar torque = 0.0f;

    PhysicsScalar inverseMass = 1.0f;
    PhysicsScalar inverseInertia = 0.0f;
    bool isStatic = false;

    void ApplyForce(const glm::vec2& value)
    {
        if (!isStatic) {
            force += value;
        }
    }

    void IntegrateForces(const glm::vec2& gravity, PhysicsScalar deltaTime)
    {
        if (isStatic) {
            return;
        }

        linearVelocity += (gravity + force * inverseMass) * deltaTime;
        force = glm::vec2(0.0f);
    }

    void IntegrateVelocity(PhysicsScalar deltaTime)
    {
        if (isStatic) {
            return;
        }

        transform.position += linearVelocity * deltaTime;
        transform.rotation += angularVelocity * deltaTime;
    }
};
```

第一版可以先不实现角速度和转动惯量，只处理平移。等圆形和多边形碰撞稳定后，再加入旋转。

注意：实际项目中要限制 `deltaTime`，否则调试断点或窗口卡顿会让物体一次移动太远：

```cpp
float step = glm::min(deltaTime, 1.0f / 30.0f);
```

更可靠的方式是使用固定时间步：

```cpp
accumulator += frameDeltaTime;
while (accumulator >= fixedDeltaTime) {
    world.Step(fixedDeltaTime);
    accumulator -= fixedDeltaTime;
}
```

推荐固定步长为 `1.0f / 60.0f`。

---

## 5. 定义 AABB 和碰撞结果

### AABB

Broad phase 先使用轴对齐包围盒过滤明显不可能碰撞的物体：

```cpp
// Physics/BroadPhase/AABB.hpp
#pragma once

#include <glm/glm.hpp>

struct AABB
{
    glm::vec2 min{0.0f};
    glm::vec2 max{0.0f};

    bool Overlaps(const AABB& other) const
    {
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y;
    }
};
```

Collider 应提供：

```cpp
AABB ComputeAABB(const Collider& collider, const Transform2D& transform);
```

### CollisionManifold

Narrow phase 不要只返回 `bool`，还要返回求解碰撞所需的信息：

```cpp
// Physics/Collision/CollisionManifold.hpp
#pragma once

#include <glm/glm.hpp>

struct CollisionManifold
{
    bool colliding = false;
    glm::vec2 normal{0.0f};
    glm::vec2 contactPoint{0.0f};
    float penetration = 0.0f;
};
```

约定：`normal` 始终从物体 A 指向物体 B。这个约定必须贯穿检测和求解，否则碰撞响应会出现反向弹开或物体互相吸引。

---

## 6. 设计 CollisionDetector 接口

```cpp
// Physics/Collision/CollisionDetector.hpp
#pragma once

#include "Physics/Collider.hpp"
#include "Physics/Collision/CollisionManifold.hpp"

class CollisionDetector
{
public:
    static CollisionManifold Detect(
        const Collider& first,
        const Transform2D& firstTransform,
        const Collider& second,
        const Transform2D& secondTransform);

private:
    static CollisionManifold CircleVsCircle(
        const CircleCollider& first,
        const Transform2D& firstTransform,
        const CircleCollider& second,
        const Transform2D& secondTransform);

    static CollisionManifold CircleVsPolygon(
        const CircleCollider& circle,
        const Transform2D& circleTransform,
        const PolygonCollider& polygon,
        const Transform2D& polygonTransform);

    static CollisionManifold PolygonVsPolygon(
        const PolygonCollider& first,
        const Transform2D& firstTransform,
        const PolygonCollider& second,
        const Transform2D& secondTransform);
};
```

推荐用 `.cpp` 实现算法，不要把复杂算法全部写在头文件里。

### 6.1 圆形 vs 圆形

设两个圆的世界圆心为 `pA`、`pB`，半径为 `rA`、`rB`：

```text
delta = pB - pA
距离平方 = dot(delta, delta)
半径和 = rA + rB
```

当：

```text
距离平方 < 半径和 * 半径和
```

发生碰撞。碰撞法线：

```cpp
normal = normalize(delta);
```

穿透深度：

```cpp
penetration = radiusSum - distance;
```

需要处理两个圆心完全重合的情况：此时不能归一化零向量，应使用一个默认法线，例如 `(1, 0)`。

### 6.2 圆形 vs 凸多边形

推荐使用“最近点”算法：

1. 将圆心转换到多边形的局部坐标。
2. 遍历每条边，找到圆心到边线段的最近点。
3. 找出距离最小的最近点。
4. 最近距离小于圆半径时发生碰撞。
5. 将碰撞法线和接触点转换回世界坐标。

前提是多边形必须是凸多边形，顶点顺序要统一为顺时针或逆时针。

### 6.3 凸多边形 vs 凸多边形

推荐使用 SAT（Separating Axis Theorem，分离轴定理）：

1. 取 A 的每条边的法线作为候选轴。
2. 取 B 的每条边的法线作为候选轴。
3. 将两个多边形投影到每条轴上。
4. 只要存在一个轴，两个投影区间没有重叠，就没有碰撞。
5. 如果所有轴都重叠，选择重叠量最小的轴作为碰撞法线。
6. 最小重叠量就是穿透深度的候选值。

SAT 只适合凸多边形。凹多边形要先分解成多个凸多边形，或使用更复杂的 GJK/EPA、耳切三角剖分等方案。

---

## 7. 添加 Broad Phase

最初物体数量很少时，可以使用 O(n²) 两两检测：

```cpp
for (size_t i = 0; i < bodies.size(); ++i) {
    for (size_t j = i + 1; j < bodies.size(); ++j) {
        if (!aabbs[i].Overlaps(aabbs[j])) {
            continue;
        }

        auto manifold = CollisionDetector::Detect(...);
        if (manifold.colliding) {
            contacts.push_back(...);
        }
    }
}
```

当物体数量增多后，再加入 `SpatialHash`：

```cpp
class SpatialHash
{
public:
    explicit SpatialHash(float cellSize);

    void Clear();
    void Insert(int bodyId, const AABB& aabb);
    std::vector<std::pair<int, int>> ComputePairs() const;
};
```

建议先实现 O(n²)，因为它更容易验证算法正确性。Broad phase 的优化不应该和 narrow phase 同时开发。

---

## 8. 定义 PhysicsWorld

```cpp
// Physics/PhysicsWorld.hpp
#pragma once

#include "Physics/RigidBody.hpp"
#include "Physics/Collider.hpp"
#include "Physics/Collision/CollisionDetector.hpp"
#include <vector>

struct PhysicsBody
{
    RigidBody rigidBody;
    Collider collider;
};

class PhysicsWorld
{
public:
    int CreateBody(const RigidBody& body, const Collider& collider);
    PhysicsBody& GetBody(int id);

    void Step(float fixedDeltaTime);

    glm::vec2 gravity{0.0f, -9.8f};

private:
    std::vector<PhysicsBody> bodies;
};
```

`Step()` 建议分成以下阶段：

```text
1. IntegrateForces
2. ComputeAABBs
3. BroadPhase
4. NarrowPhase
5. SolveVelocity
6. SolvePosition
7. IntegrateVelocity
```

初版也可以把 `IntegrateVelocity` 放在碰撞检测之前，但最终建议采用固定时间步和明确的求解阶段。

---

## 9. 碰撞响应：先实现位置修正

只检测而不修正时，两个物体会持续重叠。第一版先实现简单的位置修正：

```cpp
void CorrectPosition(
    RigidBody& first,
    RigidBody& second,
    const CollisionManifold& manifold)
{
    float inverseMassSum = first.inverseMass + second.inverseMass;
    if (inverseMassSum <= 0.0f) {
        return;
    }

    constexpr float slop = 0.005f;
    constexpr float percent = 0.8f;

    float correctionMagnitude =
        glm::max(manifold.penetration - slop, 0.0f) /
        inverseMassSum * percent;

    glm::vec2 correction = correctionMagnitude * manifold.normal;

    if (!first.isStatic) {
        first.transform.position -= correction * first.inverseMass;
    }
    if (!second.isStatic) {
        second.transform.position += correction * second.inverseMass;
    }
}
```

接着实现冲量响应。第一版只处理无旋转情况：

```cpp
void ApplyImpulse(
    RigidBody& first,
    RigidBody& second,
    const CollisionManifold& manifold,
    float restitution)
{
    glm::vec2 relativeVelocity = second.linearVelocity - first.linearVelocity;
    float velocityAlongNormal = glm::dot(relativeVelocity, manifold.normal);

    if (velocityAlongNormal > 0.0f) {
        return;
    }

    float inverseMassSum = first.inverseMass + second.inverseMass;
    if (inverseMassSum <= 0.0f) {
        return;
    }

    float impulseMagnitude =
        -(1.0f + restitution) * velocityAlongNormal / inverseMassSum;

    glm::vec2 impulse = impulseMagnitude * manifold.normal;

    if (!first.isStatic) {
        first.linearVelocity -= impulse * first.inverseMass;
    }
    if (!second.isStatic) {
        second.linearVelocity += impulse * second.inverseMass;
    }
}
```

完成平移版本后，再加入接触点相对质心、角速度和摩擦力。

---

## 10. 将物理状态同步到渲染

当前 `Stimulation` 使用固定的 `model` 矩阵：

```cpp
glm::mat4 model = glm::mat4(1.0f);
```

后续应从刚体生成模型矩阵：

```cpp
glm::mat4 model = glm::translate(
    glm::mat4(1.0f),
    glm::vec3(body.rigidBody.transform.position, 0.0f));

model = glm::rotate(
    model,
    body.rigidBody.transform.rotation,
    glm::vec3(0.0f, 0.0f, 1.0f));
```

重要：`Shape::centroid` 是局部坐标，碰撞系统使用的是刚体的世界变换。渲染质心时，质心点也必须乘以同一个 model 矩阵，否则图形移动或旋转后，质心标记会留在原地。

建议把当前的 `drawCentroid()` 改成接收模型矩阵，或者让质心使用普通图形 VAO 的同一套变换矩阵：

```cpp
void drawCentroid(const glm::mat4& model);
```

---

## 11. 推荐实现顺序

不要一次实现完整物理引擎。推荐按以下步骤推进：

### 第一步：纯数学测试

先不创建窗口、不调用 OpenGL，测试：

- 圆形 vs 圆形：相交、相切、分离、圆心重合。
- 圆形 vs 矩形：圆在内部、外部、边缘和角点附近。
- 凸多边形 vs 凸多边形：相交和分离。
- 平移和旋转后的世界坐标转换。

### 第二步：实现 RigidBody

先只加入：

- `position`
- `linearVelocity`
- `inverseMass`
- `isStatic`
- 重力

暂时不实现旋转和摩擦。

### 第三步：实现 CircleCollider

先完成圆形之间的检测和位置修正。这是最容易验证的完整闭环。

### 第四步：实现 AABB Broad Phase

先用 O(n²) 的 AABB 过滤，再调用圆形 narrow phase。

### 第五步：实现 PolygonCollider 和 SAT

确保只接受凸多边形，并验证顶点顺序。

### 第六步：加入冲量响应

先实现法线方向的弹性碰撞，再实现摩擦。

### 第七步：接入渲染

让每个物理物体对应一个渲染 Shape，并在每帧使用刚体变换更新 model 矩阵。

### 第八步：优化

最后再实现：

- 固定时间步
- 空间哈希
- 接触缓存
- 多次 solver iteration
- 旋转和摩擦
- 休眠机制

---

## 12. 当前项目需要提前调整的地方

### 12.1 不要让物理依赖 OpenGL

目前 `Shape` 继承 `BufferGeometry`，构造时会创建 VAO/VBO。这对渲染是合理的，但不适合物理测试。

推荐的过渡方案：

```cpp
struct PhysicsObject
{
    std::unique_ptr<Shape> renderShape;
    PhysicsBody physics;
};
```

物理算法只访问 `physics.collider` 和 `physics.rigidBody`，不要访问 `renderShape->VAO`。

### 12.2 明确局部坐标和世界坐标

- `Shape::vertices`：局部坐标。
- `Shape::centroid`：局部坐标。
- `RigidBody::transform.position`：世界坐标。
- 碰撞检测：使用转换后的世界坐标。
- 渲染：使用刚体变换矩阵。

### 12.3 不要用渲染顶点直接推导所有物理数据

渲染网格可能有很多细分顶点，物理形状通常需要更简单的数据：

- 圆：只需要圆心和半径。
- 矩形：只需要半宽和半高，或 4 个局部顶点。
- 多边形：只需要凸包顶点。

### 12.4 处理对象生命周期

OpenGL 资源必须在有效 Context 存在时创建和销毁。物理对象本身不应该触发 OpenGL API。这样即使在测试中构造 Collider，也不会再次遇到 VAO 初始化导致的段错误。

---

## 13. 最小可行版本接口清单

第一版至少实现这些接口：

```cpp
// 变换
WorldPoint(const Transform2D&, const glm::vec2&);

// 包围盒
AABB ComputeAABB(const Collider&, const Transform2D&);
bool AABB::Overlaps(const AABB&) const;

// 碰撞检测
CollisionManifold CollisionDetector::Detect(...);

// 运动
void RigidBody::IntegrateForces(const glm::vec2&, float);
void RigidBody::IntegrateVelocity(float);

// 响应
void CorrectPosition(RigidBody&, RigidBody&, const CollisionManifold&);
void ApplyImpulse(RigidBody&, RigidBody&, const CollisionManifold&, float);

// 世界
int PhysicsWorld::CreateBody(...);
void PhysicsWorld::Step(float fixedDeltaTime);
```

完成这些接口后，就已经具备一个可以运行的基础 2D 物理系统，而不是只在渲染层画出图形。

---

## 14. 验收标准

每完成一个阶段，都应该满足对应标准：

- 圆形静止在地面上，不穿透地面。
- 圆形从空中落下，速度受重力影响。
- 圆形撞击地面后能够弹起或停止。
- 两个圆不会互相穿透。
- 多边形与圆在边和角附近都能正确检测。
- 物理对象移动后，渲染图形和质心标记保持同步。
- 调整窗口大小后，碰撞世界坐标不依赖像素尺寸。
- 关闭窗口时不会在无效 OpenGL Context 上释放资源。

建议在加入更多图形前，先用两个圆完成完整的“创建、重力、检测、响应、渲染同步”闭环。
