#pragma once

#include "Physics/Collision/CollisionManifold.hpp"
#include "Physics/PhysicsTypes.hpp"

#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

struct ContactCacheKey {
    BodyId bodyA = InvalidIndex;
    BodyId bodyB = InvalidIndex;
    ContactFeatureId feature{};

    bool operator==(const ContactCacheKey& other) const {
        return bodyA == other.bodyA
            && bodyB == other.bodyB
            && feature.referenceIsA == other.feature.referenceIsA
            && feature.referenceFace == other.feature.referenceFace
            && feature.incidentFace == other.feature.incidentFace
            && feature.incidentVertex == other.feature.incidentVertex
            && feature.clipSide == other.feature.clipSide;
    }
};

struct CachedContactImpulse {
    glm::vec2 normal{0.0f};
    float normalImpulse = 0.0f;
    float tangentImpulse = 0.0f;
    float deltaTime = 0.0f;
    uint32_t lastSeenStep = 0;
};

struct ContactCacheKeyHash {
    std::size_t operator()(const ContactCacheKey& key) const {
        std::size_t h = std::hash<uint32_t>{}(key.bodyA);
        auto combine = [&h](std::size_t value) {
            h ^= value + 0x9e3779b9u + (h << 6) + (h >> 2);
        };
        combine(std::hash<uint32_t>{}(key.bodyB));
        combine(key.feature.referenceIsA);
        combine(key.feature.referenceFace);
        combine(key.feature.incidentFace);
        combine(key.feature.incidentVertex);
        combine(key.feature.clipSide);
        return h;
    }
};

class ContactCache {
public:
    void BeginStep(uint32_t step) {
        m_currentStep = step;
    }

    bool Restore(const ContactCacheKey& key,
        const glm::vec2& normal,
        float deltaTime,
        float& normalImpulse,
        float& tangentImpulse) {
        normalImpulse = 0.0f;
        tangentImpulse = 0.0f;

        auto it = m_entries.find(key);
        if (it == m_entries.end()) return false;

        CachedContactImpulse& entry = it->second;
        entry.lastSeenStep = m_currentStep;

        const float normalLengthSquared = glm::dot(normal, normal);
        if (!(normalLengthSquared > 1e-12f)
            || !std::isfinite(normalLengthSquared)
            || !(deltaTime > 1e-8f)
            || !std::isfinite(deltaTime)) {
            entry.normalImpulse = 0.0f;
            entry.tangentImpulse = 0.0f;
            return false;
        }
        const glm::vec2 normalizedNormal =
            normal * glm::inversesqrt(normalLengthSquared);
        const float cachedNormalLengthSquared = glm::dot(entry.normal, entry.normal);
        if (!(cachedNormalLengthSquared > 1e-12f)
            || !std::isfinite(cachedNormalLengthSquared)) {
            entry.normalImpulse = 0.0f;
            entry.tangentImpulse = 0.0f;
            return false;
        }
        const glm::vec2 cachedNormal = entry.normal
            * glm::inversesqrt(cachedNormalLengthSquared);

        // 法线变化太大，旧冲量方向不可信，清掉。
        if (glm::dot(cachedNormal, normalizedNormal) < 0.9f) {
            entry.normalImpulse = 0.0f;
            entry.tangentImpulse = 0.0f;
            return false;
        }

        // 时间步变化时，冲量按 dt 比例缩放。
        float scale = 1.0f;
        if (entry.deltaTime > 1e-8f && std::isfinite(entry.deltaTime)) {
            scale = deltaTime / entry.deltaTime;
        } else {
            return false;
        }
        scale = std::clamp(scale, 0.5f, 2.0f);

        normalImpulse = std::isfinite(entry.normalImpulse)
            ? std::max(entry.normalImpulse * scale, 0.0f)
            : 0.0f;
        tangentImpulse = std::isfinite(entry.tangentImpulse)
            ? entry.tangentImpulse * scale
            : 0.0f;
        return true;
    }

    void Store(const ContactCacheKey& key,
               const glm::vec2& normal,
               float normalImpulse,
               float tangentImpulse,
               float deltaTime) {
        CachedContactImpulse& entry = m_entries[key];
        const float normalLengthSquared = glm::dot(normal, normal);
        const bool validNormal = normalLengthSquared > 1e-12f
            && std::isfinite(normalLengthSquared);
        entry.normal = validNormal
            ? normal * glm::inversesqrt(normalLengthSquared)
            : glm::vec2{0.0f};
        entry.normalImpulse = validNormal && std::isfinite(normalImpulse)
            ? std::max(normalImpulse, 0.0f)
            : 0.0f;
        entry.tangentImpulse = validNormal && std::isfinite(tangentImpulse)
            ? tangentImpulse
            : 0.0f;
        entry.deltaTime = deltaTime > 1e-8f && std::isfinite(deltaTime)
            ? deltaTime
            : 0.0f;
        entry.lastSeenStep = m_currentStep;
    }

    void EndStep(uint32_t maxAge = 3) {
        for (auto it = m_entries.begin(); it != m_entries.end();) {
            if (m_currentStep - it->second.lastSeenStep > maxAge) {
                it = m_entries.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    std::unordered_map<ContactCacheKey, CachedContactImpulse,
        ContactCacheKeyHash> m_entries;
    uint32_t m_currentStep = 0;
};
