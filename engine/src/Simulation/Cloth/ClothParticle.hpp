#pragma once

#include <glm/glm.hpp>

struct ClothParticle {
    glm::vec3 position{0.0f};
    glm::vec3 oldPosition{0.0f};
    glm::vec3 velocity{0.0f};
    glm::vec3 force{0.0f};
    float invMass = 1.0f;
    bool isPinned = false;
};