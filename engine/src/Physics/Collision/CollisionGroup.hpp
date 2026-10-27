#pragma once

#include <cstdint>

enum class CollisionGroup : uint16_t {
    GROUP0 = 1 << 0,
    GROUP1 = 1 << 1,
    GROUP2 = 1 << 2,
    GROUP3 = 1 << 3,
    GROUP4 = 1 << 4,
    GROUP5 = 1 << 5,
    GROUP6 = 1 << 6,
    GROUP7 = 1 << 7,
    GROUP8 = 1 << 8,
    GROUP9 = 1 << 9,
    GROUP10 = 1 << 10,
    GROUP11 = 1 << 11,
    GROUP12 = 1 << 12,
    GROUP13 = 1 << 13,
    GROUP14 = 1 << 14,
    GROUP15 = 1 << 15,
};

inline constexpr uint16_t DefaultCollisionMask = 0xFFFFu;
