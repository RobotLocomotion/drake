#pragma once

#include <bitset>

static const int MAX_NUM_COLLISION_FILTER_GROUPS = 128;

namespace DrakeCollision {

typedef std::bitset<MAX_NUM_COLLISION_FILTER_GROUPS> bitmask;

// Constants
extern const bitmask ALL_MASK;
extern const bitmask NONE_MASK;
extern const bitmask DEFAULT_GROUP;
}  // namespace DrakeCollision
