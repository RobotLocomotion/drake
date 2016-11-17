#pragma once

#include <bitset>
#include <memory>

#include "drake/multibody/collision/model.h"

static const int MAX_NUM_COLLISION_FILTER_GROUPS = 128;

namespace DrakeCollision {

std::unique_ptr<Model> newModel();

typedef std::bitset<MAX_NUM_COLLISION_FILTER_GROUPS> bitmask;

// Constants
extern const bitmask ALL_MASK;
extern const bitmask NONE_MASK;
extern const bitmask DEFAULT_GROUP;

}  // namespace DrakeCollision
