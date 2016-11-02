#pragma once

#include <bitset>
#include <memory>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/collision/Model.h"

static const int MAX_NUM_COLLISION_FILTER_GROUPS = 128;

namespace DrakeCollision {

DRAKE_EXPORT std::unique_ptr<Model> newModel();

typedef std::bitset<MAX_NUM_COLLISION_FILTER_GROUPS> bitmask;

// Constants
extern const DRAKE_EXPORT bitmask ALL_MASK;
extern const DRAKE_EXPORT bitmask NONE_MASK;
extern const DRAKE_EXPORT bitmask DEFAULT_GROUP;

}  // namespace DrakeCollision
