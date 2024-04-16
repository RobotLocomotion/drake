#pragma once

#include "drake/multibody/parsing/detail_collision_filter_groups_impl.h"
#include "drake/multibody/parsing/detail_instanced_name.h"

namespace drake {
namespace multibody {
namespace internal {

// Collision filter groups that use InstancedName. This representation is
// invariant with respect to model renaming via the plant.
using CollisionFilterGroupsStorage = CollisionFilterGroupsImpl<InstancedName>;

// Storage for internals that need to have the same lifetime as a Parser
// instance. This struct is only partially typed (forward reference) in
// parser.h, but fully typed here. The goal is to keep internal namespace
// details out of parser.h.
struct ParserInternalData {
  CollisionFilterGroupsStorage collision_filter_groups_storage_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
