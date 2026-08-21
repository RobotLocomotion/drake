#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <cstdint>

#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

using LinkIndex = BodyIndex;

class SpanningForest;

using LinkOrdinal = TypeSafeIndex<class LinkOrdinalTag>;

using JointTraitsIndex = TypeSafeIndex<class JointTraitsTag>;
using WeldedLinksAssemblyIndex = TypeSafeIndex<class WeldedLinksAssemblyTag>;
using LoopConstraintIndex = TypeSafeIndex<class LoopConstraintTag>;

/* Link properties that can affect how the forest model gets built. Or-ing
these also produces a LinkFlags object. */
enum class LinkFlags : uint32_t {
  kDefault = 0,
  kStatic = 1 << 0,          ///< Implicitly welded to World.
  kMustBeBaseBody = 1 << 1,  ///< Ensure connection to World if none.
  kMassless = 1 << 2,        ///< Can't be a terminal body in a tree.
  kShadow = 1 << 3           ///< Link is a shadow (internal use only).
};

/* Joint properties that can affect how the SpanningForest gets built. Or-ing
these also produces a JointFlags object. */
enum class JointFlags : uint32_t {
  kDefault = 0,
  kMustBeModeled = 1 << 0  ///< Model explicitly even if ignorable weld.
};

/* Options for how to build the SpanningForest. Or-ing these also produces a
ForestBuildingOptions object. These can be provided as per-model instance
options to locally override global options. */
enum class ForestBuildingOptions : uint32_t {
  kDefault = 0,
  kStatic = 1 << 0,                ///< Weld all links to World.
  kUseFixedBase = 1 << 1,          ///< Use welds rather than floating joints.
  kUseRpyFloatingJoints = 1 << 2,  ///< For floating, use RPY not quaternion.
  kOptimizeWeldedLinksAssemblies =
      1 << 3  ///< Make a single Mobod for welded Links.
};

// These overloads make the above enums behave like bitmasks for the operations
// we care about here.

inline LinkFlags operator|(LinkFlags left, LinkFlags right) {
  return static_cast<LinkFlags>(static_cast<unsigned>(left) |
                                static_cast<unsigned>(right));
}
inline LinkFlags operator&(LinkFlags left, LinkFlags right) {
  return static_cast<LinkFlags>(static_cast<unsigned>(left) &
                                static_cast<unsigned>(right));
}

inline JointFlags operator|(JointFlags left, JointFlags right) {
  return static_cast<JointFlags>(static_cast<unsigned>(left) |
                                 static_cast<unsigned>(right));
}
inline JointFlags operator&(JointFlags left, JointFlags right) {
  return static_cast<JointFlags>(static_cast<unsigned>(left) &
                                 static_cast<unsigned>(right));
}

inline ForestBuildingOptions operator|(ForestBuildingOptions left,
                                       ForestBuildingOptions right) {
  return static_cast<ForestBuildingOptions>(static_cast<unsigned>(left) |
                                            static_cast<unsigned>(right));
}
inline ForestBuildingOptions operator&(ForestBuildingOptions left,
                                       ForestBuildingOptions right) {
  return static_cast<ForestBuildingOptions>(static_cast<unsigned>(left) &
                                            static_cast<unsigned>(right));
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
