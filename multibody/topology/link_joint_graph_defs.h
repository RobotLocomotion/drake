#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

class SpanningForest;

using JointTypeIndex = TypeSafeIndex<class JointTypeTag>;
using ConstraintTypeIndex = TypeSafeIndex<class ConstraintTypeTag>;
using ConstraintIndex = TypeSafeIndex<class ConstraintTag>;
using LinkCompositeIndex = TypeSafeIndex<class LinkCompositeTag>;

/** Link properties that can affect how the Forest model gets built. Or-ing
these also produces a LinkFlags object. */
enum class LinkFlags : unsigned {
  Default = 0,
  Static = 1 << 0,           ///< Implicitly welded to World
  MustBeBaseBody = 1 << 1,   ///< Ensure connection to World if none
  TreatAsMassless = 1 << 2,  ///< Can't be a terminal body in a tree
  Shadow = 1 << 3            ///< Link is a shadow (internal use only)
};

/** Joint properties that can affect how the SpanningForest gets built. Or-ing
these also produces a JointFlags object. */
enum class JointFlags : unsigned {
  Default = 0,
  MustBeModeled = 1 << 0  ///< Model explicitly even if ignorable weld.
};

/** Options for how to build the SpanningForest. Or-ing these also
produces a ModelingOptions object. These can be provided as per-model instance
options to override global options. */
enum class ModelingOptions : unsigned {
  Default = 0,
  Static = 1 << 0,                ///< All Links are static in this instance
  UseFixedBase = 1 << 1,          ///< Use welds rather than floating joints
  UseRpyFloatingJoints = 1 << 2,  ///< For floating, use RPY not quaternion
  CombineLinkComposites = 1 << 3  ///< Make a single Mobod for welded Links
};

// These overloads make the above enums behave like bitmasks.

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

inline ModelingOptions operator|(ModelingOptions left, ModelingOptions right) {
  return static_cast<ModelingOptions>(static_cast<unsigned>(left) |
                                      static_cast<unsigned>(right));
}
inline ModelingOptions operator&(ModelingOptions left, ModelingOptions right) {
  return static_cast<ModelingOptions>(static_cast<unsigned>(left) &
                                      static_cast<unsigned>(right));
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
