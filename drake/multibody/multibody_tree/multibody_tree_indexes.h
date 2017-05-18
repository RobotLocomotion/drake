#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace multibody {

/// Type used to identify Frame objects by index in a multibody tree system.
using FrameIndex = TypeSafeIndex<class FrameTag>;

/// Type used to identify Body objects by index in a multibody tree system.
using BodyIndex = TypeSafeIndex<class BodyTag>;

/// Type used to identify Mobilizer objects by index in a multibody tree system.
using MobilizerIndex = TypeSafeIndex<class MobilizerTag>;

/// Type used to identify tree nodes by index within a multibody tree system.
using BodyNodeIndex = TypeSafeIndex<class BodyNodeTag>;

/// For every MultibodyTree the **world** body _always_ has this unique index
/// and it is always zero.
// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// https://google.github.io/styleguide/cppguide.html#Static_and_Global_Variables
inline BodyIndex world_index() { return BodyIndex(0); }

}  // namespace multibody
}  // namespace drake
