#pragma once

#include <map>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

/// An isometric transform along with the frame in which it
/// is defined. Useful for keeping collections of transforms
/// in different frames.
///
/// @note
/// Instantiated templates for the following scalar types
/// @p T are provided:
/// - double
template <typename T>
struct FramedIsometry3 {
  FramedIsometry3() {}

  FramedIsometry3(const Isometry3<T>& isometry_in,
                  const std::string& frame_in) :
      isometry(isometry_in),
      frame(frame_in) {}

  Isometry3<T> isometry{Isometry3<T>::Identity()};
  std::string frame{};
};

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
