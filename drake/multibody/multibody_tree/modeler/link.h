#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/body.h"

namespace drake {
namespace multiLink {

/// %Link provides the general abstraction of a Link with an API that
/// makes no assumption about whether a Link is rigid or deformable and neither
/// does it make any assumptions about the underlying physical model or
/// approximation.
/// As an element or component of a MultiLinkTree, a Link is a
/// MultiLinkTreeElement, and therefore it has a unique index of type LinkIndex
/// within the multiLink tree it belongs to.
///
/// A %Link contains a unique LinkFrame; see LinkFrame class documentation for
/// more information.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class RigidLink {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Link)

  /// Creates a %Link with a LinkFrame associated with it.
  RigidLink();

 private:
  const Body<T>* body_;
};

}  // namespace multiLink
}  // namespace drake
