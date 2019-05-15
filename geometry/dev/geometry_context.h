#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/dev/geometry_state.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace geometry {
namespace dev {

/** The custom leaf context type for SceneGraph.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:

 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported. */
template <typename T>
class GeometryContext final : public systems::LeafContext<T> {
 public:
  /** Constructs the context with the given index for the geometry state. */
  explicit GeometryContext(int geometry_state_index);

  std::unique_ptr<systems::ContextBase>
  DoCloneWithoutPointers() const final {
    return std::unique_ptr<systems::ContextBase>(new GeometryContext<T>(*this));
  }

  /** Returns a mutable reference of the underlying geometry state. */
  GeometryState<T>& get_mutable_geometry_state();

  /** Returns a const reference of the underlying geometry state. */
  const GeometryState<T>& get_geometry_state() const;

 protected:
  GeometryContext(const GeometryContext<T>& other) = default;

 private:
  // The index of the geometry state abstract state.
  const int geometry_state_index_{-1};
};

}  // namespace dev
}  // namespace geometry
}  // namespace drake
