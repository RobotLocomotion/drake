#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_state.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace geometry {

/** The custom leaf context type for GeometrySystem and GeometryWorld.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported. */
template <typename T>
class GeometryContext final : public systems::LeafContext<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometryContext)

  /** Constructs the context with the given index for the geometry state. */
  explicit GeometryContext(int geometry_state_index);

  /** Returns a mutable reference of the underlying geometry state. */
  GeometryState<T>& get_mutable_geometry_state();

  /** Returns a const reference of the underlying geometry state. */
  const GeometryState<T>& get_geometry_state() const;

 private:
  // The index of the geometry state abstract state.
  const int geometry_state_index_{-1};
};

}  // namespace geometry
}  // namespace drake
