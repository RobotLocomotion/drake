#pragma once

#include <memory>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

/**
 A geometry instance combines a geometry definition (i.e., a shape of some
 sort), a pose (relative to a parent frame), material information, and an
 opaque collection of metadata.

 @tparam T  The underlying scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
    - double

 They are already available to link against in the containing library.
 No other values for T are currently supported. */
// TODO(SeanCurtis-TRI): Add support for AutoDiffXd.
template <typename T>
class GeometryInstance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryInstance)

  /** Constructor.
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   @param shape  The underlying shape for this geometry instance. */
  GeometryInstance(const Isometry3<T>& X_PG, std::unique_ptr<Shape> shape);

  const Isometry3<T>& get_pose() const { return X_FG_; }
  void set_pose(const Isometry3<T>& X_PG) { X_FG_ = X_PG; }

  const Shape& get_shape() const { return *shape_; }

 private:
  // The pose of the geometry relative to the source frame it ultimately hangs
  // from.
  Isometry3<T> X_FG_;

  // The shape associated with this instance.
  copyable_unique_ptr<Shape> shape_;
};
}  // namespace geometry
}  // namespace drake
