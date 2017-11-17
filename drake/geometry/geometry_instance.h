#pragma once

#include <memory>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

/** A geometry instance combines a geometry definition (i.e., a shape of some
 sort), a pose (relative to a parent "frame" P), material information, and an
 opaque collection of metadata. The parent frame can be a registered frame or
 another registered geometry. */
class GeometryInstance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryInstance)

  /** Constructor.
   @param X_PG   The pose of this geometry (`G`) in its parent's frame (`P`).
   @param shape  The underlying shape for this geometry instance. */
  GeometryInstance(const Isometry3<double>& X_PG, std::unique_ptr<Shape> shape);

  GeometryId id() const { return id_; }

  const Isometry3<double>& pose() const { return X_PG_; }
  void set_pose(const Isometry3<double>& X_PG) { X_PG_ = X_PG; }

  const Shape& shape() const {
    DRAKE_DEMAND(shape_ != nullptr);
    return *shape_;
  }

  /** Releases the shape from the instance. */
  std::unique_ptr<Shape> release_shape() { return std::move(shape_); }

 private:
  // The *globally* unique identifier for this instance. It is functionally
  // const (i.e. defined in construction) but not marked const to allow for
  // default copying/assigning.
  GeometryId id_{};

  // The pose of the geometry relative to the parent frame it hangs on.
  Isometry3<double> X_PG_;

  // The shape associated with this instance.
  copyable_unique_ptr<Shape> shape_;
};
}  // namespace geometry
}  // namespace drake
