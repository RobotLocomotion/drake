#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/**
 A geometry instance combines a geometry definition (i.e., a shape of some
 sort), a pose (relative to a parent frame), material information, and an
 opaque collection of metadata.

 @tparam T  The underlying scalar type. Must be a valid Eigen scalar.
            Only double is supported. */
// TODO(SeanCurtis-TRI): Add support for AutoDiffXd.
template <typename T>
class GeometryInstance {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryInstance)

  GeometryInstance() = default;
};
}  // namespace geometry
}  // namespace drake
