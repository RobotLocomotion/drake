#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** A FrameKinematicsVector is a std::vector associated with a geometry source.
 As such, it is constructed with a SourceId but has all std::vector operations
 available to it. This class serves as the basis of FramePoseVector,
 FrameVelocityVector, and FrameAccelerationVector.

 When computing frame kinematics values for updating the state of GeometryWorld/
 GeometrySystem, we recommend making full use of the std::vector functionality,
 e.g.,

   - Reserving space for the known number of frame ids (e.g.,
     `pose_vector.reserve(n);`).
   - Making use of `emplace_back()` or writing directly to mutable references
     via `pose_vector[i] = KinematicsValue()`.

 @tparam KinematicsValue  The underlying data type of for the order of
                          kinematics data (e.g., pose, velocity, or
                          acceleration).

 Instantiated templates for the following kinds of kinematics values are
 provided:
 - Isometry3 (for poses),
 - SpatialVelocity (for velocities),
 - SpatialAcceleration (for accelerations).

 They are already available to link against in the containing library. No other
 values for KinematicsValue are supported. */
template <class KinematicsValue>
class FrameKinematicsVector : public std::vector<KinematicsValue> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameKinematicsVector)

  /** Constructs an empty vector. */
  explicit FrameKinematicsVector(SourceId source_id);

  /** Copy constructs from a std::vector of KinematicsValue type. */
  FrameKinematicsVector(SourceId source_id,
                        const std::vector<KinematicsValue>& values);

  /** Move constructs from a std::vector of KinematicsValue type. */
  FrameKinematicsVector(SourceId source_id,
                        std::vector<KinematicsValue>&& values);

  SourceId get_source_id() const { return source_id_; }

 private:
  // The source id this data is associated with.
  SourceId source_id_;
};

/** Class for communicating ordered _pose_ information to GeometryWorld/
 GeometrySystem for registered frames.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double

 They are already available to link against in the containing library.
 No other values for T are currently supported.
 */
template <typename T>
using FramePoseVector = FrameKinematicsVector<Isometry3<T>>;

}  // namespace geometry
}  // namespace drake
