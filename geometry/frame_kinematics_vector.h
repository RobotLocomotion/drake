#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** A %FrameKinematicsVector associates a std::vector with a geometry source.
 It serves as the basis of FramePoseVector, FrameVelocityVector, and
 FrameAccelerationVector. Geometry sources report the kinematics values for
 their registered frame through these classes.

 The %FrameKinematicsVector must be constructed with the source's SourceId
 and then one kinematics value (e.g., pose) must be added to the underlying
 vector for each registered frame. The values are interpreted by the order of
 FrameId values in the corresponding FrameIdVector; the iᵗʰ value is attributed
 to the frame identified by the iᵗʰ FrameId in the FrameIdVector.

 @internal The FrameVelocityVector and FrameAccelerationVector are still to
 come.

 The intent is for the vector to be manipulated directly. Access the vector
 through a call to mutable_vector() and operate directly on the vector to
 improve performance. For example, for a FramePoseVector `poses`:

   - Reserving space for `n` frame poses (e.g.,
     `poses.mutable_vector().reserve(n);`).
   - Making use of `emplace_back()` or writing directly to mutable references
     via `poses.mutable_vector()[i] = KinematicsValue()`.

 @tparam KinematicsValue  The underlying data type of for the order of
                          kinematics data (e.g., pose, velocity, or
                          acceleration).

 One should never interact with the %FrameKinematicsVector class directly.
 Instead, the FramePoseVector, FrameVelocityVector, and FrameAccelerationVector
 classes are aliases of the %FrameKinematicsVector instantiated on specific
 data types (Isometry3, SpatialVector, and SpatialAcceleration, respectively).
 Each of these data types are templated on Eigen scalars. All supported
 combinations of data type and scalar type are already available to link against
 in the containing library. No other values for KinematicsValue are supported.

 Currently, the following data types with the following scalar types are
 supported:

  Alias           | Instantiation                            | Scalar types
 -----------------|------------------------------------------|--------------
  FramePoseVector | FrameKinematicsVector<Isometry3<Scalar>> | double
  FramePoseVector | FrameKinematicsVector<Isometry3<Scalar>> | AutoDiffXd

  @see FrameIdVector */
template <class KinematicsValue>
class FrameKinematicsVector {
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
  const std::vector<KinematicsValue>& vector() const { return vector_; }
  std::vector<KinematicsValue>& mutable_vector() { return vector_; }

 private:
  // The underlying data.
  std::vector<KinematicsValue> vector_;

  // The source id this data is associated with.
  SourceId source_id_;
};

/** Class for communicating ordered _pose_ information to GeometryWorld/
 GeometrySystem for registered frames.

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.
 */
template <typename T>
using FramePoseVector = FrameKinematicsVector<Isometry3<T>>;

}  // namespace geometry
}  // namespace drake
