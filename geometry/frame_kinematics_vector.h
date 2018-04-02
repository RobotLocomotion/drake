#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** A %FrameKinematicsVector is used to report kinematics data for registered
 frames (based on the frames registered by a particular source id).
 It serves as the basis of FramePoseVector, FrameVelocityVector, and
 FrameAccelerationVector. Geometry sources report the kinematics values for
 their registered frame through these classes.

 The %FrameKinematicsVector must be constructed with the source's SourceId and
 the number of frames *owned* by the source. Once constructed, it cannot be
 resized. Typically, this will be done in the allocation method of the
 LeafSystem which serves as a geometry source.

 Populating the vector with values is a two step process: clear and set. Before
 writing any kinematics data the vector should be _cleared_ (see clear()). After
 clearing, each registered frame will have a kinematics value assigned to it
 by calling set().

 Attempting to write more frames than the vector was constructed for is
 considered an error and will throw an exception. Failing to set data for every
 registered frame will be considered an error when the %FrameKinematicsVector
 is consumed by GeometrySystem.

 <!--
   TODO: The FrameVelocityVector and FrameAccelerationVector are still to come.
  -->

 The standard usage of this method would be in the allocation and calculation
 of a LeafSystem's output port as follows:

 ```
 class MySourceSystem : public LeafSystem<T> {
 private:
   geometry::FramePoseVector<T> AllocateFramePoseOutput(
        const MyContext& context) const {
     return geometry::FramePoseVector<T>(source_id_,
                                         static_cast<int>(frame_ids_.size()));
   }

   void CalcFramePoseOutput(const MyContext& context,
                            geometry::FramePoseVector<T>* poses) const {
     DRAKE_THROW_UNLESS(poses->source_id() == source_id_);
     DRAKE_THROW_UNLESS(poses->size() == static_cast<int>(frame_ids_.size()));
     poses->clear();
     for (int i = 0; i < static_cast<int>(frame_ids_.size()); ++i) {
       poses->set_value(frame_ids_[i], poses_[i]);
     }
   }

 SourceId source_id_;
 std::vector<FrameId> frame_ids_;
 std::vector<Isometry3<T>> poses_;
 };
 ```


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
  */
template <class KinematicsValue>
class FrameKinematicsVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameKinematicsVector)

  /** Constructs an empty vector. */
  explicit FrameKinematicsVector(SourceId source_id, int size);

  /** Initializes the vector to start setting kinematics values. */
  void clear();

  /** Sets the kinematics `value` for the frame indicated by the given `id`.
   After calling clear(), this method can be invoked N times, where N is the
   size value provided at construction. For the N + 1ˢᵗ invocation or later,
   an exception will be thrown.

   This function does _not_ prevent the same id being set twice. However, this
   will use up one of the "slots" of the vector. At best, this would lead to
   undefined behavior, at worst the duplicate value will supplant a required
   value and that will be detected when GeometrySystem consumes this data
   causing an exception to be thrown at that time. */
  void set_value(FrameId id, const KinematicsValue& value);

  SourceId source_id() const { return source_id_; }

  /** Returns the constructed size of this vector. */
  int size() const { return static_cast<int>(data_.size()); }

  /** Returns the number of poses that have been set since the last call to
   clear(). */
  int count() const { return next_pose_; }

  const std::vector<FrameId>& ids() const { return ids_; }
  const std::vector<KinematicsValue>& values() const { return data_; }

 private:
  // The source id for the geometry source reporting data in this vector
  SourceId source_id_;

  // The ids of the frames registered to source_id_
  std::vector<FrameId> ids_;

  // The per-frame kinematics data
  std::vector<KinematicsValue> data_;

  // The index of where the next reported pose will be written
  int next_pose_;
};

/** Class for communicating _pose_ information to GeometrySystem for registered
 frames.

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
