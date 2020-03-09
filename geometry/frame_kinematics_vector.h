#pragma once

#include <initializer_list>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

/** A %FrameKinematicsVector is used to report kinematics data for registered
 frames (identified by unique FrameId values) to SceneGraph.
 It serves as the basis of FramePoseVector, FrameVelocityVector, and
 FrameAccelerationVector.

 <!--
   TODO(SeanCurtis-TRI): The FrameVelocityVector and FrameAccelerationVector
   are still to come.
  -->

 ```
 template <typename T>
 class MySystem : public LeafSystem<T> {
  public:
   MySystem() {
     ...
     this->DeclareAbstractOutputPort(
         &AllocInConstructorSystem::CalcFramePoseOutput);
     ...
   }

  private:
   void CalcFramePoseOutput(const Context<T>& context,
                            geometry::FramePoseVector<T>* poses) const {
     poses->clear();
     for (int i = 0; i < static_cast<int>(frame_ids_.size()); ++i) {
       poses->set_value(frame_ids_[i], poses_[i]);
     }
   }

   std::vector<FrameId> frame_ids_;
   std::vector<RigidTransform<T>> poses_;
 };
 ```

 If a System only ever emits a single frame (or small-constant-number of
 frames), then there's a shorter alternative way to write a Calc method, using
 an initializer_list:
 ```
   void CalcFramePoseOutput(const Context<T>& context,
                            geometry::FramePoseVector<T>* poses) const {
     const RigidTransform<T>& pose = ...;
     *poses = {{frame_id_, pose}};
   }
 ```

 N.B. When the systems framework calls the `Calc` method, the value pointed to
 by `poses` is in an unspecified state.  The implementation of `Calc` must
 always ensure that `poses` contains the correct value upon return, no matter
 what value it started with.  The easy ways to do this are to call either
 `poses->clear()` or the assignment operator `*poses = ...`.

 @tparam KinematicsValue  The underlying data type of for the order of
                          kinematics data (e.g., pose, velocity, or
                          acceleration).

 One should never interact with the %FrameKinematicsVector class directly.
 Instead, the FramePoseVector, FrameVelocityVector, and FrameAccelerationVector
 classes are aliases of the %FrameKinematicsVector instantiated on specific
 data types (RigidTransform, SpatialVector, and SpatialAcceleration,
 respectively). Each of these data types are templated on Eigen scalars. All
 supported combinations of data type and scalar type are already available to
 link against in the containing library. No other values for KinematicsValue are
 supported.

 Currently, the following data types with the following scalar types are
 supported:

  Alias           | Instantiation                                 | Scalar types
 -----------------|-----------------------------------------------|-------------
  FramePoseVector | FrameKinematicsVector<RigidTransform<Scalar>> | double
  FramePoseVector | FrameKinematicsVector<RigidTransform<Scalar>> | AutoDiffXd
  FramePoseVector | FrameKinematicsVector<RigidTransform<Scalar>> | Expression
  */
template <class KinematicsValue>
class FrameKinematicsVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameKinematicsVector)

  /** Initializes the vector using an invalid SourceId with no frames .*/
  FrameKinematicsVector();

  /** Initializes the vector using an invalid SourceId and the given frames and
  kinematics values. */
  FrameKinematicsVector(
      std::initializer_list<std::pair<const FrameId, KinematicsValue>> init);

  /** Resets the vector to the given frames and kinematics values .*/
  FrameKinematicsVector& operator=(
      std::initializer_list<std::pair<const FrameId, KinematicsValue>> init);

  /** Clears all values, resetting the size to zero. */
  void clear();

  /** Sets the kinematics `value` for the frame indicated by the given `id`. */
  void set_value(FrameId id, const KinematicsValue& value);

  /** Returns number of frame_ids(). */
  int size() const {
    DRAKE_ASSERT_VOID(CheckInvariants());
    return size_;
  }

  /** Returns the value associated with the given `id`.
   @throws std::runtime_error if `id` is not in the specified set of ids.  */
  const KinematicsValue& value(FrameId id) const;

  /** Reports true if the given id is a member of this data. */
  bool has_id(FrameId id) const;

  /** Provides a range object for all of the frame ids in the vector.
   This is intended to be used as:
   @code
   for (FrameId id : this_vector.frame_ids()) {
    ...
    // Obtain the KinematicsValue of an id by `this_vector.value(id)`
    ...
   }
   @endcode
   */
  std::vector<FrameId> frame_ids() const;

 private:
  void CheckInvariants() const;

  // Mapping from frame id to its current pose.  If the map's optional value is
  // nullopt, we treat it as if the map key were absent instead.  We do this in
  // order to avoid reallocating map nodes as we repeatedly clear() and then
  // re-set_value() the same IDs over and over again.
  // TODO(jwnimmer-tri) A better way to avoid map node allocations would be to
  // replace this unordered_map with a flat_hash_map (where the entire storage
  // is a single heap slab); in that case, the complicated implementation in
  // the cc file would become simplified.
  std::unordered_map<FrameId, std::optional<KinematicsValue>> values_;

  // The count of non-nullopt items in values_.  We could recompute this from
  // values_, but we store it separately so that size() is still constant-time.
  int size_{0};
};

/** Class for communicating _pose_ information to SceneGraph for registered
 frames.

 @tparam_default_scalar
 */
template <typename T>
using FramePoseVector = FrameKinematicsVector<math::RigidTransform<T>>;

}  // namespace geometry
}  // namespace drake
