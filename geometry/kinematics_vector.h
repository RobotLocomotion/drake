#pragma once

#include <initializer_list>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// Disable formatter to preserve doxygen tables.
// clang-format off
/** A %KinematicsVector is a container class used to report kinematics data for
 registered frames and geometries (keyed by unique FrameId/GeometryId values)
 to SceneGraph where the set of keys (FrameId/GeometryId) is usually constant
 and the values (kinematics data) are varying. It is an internal class and one
 should never interact with it directly. The template aliases FramePoseVector
 and GeometryConfigurationVector that instantiate %KinematicsVector should be
 used instead.

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

 If a System only ever emits a single frame/geometry (or small-constant-number
 of frames/geometries), then there's a shorter alternative way to write a Calc
 method, using an initializer_list:
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

 @tparam Id               The key used to locate the kinematics data. Can be
                          FrameId or GeometryId.
 @tparam KinematicsValue  The underlying data type of the kinematics data (e.g.,
                          pose, configuration, or velocity).

 The FramePoseVector and GeometryConfigurationVector classes are aliases of the
 %KinematicsVector instantiated on specific data types (RigidTransform and
 VectorX respectively). Each of these data types are templated on Eigen scalars.
 All supported combinations of data type and scalar type are already available
 to link against in the containing library. No other values for KinematicsValue
 are supported.

 Currently, the following data types with the following scalar types are
 supported:

  Alias                               | Instantiation                                    | Scalar types
 -------------------------------------|--------------------------------------------------|-------------
  FramePoseVector<Scalar>             | KinematicsVector<FrameId,RigidTransform<Scalar>> | double/AutoDiffXd/Expression
  GeometryConfigurationVector<Scalar> | KinematicsVector<GeometryId, VectorX<Scalar>>    | double/AutoDiffXd/Expression
 */
// clang-format on
template <class Id, class KinematicsValue>
class KinematicsVector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(KinematicsVector);

  /** Initializes the vector with no data .*/
  KinematicsVector();

  /** Initializes the vector using the given the keys and their corresponding
   kinematics values. */
  KinematicsVector(
      std::initializer_list<std::pair<const Id, KinematicsValue>> init);

  ~KinematicsVector();

  /** Resets the vector to the given keys and kinematics values .*/
  KinematicsVector& operator=(
      std::initializer_list<std::pair<const Id, KinematicsValue>> init);

  /** Clears all values, resetting the size to zero. */
  void clear();

  /** Sets the kinematics `value` for the given `id`. */
  void set_value(Id id, const KinematicsValue& value);

  /** Returns number of ids(). */
  int size() const {
    DRAKE_ASSERT_VOID(CheckInvariants());
    return size_;
  }

  /** Returns the value associated with the given `id`.
   @throws std::exception if `id` is not in the specified set of ids.  */
  const KinematicsValue& value(Id id) const;

  /** Reports true if the given id is a member of this data. */
  bool has_id(Id id) const;

  /** Provides a range object for all of the existing ids in the vector.
   This is intended to be used as:
   @code
   for (Id id : this_vector.ids()) {
    ...
    // Obtain the KinematicsValue of an id by `this_vector.value(id)`
    ...
   }
   @endcode
   */
  std::vector<Id> ids() const;

  /** Reports if *all* values are finite. */
  bool IsFinite() const;

 private:
  void CheckInvariants() const;

  // Mapping from id to its corresponding kinematics value.  If the map's
  // optional value is nullopt, we treat it as if the map key were absent
  // instead.  We do this in order to avoid reallocating map nodes as we
  // repeatedly clear() and then re-set_value() the same IDs over and over
  // again.
  // TODO(jwnimmer-tri) A better way to avoid map node allocations would be to
  // replace this unordered_map with a flat_hash_map (where the entire storage
  // is a single heap slab); in that case, the complicated implementation in
  // the cc file would become simplified.
  std::unordered_map<Id, std::optional<KinematicsValue>> values_;

  // The count of non-nullopt items in values_.  We could recompute this from
  // values_, but we store it separately so that size() is still constant-time.
  int size_{0};
};

/** Class for communicating _pose_ information to SceneGraph for registered
 frames.

 @tparam_default_scalar
 */
template <typename T>
using FramePoseVector = KinematicsVector<FrameId, math::RigidTransform<T>>;

/** Class for communicating _configuration_ information to SceneGraph for
 registered deformable geometries.

 @tparam_default_scalar
 */
template <typename T>
using GeometryConfigurationVector = KinematicsVector<GeometryId, VectorX<T>>;

}  // namespace geometry
}  // namespace drake
