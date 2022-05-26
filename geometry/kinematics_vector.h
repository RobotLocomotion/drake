#pragma once

#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

/** A %KinematicsVector is used to report kinematics data for registered
 frames and geometries (identified by unique FrameId/GeometryId values) to
 SceneGraph. It serves as the basis of FramePoseVector and
 GeometryConfigurationVector.

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
 @tparam KinematicsValue  The underlying data type of for the order of
                          kinematics data (e.g., pose, configuration, or
                          velocity).

 One should never interact with the %KinematicsVector class directly.
 Instead, the FramePoseVector and GeometryConfigurationVector classes are
 aliases of the %KinematicsVector instantiated on specific data types
 (RigidTransform and VectorX respectively). Each of these data types are
 templated on Eigen scalars. All supported combinations of data type and scalar
 type are already available to link against in the containing library. No other
 values for KinematicsValue are supported.

 Currently, the following data types with the following scalar types are
 supported:

  Alias                       | Instantiation                                    | Scalar types
 -----------------------------|--------------------------------------------------|-------------
  FramePoseVector             | KinematicsVector<FrameId,RigidTransform<Scalar>> | double/AutoDiffXd/Expression
  GeometryConfigurationVector | KinematicsVector<GeometryId, VectorX<Scalar>>    | double/AutoDiffXd/Expression
 */
template <class Id, class KinematicsValue>
class KinematicsVector {
 public:
  /** Initializes the vector using an invalid SourceId with no frames .*/
  KinematicsVector();

  /** Initializes the vector using an invalid SourceId and the given frames and
  kinematics values. */
  KinematicsVector(
      std::initializer_list<std::pair<const Id, KinematicsValue>> init);

  /** Default copy, move, and assign. */
  KinematicsVector(const KinematicsVector&);
  KinematicsVector(KinematicsVector&&);
  KinematicsVector& operator=(const KinematicsVector&);
  KinematicsVector& operator=(KinematicsVector&&);

  ~KinematicsVector();

  /** Resets the vector to the given frames and kinematics values .*/
  KinematicsVector& operator=(
      std::initializer_list<std::pair<const Id, KinematicsValue>> init);

  /** Clears all values, resetting the size to zero. */
  void clear();

  /** Sets the kinematics `value` for the frame indicated by the given `id`. */
  void set_value(Id id, const KinematicsValue& value);

  /** Returns number of frame_ids(). */
  int size() const;

  /** Returns the value associated with the given `id`.
   @throws std::exception if `id` is not in the specified set of ids.  */
  const KinematicsValue& value(Id id) const;

  /** Reports true if the given id is a member of this data. */
  bool has_id(Id id) const;

  /** Provides a range object for all of the existing ids in the vector.
   This is intended to be used as:
   @code
   for (Id id : this_vector.frame_ids()) {
    ...
    // Obtain the KinematicsValue of an id by `this_vector.value(id)`
    ...
   }
   @endcode
   */
  DRAKE_DEPRECATED("2022-09-01", "Use ids() instead.")
  std::vector<Id> frame_ids() const;

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

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
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
