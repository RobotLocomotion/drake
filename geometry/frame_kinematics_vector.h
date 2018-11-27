#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace geometry {

#ifndef DRAKE_DOXYGEN_CXX
namespace detail {

// A type-traits-style initializer for making sure that quantities in the frame
// kinematics vector are initialized. This is because Eigen actively does *not*
// initialize its data types and there is a code path by which uninitialized
// values in the vector can be accessed.

template <typename Value>
struct KinematicsValueInitializer {
  static void Initialize(Value* value) {
    std::logic_error("Unsupported kinematics value");
  }
};

template <typename S>
struct KinematicsValueInitializer<Isometry3<S>> {
  static void Initialize(Isometry3<S>* value) {
    value->setIdentity();
  }
};

// TODO(SeanCurtis-TRI): Add specializations for SpatialVelocity and
// SpatialAcceleration (setting them to zero vectors) as those quantities are
// added to SceneGraph.

}  // namespace detail
#endif  // DRAKE_DOXYGEN_CXX

/** A %FrameKinematicsVector is used to report kinematics data for registered
 frames (identified by unique FrameId values) to SceneGraph.
 It serves as the basis of FramePoseVector, FrameVelocityVector, and
 FrameAccelerationVector.

 The %FrameKinematicsVector must be constructed with the source's SourceId and
 the ids of the frames *owned* by the source. Once constructed, it cannot be
 resized. Typically, this will be done in the allocation method of the
 LeafSystem which serves as a geometry source.

 Populating the vector with values is a two-step process: clear and set. Before
 writing any kinematics data the vector should be _cleared_ (see clear()). After
 clearing, each registered frame will have a kinematics value assigned to it
 by calling set_value().

 Only those frame ids provided in the constructor can be set. Attempting to
 set the value for any other frame id is considered an error.
 Attempting to write more frames than the vector was constructed for is
 considered an error and will throw an exception. Failing to set data for every
 registered frame will be considered an error when the %FrameKinematicsVector
 is consumed by SceneGraph.

 <!--
   TODO(SeanCurtis-TRI): The FrameVelocityVector and FrameAccelerationVector
   are still to come.
  -->

 The usage of this method would be in the allocation and calculation
 of a LeafSystem's output port. The nature of the allocation depends on whether
 the source id and number of frames are available at construction or not. The
 first example shows the case where source id and frame count are known. The
 second shows the alternate, deferred case.

 ```
 template <typename T>
 class AllocInConstructorSystem : public LeafSystem<T> {
  public:
   explicit AllocInConstructorSystem(SourceId source_id)
       : source_id_(source_id) {
     ...
     // Register frames, storing ids in frame_ids_
     this->DeclareAbstractOutputPort(
         FramePoseVector<T>(source_id, frame_ids_),
         &AllocInConstructorSystem::CalcFramePoseOutput);
     ...
   }

  private:
   void CalcFramePoseOutput(const MyContext& context,
                            geometry::FramePoseVector<T>* poses) const {
     DRAKE_DEMAND(poses->source_id() == source_id_);
     DRAKE_DEMAND(poses->size() == static_cast<int>(frame_ids_.size()));

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
 __Example 1: Known source id and frame count in constructor.__

 ```
 /// Definition of FramePoseVector deferred to define number of frames. However,
 /// it must be defined prior to call to `AllocateContext()`.
 template <typename T>
 class DeferredAllocationSystem : public LeafSystem<T> {
  public:
   DeferredAllocationSystem() {
     ...
     this->DeclareAbstractOutputPort(
         &DeferredAllocationSystem::AllocateFramePoseOutput,
         &DeferredAllocationSystem::CalcFramePoseOutput);
   }

  private:
   geometry::FramePoseVector<T> AllocateFramePoseOutput() const {
     // Assume that source_id_ has been assigned and the frames have been
     // registered.
     DRAKE_DEMAND(source_id_.is_valid());

     return geometry::FramePoseVector<T>(source_id_, frame_ids_);
   }

   void CalcFramePoseOutput(const MyContext& context,
                            geometry::FramePoseVector<T>* poses) const {
     DRAKE_DEMAND(poses->source_id() == source_id_);
     DRAKE_DEMAND(poses->size() == static_cast<int>(frame_ids_.size()));

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
 __Example 2: Deferred pose vector allocation.__

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
  FramePoseVector | FrameKinematicsVector<Isometry3<Scalar>> | Expression
  */
template <class KinematicsValue>
class FrameKinematicsVector {
  // Forward declaration.
  struct FlaggedValue;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameKinematicsVector)

  /** An object that represents the range of FrameId values in this class. It
   is used in range-based for loops to iterate through registered frames.  */
  using FrameIdRange = internal::MapKeyRange<FrameId, FlaggedValue>;

  // TODO(SeanCurtis-TRI) Find some API language that cautions users that this
  // result is not terribly useful on its own, but instead it will usually be
  // assigned into (using operator=) from another FrameKinematicsVector.
  /** Initializes the vector using an invalid SourceId with no frames .*/
  FrameKinematicsVector();

  /** Initializes the vector on the owned ids.
   @param source_id  The source id of the owning geometry source.
   @param ids        The set of *all* frames owned by this geometry source. All
                     of these ids must have values provided in the output port
                     calculation and _only_ these ids. SceneGraph will
                     validate the ids to confirm that they are all owned by
                     the source with the given `source_id`. */
  FrameKinematicsVector(SourceId source_id, const std::vector<FrameId>& ids);

  /** Initializes the vector to start setting kinematics values. */
  void clear();

  /** Sets the kinematics `value` for the frame indicated by the given `id`.
   There are various error conditions which will lead to an exception being
   thrown:

   1. the id provided is not one of the frame ids provided in the constructor.
   2. clear() hasn't been called.
   3. the value for a particular id is set multiple times between clear()
      invocations.

   If this isn't invoked for _every_ frame id provided at construction, it will
   lead to a subsequent exception when SceneGraph consumes the data. */
  void set_value(FrameId id, const KinematicsValue& value);

  SourceId source_id() const { return source_id_; }

  /** Returns the constructed size of this vector -- the number of FrameId
   values it was constructed with. */
  int size() const { return static_cast<int>(values_.size()); }

  /** Returns the value associated with the given `id`.
   @throws std::runtime_error if `id` is not in the specified set of ids.  */
  const KinematicsValue& value(FrameId id) const;

  /** Reports true if the given id is a member of this data. */
  bool has_id(FrameId id) const { return values_.count(id) > 0; }

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
  FrameIdRange frame_ids() const { return FrameIdRange(&values_); }

 private:
  // Utility function to help catch misuse.
  struct FlaggedValue {
    FlaggedValue() {
      detail::KinematicsValueInitializer<KinematicsValue>::Initialize(&value);
    }
    int64_t version{0};
    KinematicsValue value;
  };

  // The source id for the geometry source reporting data in this vector
  SourceId source_id_;

  // Mapping from frame id to its current pose (with a flag indicating
  // successful update).
  std::unordered_map<FrameId, FlaggedValue> values_;

  // The current version tag -- used to detect if values have been properly
  // updated.
  int64_t version_{0};
};

/** Class for communicating _pose_ information to SceneGraph for registered
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
