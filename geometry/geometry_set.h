#pragma once

#include <initializer_list>
#include <unordered_set>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

template <typename T>
class GeometryState;

/** The %GeometrySet, as its name implies, is a convenience class for defining a
 set of geometries. What makes it unique from a simple `std::set<GeometryId>`
 instance is that membership doesn't require explicit GeometryId enumeration;
 GeometryId values can be added to the set by adding the `FrameId` for the
 frame to which the geometries are rigidly affixed.

 This class does no validation; it is a simple collection. Ultimately, it serves
 as the operand of SceneGraph operations (e.g.,
 SceneGraph::ExcludeCollisionsWithin()). If the _operation_ has a particular
 prerequisite on the members of a %GeometrySet, it is the operation's
 responsibility to enforce that requirement.

 More formally, the SceneGraph consists of a set of geometries, each associated
 with a unique identifier. As such, we can consider the set of all identifiers
 `SG = {g₀, g₁, ..., gₙ}` that belong to a SceneGraph. A %GeometrySet should
 represent a subset of those identifiers, `Gₛ ⊆ SG`. The convenience of the
 %GeometrySet class is _how_ the subset is defined. Given a set of frame ids
 `F = {f₀, f₁, ..., fₙ}` and geometry ids `G = {g₀, g₁, ..., gₘ}`,
 `Gₛ = G ⋃ geometry(f₀) ⋃ ... ⋃ geometry(fₙ)` (where `geometry(f)` is the set of
 geometries rigidly affixed to frame f).  */
class GeometrySet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometrySet)

  GeometrySet() = default;

  /** @name    Explicit constructors

   Various workflows may arise for operating on %GeometrySet instances, e.g.:

   ```
   // Perform operation on previously existing collection of frame ids.
   std::vector<FrameId> my_ids{...};  // Previously-defined vector of ids.
   GeometrySet geometry_set;
   geometry_set.Add(my_ids);
   UnaryOperation(geometry_set);
   ```

   or

   ```
   // Perform operation between two frames.
   GeometrySet set1;
   set1.add(frame1);
   GeometrySet set2;
   set2.add(frame2);
   BinaryOperation(set1, set2);
   ```

   This set of constructors allow on-the-fly construction at the call site
   to create temporary instances when the group membership is a single id or a
   previously existing id. By doing so, the above cases become:

   ```
   // Perform operation on previously existing collection of frame ids.
   std::vector<FrameId> my_ids{...};  // Previously-defined vector of ids.
   UnaryOperation(GeometrySet(my_ids));

   // Perform operation between two frames.
   BinaryOperation(GeometrySet(frame1), GeometrySet(frame2));
   ```

   The following are all valid constructions -- this is _not_ an exhaustive list
   but a representative sampling:

   ```
   // Assume that g_* and f_* are valid GeometryId and FrameId instances,
   // respectively.
   std::vector<GeometryId> g_vector{g_0, g_1, g_2};
   std::set<GeometryId> g_set{g_3, g_4, g_5};
   std::unordered_set<FrameId> f_set{f_0, f_1};
   auto f_list = {f_2, f_3, f_4};

   GeometrySet(g_0);
   GeometrySet(f_0);
   GeometrySet({g_0, g_1});
   GeometrySet({f_0, f_1});
   GeometrySet(g_vector, f_set);
   GeometrySet(g_set, f_list);
   // Note: construction on values of both geometry and frame identifiers
   // requires passing in "collections" of ids; the initializer list is the
   // simplest collection that serves the purpose. When both are provided,
   // GeometryId always comes before FrameId.
   GeometrySet({g_0}, {f_1});
   GeometrySet({g_0}, f_set);
   // etc.
   ```
   */
  //@{

  // TODO(SeanCurtis-TRI): The call sites would become even simpler if these
  // constructors were *implicit*. E.g.,:
  //    UnaryFilterOp(my_ids);
  //    BinaryFilterOp(frame1, frame2);
  // Determine if this convenience justifies getting a style guide exception.

  explicit GeometrySet(GeometryId id) { geometries_.insert(id); }

  explicit GeometrySet(FrameId id) { frames_.insert(id); }

  template <typename Container>
  explicit GeometrySet(const Container& ids) {
    Add(ids);
  }

  // NOTE: initializer lists cannot be inferred by ADL, so they must be
  // explicitly enumerated.
  template <typename Id>
  explicit GeometrySet(std::initializer_list<Id> id_list) {
    Add(id_list);
  }

  template <typename Container>
  explicit GeometrySet(std::initializer_list<GeometryId> geometries,
                       const Container& frames) {
    Add(geometries);
    Add(frames);
  }

  template <typename Container>
  explicit GeometrySet(const Container& geometries,
                       std::initializer_list<FrameId> frames) {
    Add(geometries);
    Add(frames);
  }

  explicit GeometrySet(std::initializer_list<GeometryId> geometries,
                       std::initializer_list<FrameId> frames) {
    Add(geometries);
    Add(frames);
  }

  //@}

  /** @name    Methods for adding to the set

   The interface for adding geometries to the set is simply an overload of the
   Add() method. For maximum flexibility, the Add method can take:

     - a single geometry id
     - a single frame id
     - an iterable object containing geometry ids
     - an iterable object containing frame ids
     - two iterable objects, the first containing geometry ids, the second
       containing frame ids.
     - another %GeometrySet instance.

   NOTE: the iterable objects don't have to be the same type. The "iterable"
   can also be an initializer list. All of the following invocations are valid
   (this isn't an exhaustive list, but a representative set):

   ```
   // Assuming that f_* are valid FrameId instances and g_* are valid GeometryId
   // instances.
   GeometrySet group;
   group.Add(f_1);
   group.Add(g_1);

   std::vector<FrameId> frames{f_2, f_3, f_4};
   group.Add(frames);
   std::vector<GeometryId> geometries{g_2, g_3, g_4};
   group.Add(geometries);

   // This is valid, but redundant; the the ids in those vectors have already
   // been added.
   group.Add(geometries, frames);

   // Mismatched iterable types.
   std::set<FrameId> frame_set{f_5, f_6, f_7};
   group.Add({g_7, g_8}, frame_set);
   ```  */
  //@{

  void Add(GeometryId geometry_id) { geometries_.insert(geometry_id); }

  void Add(FrameId frame_id) { frames_.insert(frame_id); }

  template <typename Container>
  typename std::enable_if<
      std::is_same<typename Container::value_type, GeometryId>::value>::type
  Add(const Container& geometries) {
    geometries_.insert(geometries.begin(), geometries.end());
  }

  template <typename Container>
  typename std::enable_if<
      std::is_same<typename Container::value_type, FrameId>::value>::type
  Add(const Container& frames) {
    frames_.insert(frames.begin(), frames.end());
  }

  void Add(std::initializer_list<FrameId> frames) {
    frames_.insert(frames.begin(), frames.end());
  }

  void Add(std::initializer_list<GeometryId> geometries) {
    geometries_.insert(geometries.begin(), geometries.end());
  }

  template <typename ContainerG, typename ContainerF>
  typename std::enable_if<
      std::is_same<typename ContainerG::value_type, GeometryId>::value &&
      std::is_same<typename ContainerF::value_type, FrameId>::value>::type
  Add(const ContainerG& geometries, const ContainerF& frames) {
    Add(geometries);
    Add(frames);
  }

  template <typename ContainerF>
  typename std::enable_if<
      std::is_same<typename ContainerF::value_type, FrameId>::value>::type
  Add(std::initializer_list<GeometryId> geometries, const ContainerF& frames) {
    Add(geometries);
    Add(frames);
  }

  template <typename ContainerG>
  typename std::enable_if<
      std::is_same<typename ContainerG::value_type, GeometryId>::value>::type
  Add(const ContainerG& geometries, std::initializer_list<FrameId> frames) {
    Add(geometries);
    Add(frames);
  }

  void Add(const GeometrySet& other) {
    frames_.insert(other.frames_.begin(), other.frames_.end());
    geometries_.insert(other.geometries_.begin(), other.geometries_.end());
  }

  //@}

 private:
  // Provide access to the two entities that need access to the set's internals.
  friend class GeometrySetTester;
  template <typename>
  friend class GeometryState;

  // Returns the frame ids in the set.
  const std::unordered_set<FrameId> frames() const { return frames_; }

  // Reports the number of frames in the set.
  int num_frames() const { return static_cast<int>(frames_.size()); }

  // Returns the geometry ids in the set -- these are only the geometry ids
  // explicitly added to the set and _not_ those implied by added frames.
  const std::unordered_set<GeometryId> geometries() const {
    return geometries_;
  }

  // Reports the number of geometries _explicitly_ in the set. It does _not_
  // count the geometries that belong to the added frames.
  int num_geometries() const {
    return static_cast<int>(geometries_.size());
  }

  // Reports if the given `frame_id` has been added to the group.
  bool contains(FrameId frame_id) const {
    return frames_.count(frame_id) > 0;
  }

  // Reports if the given `geometry_id` has been *explicitly* added to the
  // group. It will *not* capture geometry ids affixed to added frames.
  bool contains(GeometryId geometry_id) const {
    return geometries_.count(geometry_id) > 0;
  }

  std::unordered_set<FrameId> frames_;
  std::unordered_set<GeometryId> geometries_;
};

}  // namespace geometry
}  // namespace drake
