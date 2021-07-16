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
 as the operand of various geometry operations (e.g., CollisionFilterDeclaration
 and CollisionFilterManager::Apply(). If the _operation_ has a particular
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

  explicit GeometrySet(GeometryId geometry_id) {
    geometry_ids_.insert(geometry_id);
  }

  explicit GeometrySet(FrameId frame_id) { frame_ids_.insert(frame_id); }

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
  explicit GeometrySet(std::initializer_list<GeometryId> geometry_ids,
                       const Container& frame_ids) {
    Add(geometry_ids);
    Add(frame_ids);
  }

  template <typename Container>
  explicit GeometrySet(const Container& geometry_ids,
                       std::initializer_list<FrameId> frame_ids) {
    Add(geometry_ids);
    Add(frame_ids);
  }

  explicit GeometrySet(std::initializer_list<GeometryId> geometry_ids,
                       std::initializer_list<FrameId> frame_ids) {
    Add(geometry_ids);
    Add(frame_ids);
  }

  template <
      typename ContainerG,
      typename ContainerF,
      typename = typename std::enable_if_t<
          std::is_same_v<typename ContainerG::value_type, GeometryId> &&
          std::is_same_v<typename ContainerF::value_type, FrameId>>
      >
  GeometrySet(const ContainerG& geometry_ids, const ContainerF& frame_ids) {
    Add(geometry_ids, frame_ids);
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

   std::vector<FrameId> frame_ids{f_2, f_3, f_4};
   group.Add(frame_ids);
   std::vector<GeometryId> geometry_ids{g_2, g_3, g_4};
   group.Add(geometry_ids);

   // This is valid, but redundant; the ids in those vectors have already been
   // added.
   group.Add(geometry_ids, frame_ids);

   // Mismatched iterable types.
   std::set<FrameId> frame_set{f_5, f_6, f_7};
   group.Add({g_7, g_8}, frame_set);
   ```  */
  //@{

  void Add(GeometryId geometry_id) { geometry_ids_.insert(geometry_id); }

  void Add(FrameId frame_id) { frame_ids_.insert(frame_id); }

  template <typename Container>
  typename std::enable_if_t<
      std::is_same_v<typename Container::value_type, GeometryId>>
  Add(const Container& geometry_ids) {
    geometry_ids_.insert(geometry_ids.begin(), geometry_ids.end());
  }

  template <typename Container>
  typename std::enable_if_t<
      std::is_same_v<typename Container::value_type, FrameId>>
  Add(const Container& frame_ids) {
    frame_ids_.insert(frame_ids.begin(), frame_ids.end());
  }

  void Add(std::initializer_list<FrameId> frame_ids) {
    frame_ids_.insert(frame_ids.begin(), frame_ids.end());
  }

  void Add(std::initializer_list<GeometryId> geometry_ids) {
    geometry_ids_.insert(geometry_ids.begin(), geometry_ids.end());
  }

  template <typename ContainerG, typename ContainerF>
  typename std::enable_if_t<
      std::is_same_v<typename ContainerG::value_type, GeometryId> &&
      std::is_same_v<typename ContainerF::value_type, FrameId>>
  Add(const ContainerG& geometry_ids, const ContainerF& frame_ids) {
    Add(geometry_ids);
    Add(frame_ids);
  }

  template <typename ContainerF>
  typename std::enable_if_t<
      std::is_same_v<typename ContainerF::value_type, FrameId>>
  Add(
      std::initializer_list<GeometryId> geometry_ids,
      const ContainerF& frame_ids) {
    Add(geometry_ids);
    Add(frame_ids);
  }

  template <typename ContainerG>
  typename std::enable_if_t<
      std::is_same_v<typename ContainerG::value_type, GeometryId>>
  Add(
      const ContainerG& geometry_ids,
      std::initializer_list<FrameId> frame_ids) {
    Add(geometry_ids);
    Add(frame_ids);
  }

  void Add(const GeometrySet& other) {
    frame_ids_.insert(other.frame_ids_.begin(), other.frame_ids_.end());
    geometry_ids_.insert(
        other.geometry_ids_.begin(), other.geometry_ids_.end());
  }

  //@}

 private:
  // Provide access to the two entities that need access to the set's internals.
  friend class GeometrySetTester;
  template <typename>
  friend class GeometryState;

  // Returns the frame ids in the set.
  const std::unordered_set<FrameId>& frames() const { return frame_ids_; }

  // Reports the number of frames in the set.
  int num_frames() const { return static_cast<int>(frame_ids_.size()); }

  // Returns the geometry ids in the set -- these are only the geometry ids
  // explicitly added to the set and _not_ those implied by added frames.
  const std::unordered_set<GeometryId>& geometries() const {
    return geometry_ids_;
  }

  // Reports the number of geometries _explicitly_ in the set. It does _not_
  // count the geometries that belong to the added frames.
  int num_geometries() const {
    return static_cast<int>(geometry_ids_.size());
  }

  // Reports if the given `frame_id` has been added to the group.
  bool contains(FrameId frame_id) const {
    return frame_ids_.count(frame_id) > 0;
  }

  // Reports if the given `geometry_id` has been *explicitly* added to the
  // group. It will *not* capture geometry ids affixed to added frames.
  bool contains(GeometryId geometry_id) const {
    return geometry_ids_.count(geometry_id) > 0;
  }

  std::unordered_set<FrameId> frame_ids_;
  std::unordered_set<GeometryId> geometry_ids_;
};

}  // namespace geometry
}  // namespace drake
