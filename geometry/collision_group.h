#pragma once

#include <unordered_set>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

/** This class serves as the basic unit for filtering collisions. The SceneGraph
 consists of a set of geometry identifiers G = {g₀, g₁, ..., gₙ}. Collisions
 comprise of _pairs_ of geometries (e.g., (gᵢ, gⱼ)). The %CollisionGroup serves
 as the basis for defining geometry pairs that _won't_ be considered during
 collision detection. Geometries are added to the set and then a filtering
 operation is applied to that set. Some examples of filter operations:

   - For set Gₛ, disallow _self collision_ in the set, i.e., pairs (gᵢ, gⱼ),
     ∀ gᵢ, gⱼ ∈ Gₛ, are filtered out of consideration.
   - For sets Gₛ and Gₜ, pairs (gₛ, gₜ), ∀ gₛ ∈ Gₛ and gₜ ∈ Gₜ, are filtered out
     of consideration.

 All dynamic geometries are rigidly affixed to a particular frame. As a
 convenience the set of geometries affixed to frame f can be compactly added to
 the set by adding the f's frame identifier. Ultimately, that is equivalent to
 enumerating the affixed geometries.

 Concretely, a %CollisionGroup is the union of a set of frame ids
 `F = {f₀, f₁, ..., fₙ}` and geometry ids `G = {g₀, g₁, ..., gₘ}`. That union,
 implicitly defines the set of geometry ids
 `𝒢 = G ⋃ geometry(f₀) ⋃ ... ⋃ geometry(fₙ)` (where `geometry(f)` is the set of
 geometries rigidly affixed to frame f). `𝒢` is the full set of geometries
 which belong to the  %CollisionGroup. */
class CollisionGroup {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionGroup)

  CollisionGroup() = default;

  /** @name    Explicit constructors

   The interface for filter operations will all operate on %CollisionGroup
   parameters. Leading to various workflows:

   ```
   // Add a pre-existing set of ids to the group.
   std::vector<FrameId> my_ids{...};  // Previously-defined vector of ids.
   CollisionGroup group;
   group.Add(my_ids);
   UnaryFilterOp(group);
   ```

   or

   ```
   // Perform operation between two frames.
   CollisionGroup group1;
   group1.add(frame1);
   CollisionGroup group2;
   group2.add(frame2);
   BinaryFilterOp(group1, group2);
   ```

   This set of constructors allow on-the-fly construction at the call site
   to create temporaries when the group membership is a single id or a
   previously existing id. By doing so, the above cases become:

   ```
   // Add a pre-existing set of ids to the group.
   std::vector<FrameId> my_ids{...};  // Previously-defined vector of ids.
   UnaryFilterOp(CollisionGroup(my_ids));

   // Perform operation between two frames.
   BinaryFilterOp(CollisionGroup(frame1), CollisionGroup(frame2));
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

   CollisionGroup(g_0);
   CollisionGroup(f_0);
   CollisionGroup({g_0, g_1});
   CollisionGroup({f_0, f_1});
   CollisionGroup(g_vector, f_set);
   CollisionGroup(g_set, f_list);
   // Note: construction on values of both geometry and frame identifiers
   // requires passing in "collections" of ids; the initializer list is the
   // simplest collection. When both are provided, GeometryId always comes
   // before FrameId.
   CollisionGroup({g_0}, {f_1});
   CollisionGroup({g_0}, f_set);
   // etc.
   */
  //@{

  // TODO(SeanCurtis-TRI): The call sites would become even simpler if these
  // constructors were *implicit*. E.g.,:
  //    UnaryFilterOp(my_ids);
  //    BinaryFilterOp(frame1, frame2);
  // Determine if this convenience justifies getting a style guide exception.

  explicit CollisionGroup(GeometryId id) { geometries_.insert(id); }

  explicit CollisionGroup(FrameId id) { frames_.insert(id); }

  template <typename Container>
  explicit CollisionGroup(const Container& ids) {
    Add(ids);
  }

  // NOTE: initializer lists cannot be inferred by ADL, so they must be
  // explicitly enumerated.
  template <typename Id>
  explicit CollisionGroup(std::initializer_list<Id> id_list) {
    Add(id_list);
  }

  template <typename Container>
  explicit CollisionGroup(std::initializer_list<GeometryId> geometries,
                          const Container& frames) {
    Add(geometries);
    Add(frames);
  }

  template <typename Container>
  explicit CollisionGroup(const Container& geometries,
                          std::initializer_list<FrameId> frames) {
    Add(geometries);
    Add(frames);
  }

  //@}

  /** @name    Methods for adding to the group

   The interface for adding geometries to the group is simply an overload of the
   Add() method. For maximum flexibility, the Add method can take:
     - a single geometry id
     - a single frame id
     - an iterable object containing geometry ids
     - an iterable object containing frame ids
     - two iterable objects, the first containing geometry ids, the second
       containing frame ids.

   NOTE: the iterable objects don't have to be the same type. The "iterable"
   can also be an initializer list. All of the following invocations are valid
   (this isn't an exhaustive list, but a representative set):

   ```
   // Assuming that f_* are valid FrameId instances and g_* are valid GeometryId
   // instances.
   CollisionGroup group;
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

  //@}

  /** Returns the frame ids in the group. */
  const std::unordered_set<FrameId> frames() const { return frames_; }

  /** Reports the number of frames added to the group. */
  int num_frames() const { return static_cast<int>(frames_.size()); }

  /** returns the geometry ids in the group. */
  const std::unordered_set<GeometryId> geometries() const {
    return geometries_;
  }

  /** Reports the number of geometries _explicitly_ added to the group. It does
   _not_ count the geometries that belong to the added frames.  */
  int num_geometries() const { return static_cast<int>(geometries_.size()); }

  /** Reports if the given `frame_id` has been added to the group. */
  bool contains(FrameId frame_id) const { return frames_.count(frame_id) > 0; }

  /** Reports if the given `geometry_id` has been added to the group. */
  bool contains(GeometryId geometry_id) const {
    return geometries_.count(geometry_id) > 0;
  }

 private:
  std::unordered_set<FrameId> frames_;
  std::unordered_set<GeometryId> geometries_;
};

}  // namespace geometry
}  // namespace drake
