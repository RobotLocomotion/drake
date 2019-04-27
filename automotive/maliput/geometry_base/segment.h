#pragma once

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/geometry_base/lane.h"
#include "drake/automotive/maliput/geometry_base/passkey.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace geometry_base {

class Junction;

/// geometry_base's implementation of api::Segment.
class Segment : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment);

  /// Constructs a new Segment.
  ///
  /// @param id Segment's ID.
  ///
  /// The Segment is not fully initialized until it is added to a Junction.
  explicit Segment(const api::SegmentId& id) : id_(id) {}


  /// Adds @p lane to this Segment.
  ///
  /// This Segment will take ownership of `lane` and will be assigned
  /// as its parent.  The index of `lane` will also be assigned, in
  /// order of addition.  (The first Lane added to this Segment, by
  /// the first call to AddLane(), will be assigned index 0.)
  ///
  /// Note that the maliput API requires that lanes are indexed in a Segment
  /// in right-to-left order, thus `AddLane()` should be called on lanes
  /// in right-to-left order.
  ///
  /// @returns `lane`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::Lane.
  ///
  /// @throws std::exception if `lane` is empty.
  // TODO(maddog@tri.global)  Consider enforcing right-to-left order.  (The
  //                          downside is that it makes these classes less
  //                          useful for creating aberrant test cases.)
  template <class T>
  T* AddLane(std::unique_ptr<T> lane) {
    static_assert(std::is_base_of<Lane, T>::value,
                  "T is not derived from geometry_base::Lane");
    T* const raw_pointer = lane.get();
    AddLanePrivate(std::move(lane));
    return raw_pointer;
  }


#ifndef DRAKE_DOXYGEN_CXX
  // Notifies Segment of its parent Junction.
  // This may only be called, once, by a Junction.
  //
  // @param junction  the parent Junction
  //
  // @pre `junction` is non-null.
  // @pre Parent Junction has not already been set.
  void AttachToJunction(Passkey<Junction>, const api::Junction* junction);

  // Sets the lane indexing callback.
  // This may only be called, once, by a Junction.
  //
  // @param callback  function to be called on every Lane which is attached
  //                  to this Segment, both now and in the future
  //
  // @pre `callback` is non-empty.
  // @pre The lane indexing callback has not already been set.
  void SetLaneIndexingCallback(
      Passkey<Junction>,
      const std::function<void(const api::Lane*)>& callback);
#endif  // DRAKE_DOXYGEN_CXX

  ~Segment() override = default;

 private:
  // The non-template implementation of AddLane<T>()
  void AddLanePrivate(std::unique_ptr<Lane> lane);

  api::SegmentId do_id() const override { return id_; }

  const api::Junction* do_junction() const override;

  int do_num_lanes() const override { return lanes_.size(); }

  const api::Lane* do_lane(int index) const override;

  const api::SegmentId id_;
  const api::Junction* junction_{};
  std::function<void(const api::Lane*)> lane_indexing_callback_;
  std::vector<std::unique_ptr<Lane>> lanes_;
};


}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
