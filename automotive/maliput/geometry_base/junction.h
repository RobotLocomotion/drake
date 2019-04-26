#pragma once

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/geometry_base/passkey.h"
#include "drake/automotive/maliput/geometry_base/segment.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace geometry_base {

class RoadGeometry;

/// geometry_base's implementation of api::Junction.
class Junction : public api::Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Junction);

  /// Constructs a Junction.
  ///
  /// @param id the ID of the Junction
  ///
  /// The Junction is not fully initialized until it is added to a
  /// RoadGeometry.
  explicit Junction(const api::JunctionId& id) : id_(id) {}

  /// Adds @p segment to this Junction.
  ///
  /// This Junction will take ownership of `segment` and will be assigned
  /// as its parent.
  ///
  /// @returns `segment`'s raw pointer.
  ///
  /// @tparam T must be derived from geometry_base::Segment.
  ///
  /// @throws std::exception if `segment` is empty.
  template <class T>
  T* AddSegment(std::unique_ptr<T> segment) {
    static_assert(std::is_base_of<Segment, T>::value,
                  "T is not derived from geometry_base::Segment");
    T* const raw_pointer = segment.get();
    AddSegmentPrivate(std::move(segment));
    return raw_pointer;
  }

  ~Junction() override = default;


#ifndef DRAKE_DOXYGEN_CXX
  // Notifies Junction of its parent RoadGeometry.
  // This may only be called, once, by a RoadGeometry.
  //
  // @param road_geometry  the parent RoadGeometry
  // @param segment_indexing_callback  function to be called on every Segment
  //                                   which is attached to this Junction,
  //                                   both now and in the future
  // @param lane_indexing_callback  function to be called on every Lane
  //                                which is attached to this Junction (via
  //                                a Segment), both now and in the future
  //
  // @pre `road_geometry` is non-null.
  // @pre `segment_indexing_callback` is non-empty.
  // @pre `lane_indexing_callback` is non-empty.
  // @pre Parent RoadGeometry and the callbacks have not already been set.
  void AttachToRoadGeometry(
      Passkey<RoadGeometry>,
      const api::RoadGeometry* road_geometry,
      const std::function<void(const api::Segment*)>& segment_indexing_callback,
      const std::function<void(const api::Lane*)>& lane_indexing_callback);
#endif  // DRAKE_DOXYGEN_CXX

 private:
  // The non-template implementation of AddSegment<T>()
  void AddSegmentPrivate(std::unique_ptr<Segment> segment);

  api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  int do_num_segments() const override { return segments_.size(); }

  const api::Segment* do_segment(int index) const override {
    return segments_.at(index).get();
  }

  const api::JunctionId id_;
  const api::RoadGeometry* road_geometry_{};
  std::function<void(const api::Segment*)> segment_indexing_callback_;
  std::function<void(const api::Lane*)> lane_indexing_callback_;
  std::vector<std::unique_ptr<Segment>> segments_;
};


}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
