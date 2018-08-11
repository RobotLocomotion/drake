#pragma once

#include <functional>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/geometry_base/segment.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace geometry_base {

class RoadGeometryBadge;

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
  /// @throws if `segment` is empty.
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
  // This may only be called, once, by a RoadGeometry.
  void AttachToRoadGeometry(
      const RoadGeometryBadge&,
      const api::RoadGeometry* road_geometry,
      const std::function<void(const api::Segment*)>& segment_indexing_callback,
      const std::function<void(const api::Lane*)>& lane_indexing_callback);
#endif  // DRAKE_DOXYGEN_CXX

 private:
  void AddSegmentPrivate(std::unique_ptr<Segment> segment);

  const api::JunctionId do_id() const override { return id_; }

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


#ifndef DRAKE_DOXYGEN_CXX
class JunctionBadge { friend class Junction; };
#endif  // DRAKE_DOXYGEN_CXX

}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
