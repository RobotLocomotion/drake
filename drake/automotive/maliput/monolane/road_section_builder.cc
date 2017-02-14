#include "drake/automotive/maliput/monolane/road_section_builder.h"

#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace maliput {
namespace monolane {

template <typename T>
void RoadSectionBuilder<T>::AddArcPrimitive(const T& arc_length,
                                            const T& arc_radius,
                                            const ArcDirection& arc_direction,
                                            const EndpointZ& end_z) {
  DRAKE_DEMAND(arc_length < 2 * M_PI * arc_radius);
  const T& arc_angle = (arc_direction == kCCW) ? arc_length / arc_radius
                                               : -arc_length / arc_radius;
  const ArcOffset& arc{arc_radius, arc_angle};

  Connection* connection =
      b_->Connect(std::to_string(id_++), last_endpoint_, arc, end_z);
  last_endpoint_ = connection->end();
  if (is_reversed_) {
    connection->MakeReversed();
  }
}

template <typename T>
void RoadSectionBuilder<T>::AddArcPrimitive(const T& arc_length,
                                            const T& arc_radius,
                                            const ArcDirection& arc_direction) {
  AddArcPrimitive(arc_length, arc_radius, arc_direction, flat_z_);
}

template <typename T>
void RoadSectionBuilder<T>::AddLinearPrimitive(const T& length,
                                               const EndpointZ& end_z) {
  Connection* connection =
      b_->Connect(std::to_string(id_++), last_endpoint_, length, end_z);
  last_endpoint_ = connection->end();
  if (is_reversed_) {
    connection->MakeReversed();
  }
}

template <typename T>
void RoadSectionBuilder<T>::AddLinearPrimitive(const T& length) {
  AddLinearPrimitive(length, flat_z_);
}

template class RoadSectionBuilder<double>;

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
