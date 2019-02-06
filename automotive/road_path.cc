#include "drake/automotive/road_path.h"

#include <stdexcept>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::RoadGeometry;
using trajectories::PiecewisePolynomial;

template <typename T>
RoadPath<T>::RoadPath(const LaneDirection& initial_lane_direction,
                      const T& step_size, int num_breaks)
    : path_(MakePiecewisePolynomial(initial_lane_direction, step_size,
                                    num_breaks)),
      path_prime_(path_.derivative(1 /* 1st derivative */)),
      path_double_prime_(path_.derivative(2 /* 2nd derivative */)) {}

template <typename T>
RoadPath<T>::~RoadPath() {}

template <typename T>
const PiecewisePolynomial<T>& RoadPath<T>::get_path() const {
  return path_;
}

template <typename T>
const T RoadPath<T>::GetClosestPathPosition(const Vector3<T>& geo_pos,
                                            const T& s_guess) const {
  unused(geo_pos, s_guess);
  throw std::runtime_error(
      "RoadPath<T>::GetClosestPathPosition is not implemented");
}

template <typename T>
const PiecewisePolynomial<T> RoadPath<T>::MakePiecewisePolynomial(
    const LaneDirection& initial_lane_direction, const T& step_size,
    int num_breaks) const {
  std::vector<T> s_breaks{};
  std::vector<MatrixX<T>> geo_knots(num_breaks, MatrixX<T>::Zero(3, 1));

  LaneDirection ld = initial_lane_direction;
  T s_lane{cond(ld.with_s, T(0.), T(ld.lane->length()))};
  T s_break{0.};

  // Loop over all the breaks and extract the knot points.
  for (int i = 0; i < num_breaks - 1; ++i) {
    s_breaks.emplace_back(s_break);
    s_break += T(step_size);

    GeoPosition geo_pos =
        ld.lane->ToGeoPosition({s_lane /* s */, 0. /* r */, 0. /* h */});
    geo_knots[i] << T(geo_pos.x()), T(geo_pos.y()), T(geo_pos.z());

    // Take a step.
    if (ld.with_s) {
      s_lane += T(step_size);
    } else {
      s_lane -= T(step_size);
    }

    // Compute the distance in excess of the lane boundary by taking this step.
    const T out_distance{
        cond(ld.with_s, T(s_lane - ld.lane->length()), T(-s_lane))};
    if (out_distance >= 0.) {
      const LaneEndSet* lane_end_set{
          ld.with_s ? ld.lane->GetOngoingBranches(LaneEnd::kFinish)
                    : ld.lane->GetOngoingBranches(LaneEnd::kStart)};
      if (lane_end_set->size() == 0) {  // There are no more ongoing lanes.
        // If needed, add another knot point to make up the remaining distance.
        if (out_distance != 0.) {
          s_breaks.emplace_back(s_break + T(step_size - out_distance));
          s_lane = cond(ld.with_s, T(ld.lane->length()), T(0.));
          geo_pos =
              ld.lane->ToGeoPosition({s_lane /* s */, 0. /* r */, 0. /* h */});
          geo_knots[i + 1] << T(geo_pos.x()), T(geo_pos.y()), T(geo_pos.z());
        }
        break;
      }

      // Always choose the first lane in the set as the new lane.
      ld.lane = lane_end_set->get(0).lane;
      ld.with_s = (lane_end_set->get(0).end == LaneEnd::kStart) ? true : false;

      // Correct for the distance overshoot.
      s_lane =
          cond(ld.with_s, out_distance, T(ld.lane->length()) - out_distance);
    }
  }
  // Resize the vector of knot points, if necessary.
  geo_knots.resize(s_breaks.size());

  // Create the resulting piecewise polynomial.
  return PiecewisePolynomial<T>::Cubic(s_breaks, geo_knots);
}

template class RoadPath<double>;

}  // namespace automotive
}  // namespace drake
