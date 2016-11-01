#include "drake/automotive/create_trajectory_params.h"

#include <algorithm>
#include <vector>

namespace drake {
namespace automotive {

namespace {
// A figure-eight.  One loop has a radius of @p radius - @p inset,
// the other loop has a radius of @p radius + @p inset.
Curve2<double> MakeCurve(double radius, double inset) {
  // TODO(jwnimmer-tri) This function will be rewritten once we have
  // proper splines.  Don't try too hard to understand it.  Run the
  // demo to see it first, and only then try to understand the code.

  typedef Curve2<double>::Point2 Point2d;
  std::vector<Point2d> waypoints;

  // Start (0, +i).
  // Straight right to (+r, +i).
  // Loop around (+i, +r).
  // Straight back to (+i, 0).
  waypoints.push_back({0.0, inset});
  for (int theta_deg = -90; theta_deg <= 180; ++theta_deg) {
    const Point2d center{radius, radius};
    const double theta = theta_deg * M_PI / 180.0;
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius - inset)));
  }
  waypoints.push_back({inset, 0.0});

  // Start (+i, 0).
  // Straight down to (+i, -r).
  // Loop around (-r, +i).
  // Straight back to start (implicitly via segment to waypoints[0]).
  for (int theta_deg = 0; theta_deg >= -270; --theta_deg) {
    const Point2d center{-radius, -radius};
    const double theta = theta_deg * M_PI / 180.0;
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius + inset)));
  }

  // Many copies.
  const int kNumCopies = 100;
  std::vector<Point2d> looped_waypoints;
  for (int copies = 0; copies < kNumCopies; ++copies) {
    std::copy(waypoints.begin(), waypoints.end(),
              std::back_inserter(looped_waypoints));
  }
  looped_waypoints.push_back(waypoints.front());

  return Curve2<double>(looped_waypoints);
}
}  // anonymous namespace

DRAKE_EXPORT
std::tuple<Curve2<double>, double, double> CreateTrajectoryParams(int index) {
  // The possible curves to trace (lanes).
  static const std::vector<Curve2<double>> curves{
    MakeCurve(40.0, 0.0),  // BR
    MakeCurve(40.0, 4.0),  // BR
    MakeCurve(40.0, 8.0),
  };

  // Magic car placement to make a good visual demo.
  const auto& curve = curves[index % curves.size()];
  const double start_time = (index / curves.size()) * 0.8;
  const double kSpeed = 8.0;
  return std::make_tuple(curve, kSpeed, start_time);
}

}  // namespace automotive
}  // namespace drake
