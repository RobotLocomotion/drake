#include "drake/automotive/maliput/multilane/connection.h"

#include "drake/automotive/maliput/multilane/arc_road_curve.h"
#include "drake/automotive/maliput/multilane/line_road_curve.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace maliput {
namespace multilane {

std::ostream& operator<<(std::ostream& out, const EndpointXy& endpoint_xy) {
  return out << "(x = " << endpoint_xy.x() << ", y = " << endpoint_xy.y()
             << ", heading = " << endpoint_xy.heading() << ")";
}

std::ostream& operator<<(std::ostream& out, const EndpointZ& endpoint_z) {
  return out << "(z = " << endpoint_z.z() << ", z_dot = " << endpoint_z.z_dot()
             << ", theta = " << endpoint_z.theta()
             << ", theta_dot = " << endpoint_z.theta_dot() << ")";
}

std::ostream& operator<<(std::ostream& out, const Endpoint& endpoint) {
  return out << "(xy: " << endpoint.xy() << ", z: " << endpoint.z() << ")";
}

Connection::Connection(const std::string& id, const Endpoint& start,
                       const EndpointZ& end_z, int num_lanes, double r0,
                       double lane_width, double left_shoulder,
                       double right_shoulder, double line_length)
    : type_(kLine),
      id_(id),
      start_(start),
      num_lanes_(num_lanes),
      r0_(r0),
      lane_width_(lane_width),
      left_shoulder_(left_shoulder),
      right_shoulder_(right_shoulder),
      r_min_(r0 - lane_width / 2. - right_shoulder),
      r_max_(r0 + lane_width * (static_cast<double>(num_lanes - 1) + 0.5) +
             left_shoulder),
      line_length_(line_length) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(line_length_ > 0.);
  // Computes end Endpoint and RoadCurve.
  end_ = {{start_.xy().x() + line_length_ * std::cos(start_.xy().heading()),
           start_.xy().y() + line_length_ * std::sin(start_.xy().heading()),
           start_.xy().heading()},
          end_z};
  road_curve_ = CreateRoadCurve();
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, {0., 0.}));
}

Connection::Connection(const std::string& id, const Endpoint& start,
                       const EndpointZ& end_z, int num_lanes, double r0,
                       double lane_width, double left_shoulder,
                       double right_shoulder, const ArcOffset& arc_offset)
    : type_(kArc),
      id_(id),
      start_(start),
      num_lanes_(num_lanes),
      r0_(r0),
      lane_width_(lane_width),
      left_shoulder_(left_shoulder),
      right_shoulder_(right_shoulder),
      r_min_(r0 - lane_width / 2. - right_shoulder),
      r_max_(r0 + lane_width * (static_cast<double>(num_lanes - 1) + 0.5) +
             left_shoulder),
      radius_(arc_offset.radius()),
      d_theta_(arc_offset.d_theta()) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(radius_ > 0);
  // Fills arc related parameters, computes end Endpoint and creates the
  // RoadCurve.
  theta0_ = start_.xy().heading() - std::copysign(M_PI / 2., d_theta_);
  cx_ = start.xy().x() - (radius_ * std::cos(theta0_));
  cy_ = start.xy().y() - (radius_ * std::sin(theta0_));
  const double theta1 = theta0_ + d_theta_;
  end_ = {{cx_ + radius_ * std::cos(theta1), cy_ + radius_ * std::sin(theta1),
           start_.xy().heading() + d_theta_},
          end_z};
  road_curve_ = CreateRoadCurve();
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, {0., 0.}));
}

Endpoint Connection::LaneStart(int lane_index) const {
  DRAKE_DEMAND(lane_index >= 0 && lane_index < num_lanes_);
  const double r = lane_offset(lane_index);
  const Vector3<double> position = road_curve_->W_of_prh(0., r, 0.);
  const Rot3 rotation = road_curve_->Orientation(0., r, 0.);
  const Vector3<double> w_prime =
      road_curve_->W_prime_of_prh(0., r, 0., road_curve_->Rabg_of_p(0.),
                                  road_curve_->elevation().f_dot_p(0.));
  return Endpoint(EndpointXy(position[0], position[1], rotation.yaw()),
                  EndpointZ(position[2], w_prime.z(), start_.z().theta(),
                            start_.z().theta_dot()));
}

Endpoint Connection::LaneEnd(int lane_index) const {
  DRAKE_DEMAND(lane_index >= 0 && lane_index < num_lanes_);
  const double r = lane_offset(lane_index);
  const Vector3<double> position = road_curve_->W_of_prh(1., r, 0.);
  const Rot3 rotation = road_curve_->Orientation(1., r, 0.);
  const Vector3<double> w_prime =
      road_curve_->W_prime_of_prh(1., r, 0., road_curve_->Rabg_of_p(1.),
                                  road_curve_->elevation().f_dot_p(1.));
  return Endpoint(EndpointXy(position[0], position[1], rotation.yaw()),
                  EndpointZ(position[2], w_prime.z(), start_.z().theta(),
                            start_.z().theta_dot()));
}

namespace {
// Construct a CubicPolynomial such that:
//    f(0) = Y0 / dX           f'(0) = Ydot0
//    f(1) = (Y0 + dY) / dX    f'(1) = Ydot1
//
// This is equivalent to taking a cubic polynomial g such that:
//    g(0) = Y0          g'(0) = Ydot0
//    g(dX) = Y0 + dY    g'(1) = Ydot1
// and isotropically scaling it (scale both axes) by a factor of 1/dX
CubicPolynomial MakeCubic(double dX, double Y0, double dY,
                          double Ydot0, double Ydot1) {
  return CubicPolynomial(Y0 / dX,
                         Ydot0,
                         (3. * dY / dX) - (2. * Ydot0) - Ydot1,
                         Ydot0 + Ydot1 - (2. * dY / dX));
}
}  // namespace

std::unique_ptr<RoadCurve> Connection::CreateRoadCurve() const {
  switch (type_) {
    case Connection::kLine: {
      const Vector2<double> xy0(start_.xy().x(), start_.xy().y());
      const Vector2<double> dxy(end_.xy().x() - start_.xy().x(),
                                end_.xy().y() - start_.xy().y());
      const CubicPolynomial elevation(MakeCubic(
          dxy.norm(),
          start_.z().z(),
          end_.z().z() - start_.z().z(),
          start_.z().z_dot(),
          end_.z().z_dot()));
      const CubicPolynomial superelevation(MakeCubic(
          dxy.norm(),
          start_.z().theta(),
          end_.z().theta() - start_.z().theta(),
          start_.z().theta_dot(),
          end_.z().theta_dot()));
      return
        std::make_unique<LineRoadCurve>(xy0, dxy, elevation, superelevation);
    };
    case Connection::kArc: {
      const Vector2<double> center(cx_, cy_);
      const double arc_length = radius_ * std::abs(d_theta_);
      const CubicPolynomial elevation(MakeCubic(
          arc_length,
          start_.z().z(),
          end_.z().z() - start_.z().z(),
          start_.z().z_dot(),
          end_.z().z_dot()));
      const CubicPolynomial superelevation(MakeCubic(
          arc_length,
          start_.z().theta(),
          end_.z().theta() - start_.z().theta(),
          start_.z().theta_dot(),
          end_.z().theta_dot()));
      return std::make_unique<ArcRoadCurve>(
          center, radius_, theta0_, d_theta_, elevation, superelevation);
    };
    default: { DRAKE_ABORT(); }
  }
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
