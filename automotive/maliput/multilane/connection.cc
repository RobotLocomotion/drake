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
             << ", theta = " << endpoint_z.theta() << ", theta_dot = "
             << (endpoint_z.theta_dot().has_value()
                     ? std::to_string(*endpoint_z.theta_dot())
                     : std::string("nullopt"))
             << ")";
}

std::ostream& operator<<(std::ostream& out, const Endpoint& endpoint) {
  return out << "(xy: " << endpoint.xy() << ", z: " << endpoint.z() << ")";
}

std::ostream& operator<<(std::ostream& out, const LineOffset& line_offset) {
  return out << "(length: " << line_offset.length() << ")";
}

std::ostream& operator<<(std::ostream& out, const ArcOffset& arc_offset) {
  return out << "(r: " << arc_offset.radius()
             << ", d_theta: " << arc_offset.d_theta() << ")";
}

Connection::Connection(const std::string& id, const Endpoint& start,
                       const EndpointZ& end_z, int num_lanes, double r0,
                       double lane_width, double left_shoulder,
                       double right_shoulder, const LineOffset& line_offset,
                       double linear_tolerance, double scale_length,
                       ComputationPolicy computation_policy)
    : type_(kLine),
      id_(id),
      start_(start),
      end_z_(end_z),
      num_lanes_(num_lanes),
      r0_(r0),
      lane_width_(lane_width),
      left_shoulder_(left_shoulder),
      right_shoulder_(right_shoulder),
      r_min_(r0 - lane_width / 2. - right_shoulder),
      r_max_(r0 + lane_width * (static_cast<double>(num_lanes - 1) + 0.5) +
             left_shoulder),
      linear_tolerance_(linear_tolerance),
      scale_length_(scale_length),
      computation_policy_(computation_policy),
      line_length_(line_offset.length()) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(linear_tolerance_ > 0.);
  DRAKE_DEMAND(scale_length_ > 0.);
  DRAKE_DEMAND(line_length_ > 0.);
  DRAKE_DEMAND(start.z().theta_dot().has_value());
  DRAKE_DEMAND(end_z.theta_dot().has_value());
  // Computes the RoadCurve.
  road_curve_ = CreateRoadCurve();
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, {0., 0.}));
  ComputeEndEndpoint();
}

Connection::Connection(const std::string& id, const Endpoint& start,
                       const EndpointZ& end_z, int num_lanes, double r0,
                       double lane_width, double left_shoulder,
                       double right_shoulder, const ArcOffset& arc_offset,
                       double linear_tolerance, double scale_length,
                       ComputationPolicy computation_policy)
    : type_(kArc),
      id_(id),
      start_(start),
      end_z_(end_z),
      num_lanes_(num_lanes),
      r0_(r0),
      lane_width_(lane_width),
      left_shoulder_(left_shoulder),
      right_shoulder_(right_shoulder),
      r_min_(r0 - lane_width / 2. - right_shoulder),
      r_max_(r0 + lane_width * (static_cast<double>(num_lanes - 1) + 0.5) +
             left_shoulder),
      linear_tolerance_(linear_tolerance),
      scale_length_(scale_length),
      computation_policy_(computation_policy),
      radius_(arc_offset.radius()),
      d_theta_(arc_offset.d_theta()) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(linear_tolerance_ > 0.);
  DRAKE_DEMAND(scale_length_ > 0.);
  DRAKE_DEMAND(radius_ > 0);
  DRAKE_DEMAND(start.z().theta_dot().has_value());
  DRAKE_DEMAND(end_z.theta_dot().has_value());
  // Fills arc related parameters, computes end Endpoint and creates the
  // RoadCurve.
  theta0_ = start.xy().heading() - std::copysign(M_PI / 2., d_theta_);
  // Computes the center.
  cx_ = start.xy().x() - std::cos(theta0_) * radius_;
  cy_ = start.xy().y() - std::sin(theta0_) * radius_;
  // Computes the RoadCurve.
  road_curve_ = CreateRoadCurve();

  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, {0., 0.}));
  ComputeEndEndpoint();
}

Connection::Connection(const std::string& id, const Endpoint& start,
                       double start_lane, const EndpointZ& end_z,
                       double end_lane, int num_lanes, int lane_ref,
                       double r_ref, double lane_width, double left_shoulder,
                       double right_shoulder, const LineOffset& line_offset,
                       double linear_tolerance, double scale_length,
                       ComputationPolicy computation_policy)
    : type_(kLine),
      id_(id),
      num_lanes_(num_lanes),
      r0_(-r_ref - lane_ref * lane_width),
      lane_width_(lane_width),
      left_shoulder_(left_shoulder),
      right_shoulder_(right_shoulder),
      r_min_((-r_ref - lane_ref * lane_width) - lane_width / 2. -
             right_shoulder),
      r_max_((-r_ref - lane_ref * lane_width) +
             lane_width * (static_cast<double>(num_lanes - 1) + 0.5) +
             left_shoulder),
      linear_tolerance_(linear_tolerance),
      scale_length_(scale_length),
      computation_policy_(computation_policy),
      line_length_(line_offset.length()) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(lane_ref >= 0 && lane_ref < num_lanes_);
  DRAKE_DEMAND(start_lane >= 0 && start_lane < num_lanes_);
  DRAKE_DEMAND(end_lane >= 0 && end_lane < num_lanes_);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(line_length_ > 0.);
  DRAKE_DEMAND(linear_tolerance_ > 0.);
  DRAKE_DEMAND(scale_length_ > 0.);

  // Gets the initial heading and superelevation.
  const double start_heading = start.xy().heading();
  const double start_superelevation = start.z().theta();
  const Vector3<double> start_lane_position(start.xy().x(), start.xy().y(),
                                            start.z().z());
  const double start_r = lane_offset(start_lane);
  // Computes the displacement vector from lane point to the reference curve
  // point.
  const Vector3<double> start_lane_to_ref(
      -start_r * std::cos(start_superelevation) * std::sin(start_heading),
      start_r * std::cos(start_superelevation) * std::cos(start_heading),
      start_r * std::sin(start_superelevation));
  const Vector3<double> start_reference_position =
      start_lane_position - start_lane_to_ref;
  // Assigns the start endpoint. theta_dot is zeroed as it needs to match the
  // the continuity constraint for the reference curve.
  start_ =
      Endpoint(EndpointXy(start_reference_position.x(),
                          start_reference_position.y(), start_heading),
               EndpointZ(start_reference_position.z(), start.z().z_dot(),
                         start.z().theta(), 0.));

  // Computes the r distance from the `end_lane` to the reference curve.
  // theta_dot is zeroed as it needs to match the the continuity constraint
  // for the reference curve.
  const double z_end =
      end_z.z() - lane_offset(end_lane) * std::sin(end_z.theta());
  end_z_ =
      EndpointZ(z_end, end_z.z_dot(), end_z.theta(), 0.);

  road_curve_ = CreateRoadCurve();
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, {0., 0.}));
  ComputeEndEndpoint();
}

Connection::Connection(const std::string& id, const Endpoint& start,
                       double start_lane, const EndpointZ& end_z,
                       double end_lane, int num_lanes, int lane_ref,
                       double r_ref, double lane_width, double left_shoulder,
                       double right_shoulder, const ArcOffset& arc_offset,
                       double linear_tolerance, double scale_length,
                       ComputationPolicy computation_policy)
    : type_(kArc),
      id_(id),
      num_lanes_(num_lanes),
      r0_(-r_ref - lane_ref * lane_width),
      lane_width_(lane_width),
      left_shoulder_(left_shoulder),
      right_shoulder_(right_shoulder),
      r_min_((-r_ref - lane_ref * lane_width) - lane_width / 2. -
             right_shoulder),
      r_max_((-r_ref - lane_ref * lane_width) +
             lane_width * (static_cast<double>(num_lanes - 1) + 0.5) +
             left_shoulder),
      linear_tolerance_(linear_tolerance),
      scale_length_(scale_length),
      computation_policy_(computation_policy),
      radius_(arc_offset.radius()),
      d_theta_(arc_offset.d_theta()) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(lane_ref >= 0 && lane_ref < num_lanes_);
  DRAKE_DEMAND(start_lane >= 0 && start_lane < num_lanes_);
  DRAKE_DEMAND(end_lane >= 0 && end_lane < num_lanes_);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(radius_ > 0);
  DRAKE_DEMAND(linear_tolerance_ > 0.);
  DRAKE_DEMAND(scale_length_ > 0.);

  // Fills arc related parameters, computes end Endpoint and creates the
  // RoadCurve.
  theta0_ = start.xy().heading() - std::copysign(M_PI / 2., d_theta_);
  // Gets the initial heading and superelevation.
  const double start_superelevation = start.z().theta();

  // Computes the center.
  const double start_lane_radius = radius_ -
                                   lane_offset(start_lane) *
                                       std::cos(start_superelevation) *
                                       std::copysign(1., d_theta_);
  cx_ = start.xy().x() - std::cos(theta0_) * start_lane_radius;
  cy_ = start.xy().y() - std::sin(theta0_) * start_lane_radius;

  const double start_z_dot = start.z().z_dot() * start_lane_radius / radius_;
  const double start_theta_dot = std::sin(-std::atan(start_z_dot)) / radius_;
  // Assigns the start endpoint.
  start_ = Endpoint(
      EndpointXy(cx_ + radius_ * std::cos(theta0_),
                 cy_ + radius_ * std::sin(theta0_), start.xy().heading()),
      EndpointZ(start.z().z() -
                    lane_offset(start_lane) * std::sin(start_superelevation) *
                        std::copysign(1., d_theta_),
                start_z_dot, start.z().theta(), start_theta_dot));

  const double end_superelevation = end_z.theta();
  const double end_z_dot =
      end_z.z_dot() * (radius_ -
                       lane_offset(end_lane) * std::cos(end_superelevation) *
                           std::copysign(1., d_theta_)) /
      radius_;
  const double end_theta_dot = (1. / radius_) * std::sin(-std::atan(end_z_dot));
  end_z_ = EndpointZ(end_z.z() -
                         lane_offset(end_lane) * std::sin(end_superelevation) *
                             std::copysign(1., d_theta_),
                     end_z_dot, end_z.theta(), end_theta_dot);

  road_curve_ = CreateRoadCurve();
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, {0., 0.}));
  ComputeEndEndpoint();
}

Endpoint Connection::LaneStart(int lane_index) const {
  DRAKE_DEMAND(lane_index >= 0 && lane_index < num_lanes_);
  const double r = lane_offset(lane_index);
  const Vector3<double> position = road_curve_->W_of_prh(0., r, 0.);
  const Rot3 rotation = road_curve_->Orientation(0., r, 0.);
  // Let t be the arc-length xy projection of the lane centerline and t_of_p be
  // a linear function of p (given that p <--> s is a linear relation too).
  // Given z_dot = ∂z/∂t, chain rule can be applied so:
  //
  // z_dot = ∂z/∂p * ∂p/∂t.
  //
  // The same applies to theta_dot.

  // Computes w_prime to obtain ∂z/∂p.
  const Vector3<double> w_prime =
      road_curve_->W_prime_of_prh(0., r, 0., road_curve_->Rabg_of_p(0.),
                                  road_curve_->elevation().f_dot_p(0.));
  // Computes ∂p/∂t based on Connection geometry type.

  // TODO(maddog-tri)  A (second-order?) contribution of theta_dot to ∂p/∂t is
  //                   being ignored.
  const double cos_superelevation =
      std::cos(road_curve_->superelevation().f_p(0.));
  const double planar_length = type_ == kLine ?
      line_length_ :
      std::abs(d_theta_ * (radius_ - std::copysign(1., d_theta_) * r *
                           cos_superelevation));
  // Given that ∂p/∂t = 1 / planar_length.
  const double z_dot = w_prime.z() / planar_length;
  // theta_dot is derivative with respect to t, but the reference curve t
  // coordinate. So, a ∂t_0/∂t_i is needed, being t_0 the reference curve
  // coordinate and t_i the arc-length xy projection for lane_index lane.
  const double theta_dot = type_ == kLine ? *start_.z().theta_dot()
                                          : (*start_.z().theta_dot()) *
                                                std::abs(d_theta_ * radius_) /
                                                planar_length;
  return Endpoint({position[0], position[1], rotation.yaw()},
                  {position[2], z_dot, start_.z().theta(), theta_dot});
}

Endpoint Connection::LaneEnd(int lane_index) const {
  DRAKE_DEMAND(lane_index >= 0 && lane_index < num_lanes_);
  const double r = lane_offset(lane_index);
  const Vector3<double> position = road_curve_->W_of_prh(1., r, 0.);
  const Rot3 rotation = road_curve_->Orientation(1., r, 0.);
  // Let t be the arc-length xy projection of the lane centerline and t_of_p be
  // a linear function of p (given that p <--> s is a linear relation too).
  // Given z_dot = ∂z/∂t, chain rule can be applied so:
  //
  // z_dot = ∂z/∂p * ∂p/∂t.
  //
  // The same applies to theta_dot.

  // Computes w_prime to obtain ∂z/∂p.
  const Vector3<double> w_prime =
      road_curve_->W_prime_of_prh(1., r, 0., road_curve_->Rabg_of_p(1.),
                                  road_curve_->elevation().f_dot_p(1.));
  // Computes ∂p/∂t based on Connection geometry type.

  // TODO(maddog-tri)  A (second-order?) contribution of theta_dot to ∂p/∂t is
  //                   being ignored.
  const double cos_superelevation =
      std::cos(road_curve_->superelevation().f_p(1.));
  const double planar_length = type_ == kLine ?
      line_length_ :
      std::abs(d_theta_ * (radius_ - std::copysign(1., d_theta_) * r *
                           cos_superelevation));
  // Given that ∂p/∂t = 1 / planar_length.
  const double z_dot = w_prime.z() / planar_length;
  // theta_dot is derivative with respect to t, but the reference curve t
  // coordinate. So, a ∂t_0/∂t_i is needed, being t_0 the reference curve
  // coordinate and t_i the arc-length xy projection for lane_index lane.
  const double theta_dot = type_ == kLine ? *end_z_.theta_dot()
                                          : (*end_z_.theta_dot()) *
                                                std::abs(d_theta_ * radius_) /
                                                planar_length;
  return Endpoint({position[0], position[1], rotation.yaw()},
                  {position[2], z_dot, end_z_.theta(), theta_dot});
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
      const Vector2<double> dxy(line_length_ * std::cos(start_.xy().heading()),
                                line_length_ * std::sin(start_.xy().heading()));
      const CubicPolynomial elevation(
          MakeCubic(line_length_, start_.z().z(), end_z_.z() - start_.z().z(),
                    start_.z().z_dot(), end_z_.z_dot()));
      const CubicPolynomial superelevation(MakeCubic(
          dxy.norm(), start_.z().theta(), end_z_.theta() - start_.z().theta(),
          *start_.z().theta_dot(), *end_z_.theta_dot()));
      return std::make_unique<LineRoadCurve>(
          xy0, dxy, elevation, superelevation, linear_tolerance_, scale_length_,
          computation_policy_);
    };
    case Connection::kArc: {
      const Vector2<double> center(cx_, cy_);
      const double arc_length = radius_ * std::abs(d_theta_);
      const CubicPolynomial elevation(
          MakeCubic(arc_length, start_.z().z(), end_z_.z() - start_.z().z(),
                    start_.z().z_dot(), end_z_.z_dot()));
      const CubicPolynomial superelevation(MakeCubic(
          arc_length, start_.z().theta(), end_z_.theta() - start_.z().theta(),
          *start_.z().theta_dot(), *end_z_.theta_dot()));
      return std::make_unique<ArcRoadCurve>(
          center, radius_, theta0_, d_theta_, elevation, superelevation,
          linear_tolerance_, scale_length_, computation_policy_);
    };
    default: {
      DRAKE_ABORT();
    }
  }
}

void Connection::ComputeEndEndpoint() {
  const Vector3<double> position = road_curve_->W_of_prh(1., 0., 0.);
  const Rot3 rotation = road_curve_->Orientation(1., 0., 0.);
  end_ = Endpoint({position[0], position[1], rotation.yaw()}, end_z_);
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
