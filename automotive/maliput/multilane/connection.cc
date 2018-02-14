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
  end_ = Endpoint(
      {start_.xy().x() + line_length_ * std::cos(start_.xy().heading()),
       start_.xy().y() + line_length_ * std::sin(start_.xy().heading()),
       start_.xy().heading()},
      end_z);
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
      d_theta_(arc_offset.d_theta()),
      theta0_(start.xy().heading() -
              std::copysign(M_PI / 2., arc_offset.d_theta())),
      cx_(start.xy().x() - (arc_offset.radius() * std::cos(theta0_))),
      cy_(start.xy().y() - (arc_offset.radius() * std::sin(theta0_))) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(radius_ > 0);
  // Computes end Endpoint and RoadCurve.
  const double theta1 = theta0_ + d_theta_;
  end_ = Endpoint(
      {cx_ + radius_ * std::cos(theta1), cy_ + radius_ * std::sin(theta1),
       start_.xy().heading() + d_theta_},
      end_z);
  road_curve_ = CreateRoadCurve();
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, {0., 0.}));
}

Connection::Connection(const std::string& id, const Endpoint& start,
                       int start_lane_index, const EndpointZ& end_z,
                       int end_lane_index, int num_lanes, double r_ref,
                       int lane_ref_index, double lane_width,
                       double left_shoulder, double right_shoulder,
                       double line_length)
    : type_(kLine),
      id_(id),
      num_lanes_(num_lanes),
      r0_(r_ref - static_cast<double>(lane_ref_index) * lane_width),
      lane_width_(lane_width),
      left_shoulder_(left_shoulder),
      right_shoulder_(right_shoulder),
      r_min_(r0_ - lane_width / 2. - right_shoulder),
      r_max_(r0_ + (static_cast<double>(num_lanes - 1) + 0.5) * lane_width +
             left_shoulder),
      line_length_(line_length) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(start_lane_index >= 0 && start_lane_index < num_lanes_);
  DRAKE_DEMAND(end_lane_index >= 0 && end_lane_index < num_lanes_);
  DRAKE_DEMAND(lane_ref_index >= 0 && lane_ref_index < num_lanes_);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(r0_ >= r_min_ && r0_ <= r_max_);
  DRAKE_DEMAND(line_length_ > 0.);
  // Computes the distance from the start_lane_index lane centerline to the
  // reference curve, as well as the end_lane_index lane centerline.
  const double r_start_lane = lane_offset(start_lane_index);
  const double r_end_lane_ = lane_offset(end_lane_index);
  // Computes the start and end Endpoints for the RoadCurve.
  //
  // Start and end EndpointXys should be translated to the reference curve
  // position. In order to compute new xy positions, superelevation at the
  // extents must be taken into account.

  // TODO(agalbachicar)  xy coordinates are affected by superelevation, and if
  //                     elevation is present, a composition on both effects
  //                     needs to be solved. Up to now, only elevation is
  //                     translated.
  start_ =
      Endpoint({start.xy().x() -
                    r_start_lane * std::cos(start.xy().heading() + M_PI / 2.),
                start.xy().y() -
                    r_start_lane * std::sin(start.xy().heading() + M_PI / 2.),
                start.xy().heading()},
               {start.z().z() - r_start_lane * std::sin(start.z().theta()),
                start.z().z_dot(), start.z().theta(), start.z().theta_dot()});
  end_ =
      Endpoint({start_.xy().x() + line_length_ * std::cos(start.xy().heading()),
                start_.xy().y() + line_length_ * std::sin(start.xy().heading()),
                start_.xy().heading()},
               {end_z.z() - r_end_lane_ * std::sin(end_z.theta()),
                end_z.z_dot(), end_z.theta(), end_z.theta_dot()});
  road_curve_ = CreateRoadCurve();
  // TODO(agalbachicar)  Modify Connection API to provide support for HBounds
  //                     once RoadCurve's children are capable of computing
  //                     singularities with it.
  DRAKE_DEMAND(road_curve_->IsValid(r_min_, r_max_, {0., 0.}));
}

Connection::Connection(const std::string& id, const Endpoint& start,
                       int start_lane_index, const EndpointZ& end_z,
                       int end_lane_index, int num_lanes, double r_ref,
                       int lane_ref_index, double lane_width,
                       double left_shoulder, double right_shoulder,
                       const ArcOffset& arc_offset)
    : type_(kArc),
      id_(id),
      num_lanes_(num_lanes),
      r0_(r_ref - static_cast<double>(lane_ref_index) * lane_width),
      lane_width_(lane_width),
      left_shoulder_(left_shoulder),
      right_shoulder_(right_shoulder),
      r_min_(r0_ - lane_width / 2. - right_shoulder),
      r_max_(r0_ + (static_cast<double>(num_lanes - 1) + 0.5) * lane_width +
             left_shoulder),
      radius_(arc_offset.radius()),
      d_theta_(arc_offset.d_theta()),
      theta0_(start.xy().heading() -
              std::copysign(M_PI / 2., arc_offset.d_theta())),
      cx_(start.xy().x() -
          (radius_ -
           (r0_ + static_cast<double>(start_lane_index) * lane_width_) *
               std::cos(start.z().theta()) * std::copysign(1., d_theta_)) *
              std::cos(theta0_)),
      cy_(start.xy().y() -
          (radius_ -
           (r0_ + static_cast<double>(start_lane_index) * lane_width_) *
               std::cos(start.z().theta()) * std::copysign(1., d_theta_)) *
              std::sin(theta0_)) {
  DRAKE_DEMAND(num_lanes_ > 0);
  DRAKE_DEMAND(start_lane_index >= 0 && start_lane_index < num_lanes_);
  DRAKE_DEMAND(end_lane_index >= 0 && end_lane_index < num_lanes_);
  DRAKE_DEMAND(lane_ref_index >= 0 && lane_ref_index < num_lanes_);
  DRAKE_DEMAND(lane_width_ >= 0);
  DRAKE_DEMAND(left_shoulder_ >= 0);
  DRAKE_DEMAND(right_shoulder_ >= 0);
  DRAKE_DEMAND(r_max_ >= r_min_);
  DRAKE_DEMAND(r0_ >= r_min_ && r0_ <= r_max_);
  DRAKE_DEMAND(radius_ > 0);
  // Computes the distance from the start_lane_index lane centerline to the
  // reference curve, as well as the end_lane_index lane centerline.
  const double r_start_lane = lane_offset(start_lane_index);
  const double r_end_lane = lane_offset(end_lane_index);
  // Computes planar reference curve end angle.
  const double theta1 = theta0_ + d_theta_;
  // Computes start and end Endpoints for the RoadCurve.
  //
  // Start and end EndpointXys should be translated to the reference curve
  // position. In order to compute new xy positions, superelevation at the
  // extents must be taken into account. Similarly, EndpointZ information needs
  // to be scaled. See LaneStart() or LaneEnd() methods for more details on the
  // scale factor.

  // TODO(agalbachicar)  xy coordinates are affected by superelevation, and if
  //                     elevation is present, a composition on both effects
  //                     needs to be solved. Up to now, only elevation is
  //                     translated.

  const double start_dot_scale = (radius_ -
                                  std::copysign(1., d_theta_) * r_start_lane *
                                      std::cos(start.z().theta())) /
                                 radius_;
  start_ = Endpoint({cx_ + radius_ * std::cos(theta0_),
                     cy_ + radius_ * std::sin(theta0_), start.xy().heading()},
                    {start.z().z() - r_start_lane * std::sin(start.z().theta()),
                     start.z().z_dot() * start_dot_scale, start.z().theta(),
                     start.z().theta_dot() * start_dot_scale});

  const double end_dot_scale =
      (radius_ -
       std::copysign(1., d_theta_) * r_end_lane * std::cos(end_z.theta())) /
      radius_;
  end_ = Endpoint(
      {cx_ + radius_ * std::cos(theta1), cy_ + radius_ * std::sin(theta1),
       start.xy().heading() + d_theta_},
      {end_z.z() - r_end_lane * std::sin(end_z.theta()),
       end_z.z_dot() * end_dot_scale, end_z.theta(),
       end_z.theta_dot() * end_dot_scale});
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
  const double theta_dot = type_ == kLine ?
      start_.z().theta_dot() :
      start_.z().theta_dot() * std::abs(d_theta_ * radius_) / planar_length;
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
  const double theta_dot = type_ == kLine ?
      end_.z().theta_dot() :
      end_.z().theta_dot() * std::abs(d_theta_ * radius_) / planar_length;
  return Endpoint({position[0], position[1], rotation.yaw()},
                  {position[2], z_dot, end_.z().theta(), theta_dot});
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
    default: {
      DRAKE_ABORT();
    }
  }
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
