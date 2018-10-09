#include "drake/automotive/maliput/multilane/road_curve.h"

#include <algorithm>
#include <memory>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/scalar_dense_output.h"

namespace drake {
namespace maliput {
namespace multilane {

namespace {

// Arc length derivative function ds/dp = f(p; [r, h]) for numerical resolution
// of the s(p) mapping as an antiderivative computation (i.e. quadrature).
struct ArcLengthDerivativeFunction {
  // Constructs the arc length derivative function for the given @p road_curve.
  explicit ArcLengthDerivativeFunction(const RoadCurve* road_curve)
      : road_curve_(road_curve) {}

  // Computes the arc length derivative for the RoadCurve specified
  // at construction.
  //
  // @param p The parameterization value to evaluate the derivative at.
  // @param k The parameter vector, containing r and h coordinates respectively.
  // @return The arc length derivative value at the specified point.
  // @pre The given parameter vector @p k is bi-dimensional (holding r and h
  //      coordinates only).
  // @throws std::logic_error if preconditions are not met.
  double operator()(const double& p, const VectorX<double>& k) const {
    if (k.size() != 2) {
      throw std::logic_error("Arc length derivative expects only r and"
                             " h coordinates as parameters, respectively.");
    }
    return road_curve_->W_prime_of_prh(
        p, k(0), k(1), road_curve_->Rabg_of_p(p),
        road_curve_->elevation().f_dot_p(p)).norm();
  }

 private:
  // Associated RoadCurve instance.
  const RoadCurve* road_curve_;
};

// Inverse arc length ODE function dp/ds = f(s, p; [r, h]) for numerical
// resolution of the p(s) mapping as an scalar initial value problem for
// a given RoadCurve.
struct InverseArcLengthODEFunction {
  // Constructs an inverse arc length ODE for the given @p road_curve.
  explicit InverseArcLengthODEFunction(const RoadCurve* road_curve)
      : road_curve_(road_curve) {}

  // Computes the inverse arc length derivative for the RoadCurve specified
  // at construction.
  //
  // @param s The arc length to evaluate the derivative at.
  // @param p The parameterization value to evaluate the derivative at.
  // @param k The parameter vector, containing r and h coordinates respectively.
  // @return The inverse arc length derivative value at the specified point.
  // @pre The given parameter vector @p k is bi-dimensional (holding r and h
  //      coordinates only).
  // @throws std::logic_error if preconditions are not met.
  double operator()(const double& s, const double& p,
                    const VectorX<double>& k) {
    unused(s);
    if (k.size() != 2) {
      throw std::logic_error("Inverse arc length ODE expects only r and"
                             " h coordinates as parameters, respectively.");
    }
    return 1.0 / road_curve_->W_prime_of_prh(
        p, k(0), k(1), road_curve_->Rabg_of_p(p),
        road_curve_->elevation().f_dot_p(p)).norm();
  }

 private:
  // Associated RoadCurve instance.
  const RoadCurve* road_curve_;
};


}  // namespace


RoadCurve::RoadCurve(double linear_tolerance, double scale_length,
                     const CubicPolynomial& elevation,
                     const CubicPolynomial& superelevation,
                     ComputationPolicy computation_policy)
    : scale_length_(scale_length),
      linear_tolerance_(linear_tolerance),
      elevation_(elevation),
      superelevation_(superelevation),
      computation_policy_(computation_policy) {
  // Enforces preconditions.
  DRAKE_THROW_UNLESS(scale_length > 0.);
  DRAKE_THROW_UNLESS(linear_tolerance > 0.);
  // Sets default parameter value at the beginning of the
  // curve to 0 by default.
  const double initial_p_value = 0.0;
  // Sets default arc length at the beginning of the curve
  // to 0 by default.
  const double initial_s_value = 0.0;
  // Sets default r and h coordinates to 0 by default.
  const VectorX<double> default_parameters = VectorX<double>::Zero(2);

  // Instantiates s(p) and p(s) mappings with default values.
  const systems::AntiderivativeFunction<double>::SpecifiedValues
      s_from_p_func_values(initial_p_value, default_parameters);
  s_from_p_func_ = std::make_unique<systems::AntiderivativeFunction<double>>(
      ArcLengthDerivativeFunction(this), s_from_p_func_values);

  const systems::ScalarInitialValueProblem<double>::SpecifiedValues
      p_from_s_ivp_values(initial_s_value, initial_p_value, default_parameters);
  p_from_s_ivp_ = std::make_unique<systems::ScalarInitialValueProblem<double>>(
      InverseArcLengthODEFunction(this), p_from_s_ivp_values);

  // Relative tolerance in path length is roughly bounded by e/L, where e is
  // the linear tolerance and L is the scale length. This can be seen by
  // considering straight path one scale length (or spatial period) long, and
  // then another path, whose deviation from the first is a sine function with
  // the same period and amplitude equal to the specified tolerance. The
  // difference in path length is bounded by 4e and the relative error is thus
  // bounded by 4e/L.
  relative_tolerance_ = linear_tolerance_ / scale_length_;

  // Sets `s_from_p`'s integration accuracy and step sizes. Said steps
  // should not be too large, because that could make accuracy control
  // fail, nor too small to avoid wasting cycles. The nature of the
  // problem at hand varies with the parameterization of the RoadCurve,
  // and so will optimal step sizes (in terms of their efficiency vs.
  // accuracy balance). However, for the time being, the following
  // constants (considering 0.0 <= p <= 1.0) work well as a heuristic
  // approximation to appropriate step sizes.
  systems::IntegratorBase<double>* s_from_p_integrator =
      s_from_p_func_->get_mutable_integrator();
  s_from_p_integrator->request_initial_step_size_target(0.1);
  s_from_p_integrator->set_maximum_step_size(1.0);
  s_from_p_integrator->set_target_accuracy(relative_tolerance_);

  // Sets `p_from_s`'s integration accuracy and step sizes. Said steps
  // should not be too large, because that could make accuracy control
  // fail, nor too small to avoid wasting cycles. The nature of the
  // problem at hand varies with the shape of the RoadCurve, and so will
  // optimal step sizes (in terms of their efficiency vs. accuracy balance).
  // However, for the time being, the following proportions of the scale
  // length work well as a heuristic approximation to appropriate step sizes.
  systems::IntegratorBase<double>* p_from_s_integrator =
      p_from_s_ivp_->get_mutable_integrator();
  p_from_s_integrator->request_initial_step_size_target(0.1 * scale_length);
  p_from_s_integrator->set_maximum_step_size(scale_length);
  p_from_s_integrator->set_target_accuracy(relative_tolerance_);
}

bool RoadCurve::AreFastComputationsAccurate(double r) const {
  // When superelevation() has no influence on the curve's
  // geometry and elevation() is at most linear along the curve,
  // known analytical expressions are accurate.
  return ((r == 0.0 || superelevation().is_zero())
          && elevation().order() <= 1);
}

std::function<double(double)> RoadCurve::OptimizeCalcSFromP(double r) const {
  DRAKE_THROW_UNLESS(CalcMinimumRadiusAtOffset(r) > 0.0);
  const double absolute_tolerance = relative_tolerance_ * 1.;
  if (computation_policy() == ComputationPolicy::kPreferAccuracy
      && !AreFastComputationsAccurate(r)) {
    // Populates parameter vector with (r, h) coordinate values.
    systems::AntiderivativeFunction<double>::SpecifiedValues values;
    values.k = (VectorX<double>(2) << r, 0.0).finished();
    // Prepares dense output for shared ownership, as std::function
    // instances only take copyable callables.
    const std::shared_ptr<systems::ScalarDenseOutput<double>> dense_output{
      s_from_p_func_->MakeDenseEvalFunction(1.0, values)};
    DRAKE_DEMAND(dense_output->start_time() <= 0.);
    DRAKE_DEMAND(dense_output->end_time() >= 1.);
    return [dense_output, absolute_tolerance] (double p) -> double {
      // Saturates p to lie within the [0., 1.] interval.
      const double saturated_p = std::min(std::max(p, 0.), 1.);
      DRAKE_THROW_UNLESS(std::abs(saturated_p - p) < absolute_tolerance);
      return dense_output->EvaluateScalar(saturated_p);
    };
  }
  return [this, r, absolute_tolerance] (double p) {
    // Saturates p to lie within the [0., 1.] interval.
    const double saturated_p = std::min(std::max(p, 0.), 1.);
    DRAKE_THROW_UNLESS(std::abs(saturated_p - p) < absolute_tolerance);
    return this->FastCalcSFromP(p, r);
  };
}

double RoadCurve::CalcSFromP(double p, double r) const {
  // Populates parameter vector with (r, h) coordinate values.
  systems::AntiderivativeFunction<double>::SpecifiedValues values;
  values.k = (VectorX<double>(2) << r, 0.0).finished();
  return s_from_p_func_->Evaluate(p, values);
}

std::function<double(double)> RoadCurve::OptimizeCalcPFromS(double r) const {
  DRAKE_THROW_UNLESS(CalcMinimumRadiusAtOffset(r) > 0.0);
  const double full_length = CalcSFromP(1., r);
  const double absolute_tolerance = relative_tolerance_ * full_length;
  if (computation_policy() == ComputationPolicy::kPreferAccuracy
      && !AreFastComputationsAccurate(r)) {
    // Populates parameter vector with (r, h) coordinate values.
    systems::ScalarInitialValueProblem<double>::SpecifiedValues values;
    values.k = (VectorX<double>(2) << r, 0.0).finished();
    // Prepares dense output for shared ownership, as std::function
    // instances only take copyable callables.
    const std::shared_ptr<systems::ScalarDenseOutput<double>> dense_output{
      p_from_s_ivp_->DenseSolve(full_length, values)};
    DRAKE_DEMAND(dense_output->start_time() <= 0.);
    DRAKE_DEMAND(dense_output->end_time() >= full_length);
    return [dense_output, full_length,
            absolute_tolerance] (double s) -> double {
      // Saturates s to lie within the [0., full_length] interval.
      const double saturated_s = std::min(std::max(s, 0.), full_length);
      DRAKE_THROW_UNLESS(std::abs(saturated_s - s) < absolute_tolerance);
      return dense_output->EvaluateScalar(saturated_s);
    };
  }
  return [this, r, full_length, absolute_tolerance] (double s) {
    // Saturates s to lie within the [0., full_length] interval.
    const double saturated_s = std::min(std::max(s, 0.), full_length);
    DRAKE_THROW_UNLESS(std::abs(saturated_s - s) < absolute_tolerance);
    return this->FastCalcPFromS(s, r);
  };
}


double RoadCurve::CalcGPrimeAsUsedForCalcSFromP(double p) const {
  if (computation_policy() == ComputationPolicy::kPreferSpeed) {
    return elevation().fake_gprime(p);
  }
  return elevation().f_dot_p(p);
}

Vector3<double> RoadCurve::W_of_prh(double p, double r, double h) const {
  // Calculates z (elevation) of (p,0,0).
  const double z = elevation().f_p(p) * l_max();
  // Calculates x,y of (p,0,0).
  const Vector2<double> xy = xy_of_p(p);
  // Calculates orientation of (p,r,h) basis at (p,0,0).
  const Rot3 ypr = Rabg_of_p(p);
  // Rotates (0,r,h) and sums with mapped (p,0,0).
  return ypr.apply({0., r, h}) + Vector3<double>(xy.x(), xy.y(), z);
}


Vector3<double> RoadCurve::W_prime_of_prh(double p, double r, double h,
                                          const Rot3& Rabg,
                                          double g_prime) const {
  const Vector2<double> G_prime = xy_dot_of_p(p);

  const Rot3& R = Rabg;
  const double alpha = R.roll();
  const double beta = R.pitch();
  const double gamma = R.yaw();

  const double ca = std::cos(alpha);
  const double cb = std::cos(beta);
  const double cg = std::cos(gamma);
  const double sa = std::sin(alpha);
  const double sb = std::sin(beta);
  const double sg = std::sin(gamma);

  // Evaluate dα/dp, dβ/dp, dγ/dp...
  const double d_alpha = superelevation().f_dot_p(p) * l_max();
  const double d_beta = -cb * cb * elevation().f_ddot_p(p);
  const double d_gamma = heading_dot_of_p(p);

  // Recall that W is the lane-to-world transform, defined by
  //   (x,y,z)  = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0,r,h)
  // where G is the reference curve, Z is the elevation profile, and R_αβγ is
  // a rotation matrix derived from reference curve (heading), elevation,
  // and superelevation.
  //
  // Thus, ∂W/∂p = (∂G(p)/∂p, ∂Z(p)/∂p) + (∂R_αβγ/∂p)*(0,r,h), where
  //
  //   ∂G(p)/∂p = G'(p)
  //
  //   ∂Z(p)/∂p = l_max * (z / l_max) = l_max * g'(p)
  //
  //   ∂R_αβγ/∂p = (∂R_αβγ/∂α ∂R_αβγ/∂β ∂R_αβγ/∂γ)*(dα/dp, dβ/dp, dγ/dp)
  return
      Vector3<double>(G_prime.x(),
                      G_prime.y(),
                      l_max() * g_prime) +

      Vector3<double>((((sa*sg)+(ca*sb*cg))*r + ((ca*sg)-(sa*sb*cg))*h),
                      (((-sa*cg)+(ca*sb*sg))*r - ((ca*cg)+(sa*sb*sg))*h),
                      ((ca*cb)*r + (-sa*cb)*h))
                      * d_alpha +

      Vector3<double>(((sa*cb*cg)*r + (ca*cb*cg)*h),
                      ((sa*cb*sg)*r + (ca*cb*sg)*h),
                      ((-sa*sb)*r - (ca*sb)*h))
                      * d_beta +

      Vector3<double>((((-ca*cg)-(sa*sb*sg))*r + ((+sa*cg)-(ca*sb*sg))*h),
                      (((-ca*sg)+(sa*sb*cg))*r + ((sa*sg)+(ca*sb*cg))*h),
                      0)
                      * d_gamma;
}

Rot3 RoadCurve::Rabg_of_p(double p) const {
  return Rot3(superelevation().f_p(p) * l_max(),
              -std::atan(elevation().f_dot_p(p)),
              heading_of_p(p));
}

Rot3 RoadCurve::Orientation(double p, double r, double h) const {
  // Calculate orientation of (s,r,h) basis at (s,0,0).
  const Rot3 Rabg = Rabg_of_p(p);
  const double real_g_prime = elevation().f_dot_p(p);

  // Calculate s,r basis vectors at (s,r,h)...
  const Vector3<double> s_hat = s_hat_of_prh(p, r, h, Rabg, real_g_prime);
  const Vector3<double> r_hat = r_hat_of_Rabg(Rabg);
  // ...and then derive orientation from those basis vectors.
  //
  // (s_hat  r_hat  h_hat) is an orthonormal basis, obtained by rotating the
  // (x_hat  y_hat  z_hat) basis by some R-P-Y rotation; in this case, we know
  // the value of (s_hat  r_hat  h_hat) (w.r.t. 'xyz' world frame), so we are
  // trying to recover the roll/pitch/yaw.  Since (x_hat  y_hat  z_hat) is an
  // identity matrix (e.g., x_hat = column vector (1, 0, 0), etc), then
  // (s_hat  r_hat  h_hat) equals the R-P-Y matrix itself.
  // If we define a = alpha = roll, b = beta = pitch, g = gamma = yaw,
  // then s_hat is the first column of the rotation, r_hat is the second:
  //   s_hat = (cb * cg, cb * sg, - sb)
  //   r_hat = (- ca * sg + sa * sb * cg, ca * cg + sa * sb * sg, sa * cb)
  // We solve the above for a, b, g.
  const double gamma = std::atan2(s_hat.y(), s_hat.x());
  const double beta =
      std::atan2(-s_hat.z(), Vector2<double>(s_hat.x(), s_hat.y()).norm());
  const double cb = std::cos(beta);
  const double alpha = std::atan2(
      r_hat.z() / cb, ((r_hat.y() * s_hat.x()) - (r_hat.x() * s_hat.y())) / cb);
  return {alpha, beta, gamma};
}

Vector3<double> RoadCurve::s_hat_of_prh(double p, double r, double h,
                                        const Rot3& Rabg,
                                        double g_prime) const {
  const Vector3<double> W_prime = W_prime_of_prh(p, r, h, Rabg, g_prime);
  return W_prime * (1.0 / W_prime.norm());
}


Vector3<double> RoadCurve::r_hat_of_Rabg(const Rot3& Rabg) const {
  return Rabg.apply({0., 1., 0.});
}


}  // namespace multilane
}  // namespace maliput
}  // namespace drake
