#include "drake/automotive/maliput/multilane/road_curve.h"

namespace drake {
namespace maliput {
namespace multilane {

Vector3<double> RoadCurve::W_of_prh(double p, double r, double h) const {
  // Calculates z (elevation) of (p,0,0).
  const double z = elevation().f_p(p) * p_scale();
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
  const double d_alpha = superelevation().f_dot_p(p) * p_scale();
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
  //   ∂Z(p)/∂p = p_scale * (z / p_scale) = p_scale * g'(p)
  //
  //   ∂R_αβγ/∂p = (∂R_αβγ/∂α ∂R_αβγ/∂β ∂R_αβγ/∂γ)*(dα/dp, dβ/dp, dγ/dp)
  return
      Vector3<double>(G_prime.x(),
                      G_prime.y(),
                      p_scale() * g_prime) +

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
  return Rot3(superelevation().f_p(p) * p_scale(),
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
