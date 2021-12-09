#include "drake/multibody/plant/hydroelastic_traction_calculator.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/multibody/triangle_quadrature/triangle_quadrature.h"

namespace drake {

using geometry::ContactSurface;
using geometry::TriangleSurfaceMesh;
using geometry::TriangleSurfaceMeshFieldLinear;
using math::RigidTransform;

namespace multibody {
namespace internal {

template <class T>
void HydroelasticTractionCalculator<T>::
    ComputeSpatialForcesAtCentroidFromHydroelasticModel(
        const Data& data, double dissipation, double mu_coulomb,
        std::vector<HydroelasticQuadraturePointData<T>>*
            traction_at_quadrature_points,
        SpatialForce<T>* F_Ac_W) const {
  DRAKE_DEMAND(traction_at_quadrature_points != nullptr);
  DRAKE_DEMAND(F_Ac_W != nullptr);

  // Use a second-order Gaussian quadrature rule. For linear pressure fields,
  // the second-order rule allows exact computation (to floating point error)
  // of the moment on the bodies from the integral of the contact tractions.
  // The moment r × f is a quadratic function of the surface location, since it
  // is a linear operation (r × f) applied to a (typically) linear function
  // (i.e., the traction, f). Higher-order pressure fields and nonlinear
  // tractions (from, e.g., incorporating the Stribeck curve into the friction
  // model) might see benefit from a higher-order quadrature.
  const GaussianTriangleQuadratureRule gaussian(2 /* order */);

  // We'll be accumulating force on body A at the surface centroid C,
  // triangle-by-triangle.
  F_Ac_W->SetZero();

  // Reserve enough memory to keep from doing repeated heap allocations in the
  // quadrature process.
  traction_at_quadrature_points->clear();
  traction_at_quadrature_points->reserve(data.surface.num_faces());

  // Integrate the tractions over all triangles in the contact surface.
  for (int i = 0; i < data.surface.num_faces(); ++i) {
    // Construct the function to be integrated over triangle i.
    // TODO(sherm1) Pull functor creation out of the loop (not a good idea to
    //              create a new functor for every i).
    if (data.surface.is_triangle()) {
      std::function<SpatialForce<T>(const Vector3<T>&)> traction_Ac_W =
          [this, &data, i, dissipation, mu_coulomb,
           traction_at_quadrature_points](const Vector3<T>& Q_barycentric) {
            traction_at_quadrature_points->emplace_back(CalcTractionAtPoint(
                data, i, Q_barycentric, dissipation, mu_coulomb));
            const HydroelasticQuadraturePointData<T>& traction_output =
                traction_at_quadrature_points->back();
            return ComputeSpatialTractionAtAcFromTractionAtAq(
                data, traction_output.p_WQ, traction_output.traction_Aq_W);
          };

      // Compute the integral over the triangle to get a force from the
      // tractions (force/area) at the Gauss points (shifted to C).
      const SpatialForce<T> Fi_Ac_W =  // Force from triangle i.
          TriangleQuadrature<SpatialForce<T>, T>::Integrate(
              traction_Ac_W, gaussian, data.surface.area(i));
      // Update the spatial force at the centroid.
      (*F_Ac_W) += Fi_Ac_W;
    } else {
      traction_at_quadrature_points->emplace_back(
          CalcTractionAtCentroid(data, i, dissipation, mu_coulomb));
      const HydroelasticQuadraturePointData<T>& traction_output =
          traction_at_quadrature_points->back();
      const SpatialForce<T> traction_Ac_W =
          ComputeSpatialTractionAtAcFromTractionAtAq(
              data, traction_output.p_WQ, traction_output.traction_Aq_W);
      (*F_Ac_W) += data.surface.area(i) * traction_Ac_W;
    }
  }
}

template <class T>
void HydroelasticTractionCalculator<T>::
    ShiftSpatialForcesAtCentroidToBodyOrigins(
        const Data& data, const SpatialForce<T>& F_Ac_W,
        SpatialForce<T>* F_Ao_W, SpatialForce<T>* F_Bo_W) const {
  DRAKE_DEMAND(F_Ao_W && F_Bo_W);

  // The spatial force on body A was accumulated at the surface centroid C. We
  // need to shift it to A's origin Ao. The force on body B is equal and
  // opposite to the force on body A, but we want it as if applied at Bo.
  const Vector3<T>& p_WC = data.p_WC;
  const Vector3<T>& p_WAo = data.X_WA.translation();
  const Vector3<T>& p_WBo = data.X_WB.translation();
  const Vector3<T> p_CAo_W = p_WAo - p_WC;
  const Vector3<T> p_CBo_W = p_WBo - p_WC;

  *F_Ao_W = F_Ac_W.Shift(p_CAo_W);
  *F_Bo_W = -(F_Ac_W.Shift(p_CBo_W));
}

// Computes the spatial force on body A acting at a point Ac coincident with
// the surface centroid C, due to the traction on body A at the given contact
// point Q.
// @param data computed once for each pair of geometries.
// @param p_WQ the position vector from the origin of the world frame to the
//        contact point Q, expressed in the world frame.
// @param traction_Aq_W the traction vector applied to Body A at Point Q,
//        expressed in the world frame, where Body A is the body to which
//        `surface.M_id()` is fixed.
// @retval Ft_Ac_W on return, the spatial traction acting at point Ac of
//         Body A resulting from the given traction at Q. (Body A is the one
//         to which `surface.M_id()` is fixed.)
template <typename T>
SpatialForce<T> HydroelasticTractionCalculator<T>::
    ComputeSpatialTractionAtAcFromTractionAtAq(
        const Data& data, const Vector3<T>& p_WQ,
        const Vector3<T>& traction_Aq_W) const {
  // Find the vector from Q to C.
  const Vector3<T> p_QC_W = data.p_WC - p_WQ;

  // Convert the traction to a momentless spatial traction (i.e., without
  // changing the point of application), then shift to body A's origin which
  // will add a moment. (We're using "Ft" for spatial traction.)
  const SpatialForce<T> Ft_Aq_W(Vector3<T>(0, 0, 0), traction_Aq_W);
  const SpatialForce<T> Ft_Ac_W = Ft_Aq_W.Shift(p_QC_W);
  return Ft_Ac_W;  // Still a traction (force/area).
}

// Method for computing traction at a point on a face of the contact surface.
template <typename T>
HydroelasticQuadraturePointData<T>
HydroelasticTractionCalculator<T>::CalcTractionAtPoint(
    const Data& data, int face_index,
    // NOLINTNEXTLINE(runtime/references): "template Bar..." confuses cpplint.
    const typename TriangleSurfaceMesh<T>::template Barycentric<T>&
        Q_barycentric,
    double dissipation, double mu_coulomb) const {
  // Compute the point of contact in the world frame.
  const Vector3<T> p_WQ =
      data.surface.tri_mesh_W().CalcCartesianFromBarycentric(face_index,
                                                             Q_barycentric);

  const T e = data.surface.tri_e_MN().Evaluate(face_index, Q_barycentric);

  // Contact surfaces are documented to have face normals that point *out of* N
  // and *into* M -- which is the face normal of the contact surface (as
  // documented).
  const Vector3<T> nhat_W = data.surface.face_normal(face_index);

  return CalcTractionAtQHelper(data, face_index, e, nhat_W, dissipation,
                               mu_coulomb, p_WQ);
}

template <typename T>
HydroelasticQuadraturePointData<T>
HydroelasticTractionCalculator<T>::CalcTractionAtCentroid(
    const Data& data, int face_index, double dissipation,
    double mu_coulomb) const {
  const Vector3<T>& p_WC = data.surface.centroid(face_index);
  T e;
  if (data.surface.is_triangle()) {
    const typename TriangleSurfaceMesh<T>::template Barycentric<T>
        centroid_barycentric(1. / 3., 1. / 3., 1. / 3.);
    e = data.surface.tri_e_MN().Evaluate(face_index, centroid_barycentric);
  } else {
    e = data.surface.poly_e_MN().EvaluateCartesian(face_index, p_WC);
  }

  // Contact surfaces are documented to have face normals that point *out of* N
  // and *into* M -- which is the face normal of the contact surface (as
  // documented).
  const Vector3<T>& nhat_W = data.surface.face_normal(face_index);

  return CalcTractionAtQHelper(data, face_index, e, nhat_W, dissipation,
                               mu_coulomb, p_WC);
}

/*
 Helper function for computing the traction at a point, irrespective of whether
 that point is coincident with a vertex or is located at an arbitrary
 point on the contact surface.
 @param face_index the face index for the triangle over which quadrature is
        being performed.
 @param e the "potential pressure" (in N/m²) at the point as defined in
        [Elandt 2019]. Note that we drop the _MN suffix here, as this
        suffix can get confused with the identical suffix (used for a different
        purpose) employed by monogram notation.
 @param nhat_W the normal from Geometry M to Geometry N, expressed in the world
        frame, to the contact surface at p_WQ. By extension, this
        means that the normal points from Body A to Body B.
 @param dissipation the dissipation acting between the two bodies.
 @param mu_coulomb the coefficient of friction acting between the two bodies.
 @param p_WQ the point localized to the contact surface, as an offset vector
        expressed in the world frame.
 */
template <typename T>
HydroelasticQuadraturePointData<T>
HydroelasticTractionCalculator<T>::CalcTractionAtQHelper(
    const Data& data, int face_index, const T& e, const Vector3<T>& nhat_W,
    double dissipation, double mu_coulomb, const Vector3<T>& p_WQ) const {
  HydroelasticQuadraturePointData<T> traction_data;

  // Set entries that do not require computation first.
  traction_data.face_index = face_index;
  traction_data.p_WQ = p_WQ;

  // Get the relative spatial velocity at the point Q between the
  // two bodies A and B (to which M and N are affixed, respectively) by
  // subtracting the spatial velocity of a point (Bq) coincident with p_WQ on
  // Body B from the spatial velocity of a point (Aq) coincident with p_WQ on
  // Body A.

  // First compute the spatial velocity of Body A at Aq.
  const Vector3<T> p_AoAq_W = traction_data.p_WQ - data.X_WA.translation();
  const SpatialVelocity<T> V_WAq = data.V_WA.Shift(p_AoAq_W);

  // Next compute the spatial velocity of Body B at Bq.
  const Vector3<T> p_BoBq_W = traction_data.p_WQ - data.X_WB.translation();
  const SpatialVelocity<T> V_WBq = data.V_WB.Shift(p_BoBq_W);

  // Finally compute the relative velocity of Frame Aq relative to Frame Bq,
  // expressed in the world frame, and then the translational component of this
  // velocity.
  const SpatialVelocity<T> V_BqAq_W = V_WAq - V_WBq;
  const Vector3<T>& v_BqAq_W = V_BqAq_W.translational();

  // Get the velocity along the normal to the contact surface. Note that a
  // positive value indicates that bodies are separating at Q while a negative
  // value indicates that bodies are approaching at Q.
  const T vn_BqAq_W = v_BqAq_W.dot(nhat_W);

  // Get the damping value (c) from the compliant model dissipation (α).
  // Equation (16) from [Hunt 1975], but neglecting the 3/2 term used for
  // Hertzian contact, yields c = α * e_mn with units of N⋅s/m³.
  const T c = dissipation * e;

  // Determine the normal traction at the point.
  using std::max;
  const T normal_traction = max(e - vn_BqAq_W * c, T(0));

  // Get the slip velocity at the point.
  traction_data.vt_BqAq_W = v_BqAq_W - nhat_W * vn_BqAq_W;

  // We write our regularized model of friction as:
  //   fₜ = −μᵣ(‖vₜ‖) vₜ/‖vₜ‖ fₙ                                             (1)
  // where the regularized friction coefficient is computed as:
  //   μᵣ(‖vₜ‖) = μ 2/π atan(‖vₜ‖/vₛ)                                        (2)
  // Even though the above expressions are continuous and differentiable at
  // vₜ = 0 (that is the limits from the left and from the right exist and are
  // unique), we cannot evaluate them "numerically" at vₜ = 0 since
  // this leads to division by zero. We need to come up with a mathematically
  // equivalent expression that can be evaluated numerically at vₜ = 0.
  // We start with the substitution of (2) into (1) to get:
  //   fₜ = −μ 2/π atan(‖vₜ‖/vₛ)vₜ/‖vₜ‖ fₙ                                   (3)
  // We now divide the numerator and denominator of Eq. (3) by vₛ (that is, we
  // effectively multiply by 1) to write:
  //   fₜ = −μ 2/π atan(‖vₜ‖/vₛ)(vₜ/vₛ)/(‖vₜ‖/vₛ)fₙ                          (4)
  // Finally we make the substitution x = ‖vₜ‖/vₛ and group together the terms
  // in x to get:
  //   fₜ =−μ 2/π vₜ/vₛ fₙ (atan(x)/x)
  // We then make the observation that the function atan(x)/x is continuosly
  // differentiable (that is the limits from both sides are exist and are
  // unique) and therefore we can write a custom implementation that avoids
  // division by zero at x = 0. This custom implementation is provided by
  // CalcAtanXOverXFromXSquared().
  const Vector3<T>& vt_BqAq_W = traction_data.vt_BqAq_W;
  const double vs_squared = vslip_regularizer_ * vslip_regularizer_;
  const T x_squared = vt_BqAq_W.squaredNorm() / vs_squared;
  const T regularized_friction = (2.0 / M_PI) * mu_coulomb * normal_traction *
                                 CalcAtanXOverXFromXSquared(x_squared) /
                                 vslip_regularizer_;  // [Ns/m].
  const Vector3<T> ft_Aq_W = -regularized_friction * vt_BqAq_W;

  // Compute the traction.
  traction_data.traction_Aq_W = nhat_W * normal_traction + ft_Aq_W;

  return traction_data;
}

template <typename T>
T HydroelasticTractionCalculator<T>::CalcAtanXOverXFromXSquared(const T& x2) {
  // We are protecting the computation near x = 0 specifically so that
  // numerical values (say double and AutoDiffXd) do not lead to ill-formed
  // expressions with divisions by zero.
  constexpr double x_cuttoff = 0.12;
  constexpr double x_cutoff_squared = x_cuttoff * x_cuttoff;
  if (x2 <= x_cutoff_squared) {
    // We use the Taylor expansion of f(x)=atan(x)/x below a given cutoff
    // x_cutoff, since neither atan(x)/x nor its automatic derivatives with
    // AutodiffXd can be evaluated at x = 0. However, f(x) is well defined
    // mathematically given its limits from left and right exist. Choosing
    // the value of x_cutoff and the number of terms is done to minimize the
    // amount of round-off errors. We estimated these values by comparing
    // against reference values computed with Variable Precision Arithmetic.
    // For further details please refer to Drake issue #15029 documenting this
    // process.

    // clang-format off
      return 1. -
             x2 * (1. / 3. -
             x2 * (1. / 5. -
             x2 * (1. / 7. -
             x2 * (1. / 9. -
             x2 * (1. / 11. -
             x2 * (1. / 13. -
             x2 * (1. / 15. -
             x2 / 17.)))))));
    // clang-format on
  }
  using std::atan;
  using std::sqrt;
  const T x = sqrt(x2);
  return atan(x) / x;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::internal::HydroelasticTractionCalculator)
