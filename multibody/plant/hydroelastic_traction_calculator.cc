#include "drake/multibody/plant/hydroelastic_traction_calculator.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/math/orthonormal_basis.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/multibody/triangle_quadrature/triangle_quadrature.h"

namespace drake {

using geometry::ContactSurface;
using geometry::SurfaceFace;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceMesh;
using geometry::SurfaceMeshFieldLinear;
using geometry::SurfaceVertexIndex;
using math::RigidTransform;

namespace multibody {
namespace internal {

template <class T>
void HydroelasticTractionCalculator<T>::
ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
    const Data& data,
    double dissipation, double mu_coulomb,
    SpatialForce<T>* F_Ao_W, SpatialForce<T>* F_Bo_W) const {
  DRAKE_DEMAND(F_Ao_W && F_Bo_W);

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
  SpatialForce<T> F_Ac_W;
  F_Ac_W.SetZero();

  // Integrate the tractions over all triangles in the contact surface.
  for (SurfaceFaceIndex i(0); i < data.surface.mesh_W().num_faces(); ++i) {
    // Construct the function to be integrated over triangle i.
    // TODO(sherm1) Pull functor creation out of the loop (not a good idea to
    //              create a new functor for every i).
    std::function<SpatialForce<T>(const Vector3<T>&)> traction_Ac_W =
        [this, &data, i, dissipation,
         mu_coulomb](const Vector3<T>& Q_barycentric) {
          TractionAtPointData traction_output = CalcTractionAtPoint(
              data, i, Q_barycentric, dissipation, mu_coulomb);
          return ComputeSpatialTractionAtAcFromTractionAtAq(
              data, traction_output.p_WQ, traction_output.traction_Aq_W);
        };

    // Compute the integral over the triangle to get a force from the
    // tractions (force/area) at the Gauss points (shifted to C).
    const SpatialForce<T> Fi_Ac_W =  // Force from triangle i.
        TriangleQuadrature<SpatialForce<T>, T>::Integrate(
            traction_Ac_W, gaussian, data.surface.mesh_W().area(i));

    // Update the spatial force at body A's origin.
    F_Ac_W += Fi_Ac_W;
  }

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

// Method for computing traction at a vertex of the contact surface.
template <typename T>
typename HydroelasticTractionCalculator<T>::TractionAtPointData
HydroelasticTractionCalculator<T>::CalcTractionAtVertex(
    const Data& data,
    SurfaceVertexIndex vertex_index,
    double dissipation, double mu_coulomb) const {
  // Compute the point of contact in the world frame.
  const Vector3<T> p_WQ = data.surface.mesh_W().vertex(vertex_index).r_MV();

  const T e = data.surface.EvaluateE_MN(vertex_index);

  const Vector3<T> grad_h_W = data.surface.EvaluateGrad_h_MN_W(vertex_index);
  // TODO(edrumwri): Remove this TODO or update this code when we know whether
  // h_W can ever be zero in expected cases.
  const T norm_grad_h_W = grad_h_W.norm();
  DRAKE_DEMAND(norm_grad_h_W > std::numeric_limits<double>::epsilon() * 10);
  const Vector3<T> nhat_W = grad_h_W / norm_grad_h_W;

  return CalcTractionAtQHelper(data, e, nhat_W, dissipation, mu_coulomb, p_WQ);
}

// Method for computing traction at a point on a face of the contact surface.
// If the point is coincident with a vertex, CalcTractionAtVertex()
// is more efficient.
template <typename T>
typename HydroelasticTractionCalculator<T>::TractionAtPointData
HydroelasticTractionCalculator<T>::CalcTractionAtPoint(
    const Data& data,
    SurfaceFaceIndex face_index,
    const typename SurfaceMesh<T>::Barycentric& Q_barycentric,
    double dissipation, double mu_coulomb) const {
  // Compute the point of contact in the world frame.
  const Vector3<T> p_WQ = data.surface.mesh_W().CalcCartesianFromBarycentric(
      face_index, Q_barycentric);

  const T e = data.surface.EvaluateE_MN(face_index, Q_barycentric);

  const Vector3<T> grad_h_W = data.surface.EvaluateGrad_h_MN_W(
      face_index, Q_barycentric);
  // TODO(edrumwri): Remove this TODO or update this code when we know whether
  // h_W can ever be zero in expected cases.
  const T norm_grad_h_W = grad_h_W.norm();
  DRAKE_DEMAND(norm_grad_h_W > std::numeric_limits<double>::epsilon() * 10);
  const Vector3<T> nhat_W = grad_h_W / norm_grad_h_W;

  return CalcTractionAtQHelper(data, e, nhat_W, dissipation, mu_coulomb, p_WQ);
}

/*
 Helper function for computing the traction at a point, irrespective of whether
 that point is coincident with a vertex or is located at an arbitrary
 point on the contact surface.
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
typename HydroelasticTractionCalculator<T>::TractionAtPointData
HydroelasticTractionCalculator<T>::CalcTractionAtQHelper(
    const Data& data,
    const T& e, const Vector3<T>& nhat_W,
    double dissipation, double mu_coulomb, const Vector3<T>& p_WQ) const {
  TractionAtPointData traction_data;

  // Set p_WQ first.
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

  // Determine the traction using a soft-norm.
  using std::atan;
  using std::sqrt;
  const T squared_vt = traction_data.vt_BqAq_W.squaredNorm();
  const T norm_vt = sqrt(squared_vt);
  const T soft_norm_vt = sqrt(squared_vt +
      vslip_regularizer_ * vslip_regularizer_);

  // Get the regularized direction of slip.
  const Vector3<T> vt_hat_BqAq_W = traction_data.vt_BqAq_W / soft_norm_vt;

  // Compute the traction.
  const T frictional_scalar = mu_coulomb * normal_traction *
      2.0 / M_PI * atan(norm_vt / T(vslip_regularizer_));
  traction_data.traction_Aq_W = nhat_W * normal_traction -
      vt_hat_BqAq_W * frictional_scalar;

  return traction_data;
}

// Creates linearly interpolated fields over the contact surface for use in
// contact reporting.
// @warning The newly created mesh fields retain a pointer to the surface mesh
//         (i.e., `data.surface.mesh_W()`), so that pointer must remain valid
//        while this object is alive.
template <typename T>
typename HydroelasticTractionCalculator<T>::ContactReportingFields
HydroelasticTractionCalculator<T>::CreateReportingFields(
    const HydroelasticTractionCalculator<T>::Data& data,
    double dissipation, double mu_coulomb) const {
  // Alias the contact surface.
  const ContactSurface<T>& surface = data.surface;

  // Compute a value for each vertex.
  std::vector<Vector3<T>> vt_BqAq_W(surface.mesh_W().num_vertices());
  std::vector<Vector3<T>> traction_Aq_W(surface.mesh_W().num_vertices());

  for (SurfaceVertexIndex i(0); i < surface.mesh_W().num_vertices(); ++i) {
    // Compute the traction and the slip velocity at the vertex.
    HydroelasticTractionCalculator<T>::TractionAtPointData output =
        CalcTractionAtVertex(data, i, dissipation, mu_coulomb);
    traction_Aq_W[i] = output.traction_Aq_W;
    vt_BqAq_W[i] = output.vt_BqAq_W;
  }

  // Create the field structure.
  ContactReportingFields fields;
  fields.traction_A_W = std::make_unique<SurfaceMeshFieldLinear<Vector3<T>, T>>(
      "traction", std::move(traction_Aq_W), &surface.mesh_W());
  fields.vslip_AB_W = std::make_unique<SurfaceMeshFieldLinear<Vector3<T>, T>>(
      "slip_velocity", std::move(vt_BqAq_W), &surface.mesh_W());
  return fields;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

// TODO(edrumwri) instantiate on SymbolicExpression when it no longer
// causes a linker error complaining about an unresolved symbol in SceneGraph.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::internal::HydroelasticTractionCalculator)
