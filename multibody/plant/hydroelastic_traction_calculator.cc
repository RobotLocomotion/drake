#include "drake/multibody/plant/hydroelastic_traction_calculator.h"

#include <algorithm>
#include <utility>

#include "drake/math/orthonormal_basis.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/multibody/triangle_quadrature/triangle_quadrature.h"

namespace drake {

using geometry::ContactSurface;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceMesh;
using math::RigidTransform;
using systems::Context;

namespace multibody {
namespace internal {

template <typename T>
HydroelasticTractionCalculator<T>::HydroelasticTractionCalculatorData::
    HydroelasticTractionCalculatorData(const Context<T>& context,
                                       const MultibodyPlant<T>& plant,
                                       const ContactSurface<T>* surface)
    : surface_(*surface) {
  DRAKE_DEMAND(surface);

  // Get the transform of the geometry for M to the world frame.
  const auto& query_object = plant.get_geometry_query_input_port().
      template Eval<geometry::QueryObject<T>>(context);
  X_WM_ = query_object.X_WG(surface->id_M());

  const Vector3<T>& p_MC = surface_.mesh().centroid();
  p_WC_ = X_WM_ * p_MC;

  // Get the bodies that the two geometries are affixed to. We'll call these
  // A and B.
  const geometry::FrameId frameM_id = query_object.inspector().GetFrameId(
      surface->id_M());
  const geometry::FrameId frameN_id = query_object.inspector().GetFrameId(
      surface->id_N());
  const Body<T>& bodyA = *plant.GetBodyFromFrameId(frameM_id);
  const Body<T>& bodyB = *plant.GetBodyFromFrameId(frameN_id);

  // Get the transformation of the two bodies to the world frame.
  X_WA_ = plant.EvalBodyPoseInWorld(context, bodyA);
  X_WB_ = plant.EvalBodyPoseInWorld(context, bodyB);

  // Get the spatial velocities for the two bodies (at the body frames).
  V_WA_ = plant.EvalBodySpatialVelocityInWorld(context, bodyA);
  V_WB_ = plant.EvalBodySpatialVelocityInWorld(context, bodyB);
}

template <class T>
void HydroelasticTractionCalculator<T>::
ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
    const Context<T>& context,
    const MultibodyPlant<T>& plant,
    const ContactSurface<T>& surface,
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

  // Collect kinematic data once.
  const HydroelasticTractionCalculatorData data(context, plant, &surface);

  // We'll be accumulating force on body A at the surface centroid C,
  // triangle-by-triangle.
  SpatialForce<T> F_Ac_W;
  F_Ac_W.SetZero();

  // Integrate the tractions over all triangles in the contact surface.
  for (geometry::SurfaceFaceIndex i(0); i < surface.mesh().num_faces(); ++i) {
    // Construct the function to be integrated over triangle i.
    // TODO(sherm1) Pull functor creation out of the loop (not a good idea to
    //              create a new functor for every i).
    std::function<SpatialForce<T>(const Vector3<T>&)> traction_Ac_W =
        [this, &data, i, dissipation,
         mu_coulomb](const Vector3<T>& Q_barycentric) {
          Vector3<T> p_WQ;
          const Vector3<T> traction_Aq_W = CalcTractionAtPoint(
              data, i, Q_barycentric, dissipation, mu_coulomb, &p_WQ);
          return ComputeSpatialTractionAtAcFromTractionAtAq(data, p_WQ,
                                                            traction_Aq_W);
        };

    // Compute the integral over the triangle to get a force from the
    // tractions (force/area) at the Gauss points (shifted to C).
    const SpatialForce<T> Fi_Ac_W =  // Force from triangle i.
        TriangleQuadrature<SpatialForce<T>, T>::Integrate(
            traction_Ac_W, gaussian, surface.mesh().area(i));

    // Update the spatial force at body A's origin.
    F_Ac_W += Fi_Ac_W;
  }

  // The spatial force on body A was accumulated at the surface centroid C. We
  // need to shift it to A's origin Ao. The force on body B is equal and
  // opposite to the force on body A, but we want it as if applied at Bo.
  const Vector3<T>& p_WC = data.p_WC();
  const Vector3<T>& p_WAo = data.X_WA().translation();
  const Vector3<T>& p_WBo = data.X_WB().translation();
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
        const HydroelasticTractionCalculatorData& data, const Vector3<T>& p_WQ,
        const Vector3<T>& traction_Aq_W) const {
  // Find the vector from Q to C.
  const Vector3<T> p_QC_W = data.p_WC() - p_WQ;

  // Convert the traction to a momentless spatial traction (i.e., without
  // changing the point of application), then shift to body A's origin which
  // will add a moment. (We're using "Ft" for spatial traction.)
  const SpatialForce<T> Ft_Aq_W(Vector3<T>(0, 0, 0), traction_Aq_W);
  const SpatialForce<T> Ft_Ac_W = Ft_Aq_W.Shift(p_QC_W);
  return Ft_Ac_W;  // Still a traction (force/area).
}

template <typename T>
Vector3<T> HydroelasticTractionCalculator<T>::CalcTractionAtPoint(
    const HydroelasticTractionCalculatorData& data,
    SurfaceFaceIndex face_index,
    const typename SurfaceMesh<T>::Barycentric& Q_barycentric,
    double dissipation, double mu_coulomb, Vector3<T>* p_WQ) const {
  DRAKE_DEMAND(p_WQ != nullptr);
  // Compute the point of contact in the world frame.
  const Vector3<T> p_MQ = data.surface().mesh().CalcCartesianFromBarycentric(
      face_index, Q_barycentric);
  *p_WQ = data.X_WM() * p_MQ;

  // Get the "potential pressure" (in N/m²) at the point as defined in
  // [Elandt 2019]. Note that we drop the _MN suffix here and below, as this
  // suffix can get confused with the identical suffix (used for a different
  // purpose) employed by monogram notation.
  const T E = data.surface().EvaluateE_MN(face_index, Q_barycentric);

  // Get the normal from Geometry M to Geometry N, expressed in the world frame,
  // to the contact surface at Point Q. By extension, this means that the normal
  // points from Body A to Body B.
  const Vector3<T> h_M = data.surface().EvaluateGrad_h_MN_M(
      face_index, Q_barycentric);
  const Vector3<T> nhat_M = h_M.normalized();
  const Vector3<T> nhat_W = data.X_WM().rotation() * nhat_M;

  // Get the relative spatial velocity at the point Q between the
  // two bodies A and B (to which M and N are affixed, respectively) by
  // subtracting the spatial velocity of a point (Bq) coincident with p_WQ on
  // Body B from the spatial velocity of a point (Aq) coincident with p_WQ on
  // Body A.

  // First compute the spatial velocity of Body A at Aq.
  const Vector3<T> p_AoAq_W = *p_WQ - data.X_WA().translation();
  const SpatialVelocity<T> V_WAq = data.V_WA().Shift(p_AoAq_W);

  // Next compute the spatial velocity of Body B at Bq.
  const Vector3<T> p_BoBq_W = *p_WQ - data.X_WB().translation();
  const SpatialVelocity<T> V_WBq = data.V_WB().Shift(p_BoBq_W);

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
  const T c = dissipation * E;

  // Determine the normal traction at the point.
  using std::max;
  const T normal_traction = max(E - vn_BqAq_W * c, T(0));

  // Get the slip velocity at the point.
  const Vector3<T> vt_BqAq_W = v_BqAq_W - nhat_W * vn_BqAq_W;

  // Determine the traction using a soft-norm.
  using std::atan;
  using std::sqrt;
  const T squared_vt = vt_BqAq_W.squaredNorm();
  const T norm_vt = sqrt(squared_vt);
  const T soft_norm_vt = sqrt(squared_vt +
      vslip_regularizer_ * vslip_regularizer_);

  // Get the regularized direction of slip.
  const Vector3<T> vt_hat_BqAq_W = vt_BqAq_W / soft_norm_vt;

  // Compute the traction.
  const T frictional_scalar = mu_coulomb * normal_traction *
      2.0 / M_PI * atan(norm_vt / T(vslip_regularizer_));
  return nhat_W * normal_traction - vt_hat_BqAq_W * frictional_scalar;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

// TODO(edrumwri) instantiate on SymbolicExpression when it no longer
// causes a linker error complaining about an unresolved symbol in SceneGraph.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::internal::HydroelasticTractionCalculator)
