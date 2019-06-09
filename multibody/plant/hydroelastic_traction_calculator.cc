#include "drake/multibody/plant/hydroelastic_traction_calculator.h"

#include <algorithm>

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

  // Get the transformation of the geometry for M to the world frame.
  const auto& query_object = plant.get_geometry_query_input_port().
      template Eval<geometry::QueryObject<T>>(context);
  X_WM_ = query_object.X_WG(surface->id_M());

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

// Computes the spatial forces on the two bodies due to the traction at the
// given contact point.
// @param data computed once for each pair of geometries.
// @param p_WQ the offset vector from the origin of the world frame to the
//        contact point, expressed in the world frame.
// @param traction_Aq_W the traction vector applied to Body A at Point Q,
//        expressed in the world frame, where Body A is the body that
//        `surface.M_id()` is attached to.
// @param[out] F_Ao_W on return, the spatial force (due to the traction) that
//             acts at the origin of the frame of Body A (i.e., that affixed to
//             `surface.M_id()`).
// @param[out] F_Bo_W on return, the spatial force (due to the traction) that
//             acts at the origin of the frame of Body B (i.e., that affixed
//             to `surface.N_id()`).
template <typename T>
void HydroelasticTractionCalculator<T>::
    ComputeSpatialForcesAtBodyOriginsFromTraction(
        const HydroelasticTractionCalculatorData& data,
        const Vector3<T>& p_WQ, const Vector3<T>& traction_Aq_W,
        SpatialForce<T>* F_Ao_W, SpatialForce<T>* F_Bo_W) const {
  // Set the two vectors from the contact point to the two body frames, all
  // expressed in the world frame.
  const Vector3<T> p_QAo_W = data.X_WA().translation() - p_WQ;
  const Vector3<T> p_QBo_W = data.X_WB().translation() - p_WQ;

  // Convert the traction to a momentless-spatial force (i.e., without
  // changing the point of application). This force will be applied to one
  // body and the (negated) reaction force will be applied to the other.
  SpatialForce<T> F_Q_W(Vector3<T>(0, 0, 0), traction_Aq_W);
  *F_Ao_W = F_Q_W.Shift(p_QAo_W);
  *F_Bo_W = (-F_Q_W).Shift(p_QBo_W);
}

template <typename T>
Vector3<T> HydroelasticTractionCalculator<T>::CalcTractionAtPoint(
    const HydroelasticTractionCalculatorData& data,
    SurfaceFaceIndex face_index,
    const typename SurfaceMesh<T>::Barycentric& Q_barycentric,
    double dissipation, double mu_coulomb, Vector3<T>* p_WQ) const {
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
