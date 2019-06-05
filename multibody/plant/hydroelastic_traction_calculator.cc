#include "drake/multibody/plant/hydroelastic_traction_calculator.h"

#include <algorithm>

#include "drake/math/orthonormal_basis.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"
#include "drake/multibody/triangle_quadrature/triangle_quadrature.h"

using drake::geometry::ContactSurface;
using drake::math::ComputeBasisFromAxis;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::multibody::SpatialForce;
using drake::systems::Context;

namespace drake {
namespace multibody {

namespace internal {

template <typename T>
HydroelasticTractionCalculatorData<T>::HydroelasticTractionCalculatorData(
    const systems::Context<T>* context,
    const geometry::ContactSurface<T>* surface,
    const MultibodyPlant<T>* plant) :
    context_(context), surface_(surface), plant_(plant) {
  // Get the transformation of the geometry for M to the world frame.
  X_WM_ = GetGeometryTransformationToWorld(surface->id_M());

  // Get the bodies that the two geometries are affixed to. We'll call these
  // A and B.
  BodyIndex bodyA_index = plant->GetBodyIndexFromRegisteredGeometryId(
      surface->id_M());
  BodyIndex bodyB_index = plant->GetBodyIndexFromRegisteredGeometryId(
      surface->id_N());
  const Body<T>& bodyA = plant->get_body(bodyA_index);
  const Body<T>& bodyB = plant->get_body(bodyB_index);

  // Get the transformation of the two bodies to the world frame.
  X_WA_ = plant->EvalBodyPoseInWorld(*context, bodyA);
  X_WB_ = plant->EvalBodyPoseInWorld(*context, bodyB);

  // Get the spatial velocities for the two bodies (at the body frames).
  V_WA_ = plant->EvalBodySpatialVelocityInWorld(*context, bodyA);
  V_WB_ = plant->EvalBodySpatialVelocityInWorld(*context, bodyB);
}

template <typename T>
const RigidTransform<T>
HydroelasticTractionCalculatorData<T>::GetGeometryTransformationToWorld(
    geometry::GeometryId geometry_id) const {
  DRAKE_DEMAND(plant_);

  // Need the SceneGraphInspector to query the pose relating the geometry's pose
  // to the pose of the frame it is affixed to.
  const auto& query_object = plant_->get_geometry_query_input_port().
      template Eval<geometry::QueryObject<T>>(context());
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();
  // TODO(edrumwri): Replace this to use a RigidTransform reference when
  // SceneGraphInspector::X_PG() no longer returns an Isometry3.
  const RigidTransform<T> X_PG =
      RigidTransform<double>(inspector.X_PG(geometry_id)).template cast<T>();

  // Compute the pose relating the geometry's pose to the world frame.
  BodyIndex body_index = plant_->GetBodyIndexFromRegisteredGeometryId(
      geometry_id);
  const Body<T>& body = plant_->get_body(body_index);
  return plant_->EvalBodyPoseInWorld(context(), body) * X_PG;
}

}  // namespace internal

// Computes the spatial forces on the two bodies due to the traction at the
// given contact point.
// @param data computed once for each pair of geometries.
// @param p_WQ the offset vector from the origin of the world frame to the
//        contact point, expressed in the world frame.
// @param traction_Q_W the traction vector at Point Q, expressed in the world
//        frame, that points toward `surface.M_id()`.
// @param[out] F_Ao_W on return, the spatial force (due to the traction) that
//             acts at the origin of the frame of Body A (i.e., that affixed to
//             `surface.M_id()`).
// @param[out] F_Bo_W on return, the spatial force (due to the traction) that
//             acts at the origin of the frame of Body B (i.e., that affixed
//             to `surface.N_id()`).
template <typename T>
void HydroelasticTractionCalculator<T>::
ComputeSpatialForcesAtBodyOriginsFromTraction(
    const internal::HydroelasticTractionCalculatorData<T>& data,
    const Vector3<T>& p_WQ,
    const Vector3<T>& traction_Q_W,
    SpatialForce<T>* F_Ao_W, SpatialForce<T>* F_Bo_W) const {
  // Set the two vectors from the contact point to the two body frames, all
  // expressed in the world frame.
  const Vector3<T> p_QAo_W = data.X_WA().translation() - p_WQ;
  const Vector3<T> p_QBo_W = data.X_WB().translation() - p_WQ;

  // Convert the traction to a momentless-spatial force (i.e., without
  // changing the point of application). This force will be applied to one
  // body and the (negated) reaction force will be applied to the other.
  SpatialForce<T> F_Q_W(Vector3<T>(0, 0, 0), traction_Q_W);
  *F_Ao_W = F_Q_W.Shift(p_QAo_W);
  *F_Bo_W = (-F_Q_W).Shift(p_QBo_W);
}

// Determines the point of contact corresponding to the given barycentric
// coordinates. Returns an offset vector from the world frame to the point of
// contact (Q), expressed in the world frame.
template <typename T>
Vector3<T> HydroelasticTractionCalculator<T>::CalcContactPoint(
    const ContactSurface<T>& surface,
    geometry::SurfaceFaceIndex face_index,
    const typename geometry::SurfaceMesh<T>::Barycentric&
        Q_barycentric_M,
    const RigidTransform<T>& X_WM) const {
  // Convert the barycentric coordinate to 3D.
  const auto& mesh = surface.mesh();
  const auto& va = mesh.vertex(mesh.element(face_index).vertex(0));
  const auto& vb = mesh.vertex(mesh.element(face_index).vertex(1));
  const auto& vc = mesh.vertex(mesh.element(face_index).vertex(2));
  const Vector3<T> r_MQ = va.r_MV() * Q_barycentric_M[0] +
      vb.r_MV() * Q_barycentric_M[1] + vc.r_MV() * Q_barycentric_M[2];
  return X_WM * r_MQ;
}

template <typename T>
Vector3<T> HydroelasticTractionCalculator<T>::CalcTractionAtPoint(
    const internal::HydroelasticTractionCalculatorData<T>& data,
    geometry::SurfaceFaceIndex face_index,
    const typename geometry::SurfaceMesh<T>::Barycentric&
        Q_barycentric_M, double dissipation, double mu_coulomb,
    Vector3<T>* p_WQ) const {
  // Compute the point of contact in the world frame.
  *p_WQ = CalcContactPoint(
      data.surface(), face_index, Q_barycentric_M, data.X_WM());

  // Get the "hydroelastic pressure" at the point (in Newtons).
  const T e_mn = data.surface().EvaluateE_MN(face_index, Q_barycentric_M);

  // Get the normal from M to N, expressed in the global frame, to the contact
  // surface at Q.
  const Vector3<T> h_MN_M = data.surface().EvaluateGrad_h_MN_M(
      face_index, Q_barycentric_M);
  const Vector3<T> nhat_MN_M = h_MN_M.normalized();
  const Vector3<T> nhat_MN_W = data.X_WM().rotation() * nhat_MN_M;

  // Get the relative spatial velocity at the point Q between the
  // two bodies, by subtracting the spatial velocity of a point (Bq)
  // coincident with p_WQ on Body B from the spatial velocity of a point (Aq)
  // coincident with p_WQ on Body A.

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
  const Vector3<T>& v_NM_W = V_BqAq_W.translational();

  // Get the velocity along the normal to the contact surface. Note that a
  // positive value indicates that bodies are separating at Q while a negative
  // value indicates that bodies are approaching at Q.
  const T vn_NM_W = v_NM_W.dot(nhat_MN_W);

  // Get the damping value (c) from the compliant model dissipation (α).
  // Equation (16) from [Hunt 1975], but neglecting the 3/2 term used for
  // Hertzian contact, yields c = α * e_mn with units of N⋅s/m.
  const T c = dissipation * e_mn;

  // Determine the normal pressure at the point.
  using std::max;
  const T normal_pressure = max(e_mn - vn_NM_W * c, T(0));

  // Get the slip velocity at the point.
  const Vector3<T> vt_NM_W = v_NM_W - nhat_MN_W * vn_NM_W;

  // Determine the traction using a soft-norm.
  using std::atan;
  using std::sqrt;
  const T squared_vt = vt_NM_W.squaredNorm();
  const T norm_vt = sqrt(squared_vt);
  const T soft_norm_vt = sqrt(squared_vt +
      vslip_regularizer_ * vslip_regularizer_);

  // Get the regularized direction of slip.
  const Vector3<T> vt_hat_NM_W = vt_NM_W / soft_norm_vt;

  // Compute the traction.
  const T frictional_scalar = mu_coulomb * normal_pressure *
      2.0 / M_PI * atan(norm_vt / T(vslip_regularizer_));
  return nhat_MN_W * normal_pressure - vt_hat_NM_W * frictional_scalar;
}

}  // namespace multibody
}  // namespace drake

// TODO(edrumwri) instantiate these on SymbolicExpression when they no longer
// causes a linker error complaining about an unresolved symbol in SceneGraph.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::internal::HydroelasticTractionCalculatorData)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::HydroelasticTractionCalculator)
