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

template <class T>
const RigidTransform<T>
HydroelasticTractionCalculator<T>::GetTransformFromGeometry(
    const Context<T>& context, geometry::GeometryId geometry_id) const {
  // Need the SceneGraphInspector to query the pose relating the geometry's pose
  // to the pose of the frame it is attached to.
  const auto& query_object = plant_->get_geometry_query_input_port().
      template Eval<geometry::QueryObject<T>>(context);
  const geometry::SceneGraphInspector<T>& inspector = query_object.inspector();
  // TODO(edrumwri): Replace this to use a RigidTransform reference when
  // SceneGraphInspector::X_PG() no longer returns an Isometry3.
  const RigidTransform<T> X_PG =
      RigidTransform<double>(inspector.X_PG(geometry_id)).template cast<T>();

  // Compute the pose relating the geometry's pose to the world frame.
  BodyIndex body_index = plant_->GetBodyIndexFromRegisteredGeometryId(
      geometry_id);
  const Body<T>& body = plant_->get_body(body_index);
  return plant_->EvalBodyPoseInWorld(context, body) * X_PG;
}

template <class T>
void HydroelasticTractionCalculator<T>::
ComputeSpatialForcesAtBodyOriginsFromTraction(
    const Vector3<T>& p_WQ,
    const Vector3<T>& traction_Q_W,
    const RigidTransform<T>& X_WM, const RigidTransform<T>& X_WN,
    SpatialForce<T>* F_Mo_W, SpatialForce<T>* F_No_W) const {
  // Set the two vectors from the contact point to the two body frames, all
  // expressed in the world frame.
  const Vector3<T> p_QMo_W = X_WM.translation() - p_WQ;
  const Vector3<T> p_QNo_W = X_WN.translation() - p_WQ;

  // Convert the traction to a momentless-spatial force (i.e., without
  // changing the point of application). This force will be applied to one
  // body and the (negated) reaction force will be applied to the other.
  SpatialForce<T> F_Q_W(Vector3<T>(0, 0, 0), traction_Q_W);
  *F_Mo_W = F_Q_W.Shift(p_QMo_W);
  *F_No_W = (-F_Q_W).Shift(p_QNo_W);
}

// Determines the point of contact corresponding to the given barycentric
// coordinates. Returns an offset vector from the world frame to the point of
// contact, expressed in the world frame.
template <class T>
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
  const Vector3<T> r_M = va.r_MV() * Q_barycentric_M[0] +
      vb.r_MV() * Q_barycentric_M[1] + vc.r_MV() * Q_barycentric_M[2];
  return X_WM.inverse() * r_M;
}

template <class T>
Vector3<T> HydroelasticTractionCalculator<T>::CalcTractionAtPoint(
    const Context<T>& context, const ContactSurface<T>& surface,
    geometry::SurfaceFaceIndex face_index,
    const typename geometry::SurfaceMesh<T>::Barycentric&
        Q_barycentric_M, double dissipation, double mu_coulomb,
    const RigidTransform<T>& X_WM, const RigidTransform<T>& X_WN,
    Vector3<T>* p_WQ) const {
  // Compute the point of contact in the world frame.
  *p_WQ = CalcContactPoint(surface, face_index, Q_barycentric_M, X_WM);

  // Get the bodies that the two geometries are attached to.
  BodyIndex bodyM_index = plant_->GetBodyIndexFromRegisteredGeometryId(
      surface.id_M());
  BodyIndex bodyN_index = plant_->GetBodyIndexFromRegisteredGeometryId(
      surface.id_N());
  const Body<T>& bodyM = plant_->get_body(bodyM_index);
  const Body<T>& bodyN = plant_->get_body(bodyN_index);

  // Get the "hydroelastic pressure" at the point (in Newtons).
  const T e_mn = surface.EvaluateE_MN(face_index, Q_barycentric_M);

  // Get the normal from M to N, expressed in the global frame, to the contact
  // surface at Q.
  const Vector3<T> h_MN_M = surface.EvaluateGrad_h_MN_M(
      face_index, Q_barycentric_M);
  const Vector3<T> nhat_MN_M = h_MN_M.normalized();
  const Vector3<T> nhat_MN_W = X_WM.rotation().transpose() * nhat_MN_M;

  // Compute an orthonormal basis (the contact frame, C) using the normal.
  Matrix3<T> R_WC = ComputeBasisFromAxis(0, nhat_MN_W);

  // Get the relative spatial velocity at the point Q between the
  // two bodies, by subtracting the spatial velocity of a point (Nq)
  // coincident with p_WQ on Body N from the spatial velocity of a point (Mq)
  // coincident with p_WQ on Body M.

  // First compute the spatial velocity of Body M at Mq.
  const SpatialVelocity<T>& V_WM = plant_->EvalBodySpatialVelocityInWorld(
    context, bodyM);
  const Vector3<T> p_MoMq_W = *p_WQ - X_WM.translation();
  const SpatialVelocity<T> V_WMq = V_WM.Shift(p_MoMq_W);

  // Next compute the spatial velocity of Body N at Nq.
  const SpatialVelocity<T>& V_WN = plant_->EvalBodySpatialVelocityInWorld(
    context, bodyN);
  const Vector3<T> p_NoNq_W = *p_WQ - X_WN.translation();
  const SpatialVelocity<T> V_WNq = V_WN.Shift(p_NoNq_W);

  // Finally compute the relative velocity of Frame Mq relative to Frame Nq,
  // expressed in the world frame, and then the translational component of this
  // velocity.
  const SpatialVelocity<T> V_NqMq_W = V_WMq - V_WNq;
  const Vector3<T>& Qdot_NM_W = V_NqMq_W.translational();

  // Get the velocity along the normal to the contact surface. Note that a
  // positive value indicates that bodies are separating at Q while a negative
  // value indicates that bodies are approaching at Q.
  const T Qdot_nhat_NM = Qdot_NM_W.dot(nhat_MN_W);

  // Get the damping value (c) from the compliant model dissipation (α).
  // Equation (16) from [Hunt 1975], but neglecting the 3/2 term used for
  // Hertzian contact, yields c = α * e_mn with units of N⋅s/m.
  const T c = dissipation * e_mn;

  // Determine the normal pressure at the point.
  using std::max;
  const T normal_pressure = max(e_mn - Qdot_nhat_NM * c, T(0));

  // Get the slip velocity at the point.
  const Vector3<T> tan1 = R_WC.col(1);
  const Vector3<T> tan2 = R_WC.col(2);
  const Vector2<T> Qdot_NM_tan(Qdot_NM_W.dot(tan1), Qdot_NM_W.dot(tan2));

  // Determine the traction using a soft-norm.
  const T squared_Qdot_tan = Qdot_NM_tan.squaredNorm();
  const T norm_Qdot_tan = sqrt(squared_Qdot_tan);
  using std::atan;
  using std::sqrt;
  const T soft_norm_Qdot_tan = sqrt(squared_Qdot_tan +
      vslip_regularizer_ * vslip_regularizer_);

  // Get the regularized direction of slip.
  const Vector2<T> Qdot_hat_tan_NM = Qdot_NM_tan / soft_norm_Qdot_tan;

  // Compute the traction.
  const T frictional_scalar = mu_coulomb * normal_pressure *
      2.0 / M_PI * atan(norm_Qdot_tan / T(vslip_regularizer_));
  return R_WC * Vector3<T>(normal_pressure,
      -Qdot_hat_tan_NM(0) * frictional_scalar,
      -Qdot_hat_tan_NM(1) * frictional_scalar);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::HydroelasticTractionCalculator)
