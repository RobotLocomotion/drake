#include "drake/multibody/plant/hydroelastic_contact/hydroelastic_traction.h"

#include <algorithm>

#include "drake/math/orthonormal_basis.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"

using drake::geometry::ContactSurface;
using drake::math::ComputeBasisFromAxis;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::multibody::SpatialForce;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace hydroelastic_contact {

// Gets the transform for the body that the given geometry is attached to.
template <class T>
const RigidTransform<T>& HydroelasticTraction<T>::GetTransformFromGeometry(
    const Context<T>& context, geometry::GeometryId geometry_id) const {
  BodyIndex body_index = plant_->GetBodyIndexFromRegisteredGeometryId(
      geometry_id);
  const Body<T>& body = plant_->get_body(body_index);
  return plant_->EvalBodyPoseInWorld(context, body);
}

// Computes the spatial forces on the two bodies due to the traction at the
// given contact point.
template <class T>
void HydroelasticTraction<T>::ShiftTractionToSpatialForcesAtBodyOrigins(
    const Context<T>& context,
    const ContactSurface<T>& surface,
    const Vector3<T>& r_W,
    const Vector3<T>& traction_W,
    SpatialForce<T>* f_Mo_W, SpatialForce<T>* f_No_W) const {
  // Get the transforms for the bodies that the two geometries are attached to.
  const RigidTransform<T>& X_WM = GetTransformFromGeometry(context,
      surface.id_M());
  const RigidTransform<T>& X_WN = GetTransformFromGeometry(context,
      surface.id_N());

  // Set the two vectors from the contact point to the two body frames.
  const Vector3<T> p_rMo_W = X_WM.translation() - r_W;
  const Vector3<T> p_rNo_W = X_WN.translation() - r_W;

  // Transform the traction into a momentless-spatial force (i.e., without
  // changing the point of application). This force will be applied to one
  // body and the (negated) reaction force will be applied to the other.
  SpatialForce<T> f_r_W(Vector3<T>(0, 0, 0), traction_W);
  *f_Mo_W = f_r_W.Shift(p_rMo_W);
  *f_No_W = (-f_r_W).Shift(p_rNo_W);
}

// Determines the point of contact corresponding to the given barycentric
// coordinates.
// @return r_W an offset vector from the world frame to the point of contact,
//         expressed in the world frame.
template <class T>
Vector3<T> HydroelasticTraction<T>::CalcContactPoint(
    const Context<T>& context,
    const ContactSurface<T>& surface,
    geometry::SurfaceFaceIndex face_index,
    const typename geometry::SurfaceMesh<T>::Barycentric&
        r_barycentric_M) const {
  // Get the transform for the body that the geometry "M" is attached to.
  const RigidTransform<T>& X_WM = GetTransformFromGeometry(
      context, surface.id_M());

  // Convert the barycentric coordinate to 3D.
  const auto& mesh = surface.mesh();
  const auto& va = mesh.vertex(mesh.element(face_index).vertex(0));
  const auto& vb = mesh.vertex(mesh.element(face_index).vertex(1));
  const auto& vc = mesh.vertex(mesh.element(face_index).vertex(2));
  const Vector3<T> r_M = va.r_MV() * r_barycentric_M[0] +
      vb.r_MV() * r_barycentric_M[1] + vc.r_MV() * r_barycentric_M[2];
  return X_WM.inverse() * r_M;
}

template <class T>
Vector3<T> HydroelasticTraction<T>::CalcTractionAtPoint(
    const Context<T>& context, const ContactSurface<T>& surface,
    geometry::SurfaceFaceIndex face_index,
    const typename geometry::SurfaceMesh<T>::Barycentric&
        r_barycentric_M, const Vector3<T>& r_W, double dissipation,
    double mu_coulomb) const {
  // Get the bodies that the two geometries are attached to.
  BodyIndex bodyM_index = plant_->GetBodyIndexFromRegisteredGeometryId(
      surface.id_M());
  BodyIndex bodyN_index = plant_->GetBodyIndexFromRegisteredGeometryId(
      surface.id_N());
  const Body<T>& bodyM = plant_->get_body(bodyM_index);
  const Body<T>& bodyN = plant_->get_body(bodyN_index);

  // Get the transforms for the bodies that the two geometries are attached to.
  const RigidTransform<T>& X_WM = GetTransformFromGeometry(
      context, surface.id_M());
  const RigidTransform<T>& X_WN = GetTransformFromGeometry(
      context, surface.id_N());

  // Get the hydroelastic pressure at the point.
  const T E_MN = surface.EvaluateE_MN(face_index, r_barycentric_M);

  // Get the normal, expressed in the global frame, to the contact surface at r.
  const Vector3<T> h_MN_M = surface.EvaluateGrad_h_MN_M(
      face_index, r_barycentric_M);
  const Vector3<T> nhat_MN_M = h_MN_M.normalized();
  const Vector3<T> nhat_MN_W = X_WM.inverse() * nhat_MN_M;

  // Compute an orthonormal basis (the contact frame, C) using the normal.
  Matrix3<T> R_WC = ComputeBasisFromAxis(0, nhat_MN_W);

  // Get the relative spatial velocity at the point r between the
  // two bodies, by subtracting the spatial velocity of a point (Nr)
  // coincident with r_W on Body N from the spatial velocity of a point (Mr)
  // coincident with r_W on Body M.

  // First compute the spatial velocity of Body M at Mr.
  const SpatialVelocity<T>& v_Mo_W = plant_->EvalBodySpatialVelocityInWorld(
    context, bodyM);
  const Vector3<T> p_MoMr_W = r_W - X_WM.translation();
  const SpatialVelocity<T> v_Mr_W = v_Mo_W.Shift(p_MoMr_W);

  // Next compute the spatial velocity of Body N at Nr.
  const SpatialVelocity<T>& v_No_W = plant_->EvalBodySpatialVelocityInWorld(
    context, bodyN);
  const Vector3<T> p_NoNr_W = r_W - X_WN.translation();
  const SpatialVelocity<T> v_Nr_W = v_No_W.Shift(p_NoNr_W);

  // Finally compute the relative velocity at r, expressed in the world frame,
  // and then the translational velocity.
  const SpatialVelocity<T> v_r_W = v_Mr_W - v_Nr_W;
  const Vector3<T>& rdot_MN_W = v_r_W.translational();

  // Get the velocity along the normal to the contact surface. Note that a
  // positive value indicates that bodies are separating at r while a negative
  // value indicates that bodies are approaching at r.
  const T rdot_nhat_MN = rdot_MN_W.dot(nhat_MN_W);

  // Get the damping value (c) from the compliant model dissipation (α).
  // Equation (16) from [Hunt 1975] yields c = 3/2 * α * E_MN.
  const T c = dissipation * 1.5 * E_MN;

  // Determine the normal pressure at the point.
  using std::max;
  const T normal_pressure = max(E_MN - rdot_nhat_MN * c, T(0));

  // Get the slip velocity at the point.
  const Vector3<T> tan1 = R_WC.col(1);
  const Vector3<T> tan2 = R_WC.col(2);
  const Vector2<T> rdot_MN_tan(rdot_MN_W.dot(tan1), rdot_MN_W.dot(tan2));

  // Determine the traction.
  // TODO(edrumwri): Make the regularization parameter a user-settable
  // parameter.
  // The closer that v_reg is to zero, the closer that this will approximate
  // Coulomb friction.
  const double v_reg = 1e-8;            // regularization parameter (m/s).
  const double v_reg_squared = v_reg * v_reg;
  const T squared_rdot_tan = rdot_MN_tan.squaredNorm();
  using std::atan;
  using std::sqrt;

  // Get the slip speed.
  const T norm_rdot_tan = sqrt(squared_rdot_tan);

  // Get the direction of slip.
  const Vector2<T> rdot_hat_tan_MN = rdot_MN_tan / norm_rdot_tan;

  // Compute the traction.
  const T frictional_scalar = mu_coulomb * normal_pressure *
      2.0 / M_PI * atan(squared_rdot_tan / T(v_reg_squared));
  if (frictional_scalar > 0) {
    return R_WC * Vector3<T>(normal_pressure,
        -rdot_hat_tan_MN(0) * frictional_scalar,
        -rdot_hat_tan_MN(1) * frictional_scalar);
  } else {
    return R_WC * Vector3<T>(normal_pressure, 0, 0);
  }
}

}  // namespace hydroelastic_contact
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::hydroelastic_contact::HydroelasticTraction)
