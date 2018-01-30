#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/compliant_material.h"
#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"
#include "drake/multibody/rigid_body_plant/contact_results.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

// 1 cm/s
const double
    CompliantContactModelParameters::kDefaultVStictionTolerance{1e-2};
// 0.2 mm
const double
    CompliantContactModelParameters::kDefaultCharacteristicRadius{2e-4};

template <typename T>
void CompliantContactModel<T>::set_default_material(
    const CompliantMaterial& material) {
  default_material_ = material;
}

template <typename T>
void CompliantContactModel<T>::set_model_parameters(
    const CompliantContactModelParameters& values) {
  DRAKE_DEMAND(values.v_stiction_tolerance > 0 &&
      values.characteristic_radius > 0);
  inv_v_stiction_tolerance_ = 1.0 / values.v_stiction_tolerance;
  characteristic_radius_ = values.characteristic_radius;
}

template <typename T>
VectorX<T> CompliantContactModel<T>::ComputeContactForce(
    const RigidBodyTree<T>& tree,
    const KinematicsCache<T>& kinsol, ContactResults<T>* contacts) const {
  using std::sqrt;

  // TODO(amcastro-tri): get rid of this const_cast.
  // Unfortunately collisionDetect() modifies the collision model in the RBT
  // when updating the collision element poses.
  // TODO(naveenoid) : This method call limits the template instanziation of
  // this class currently to T = double only.
  std::vector<drake::multibody::collision::PointPair> pairs =
      const_cast<RigidBodyTree<T>*>(&tree)->ComputeMaximumDepthCollisionPoints(
          kinsol, true);

  VectorX<T> contact_force(kinsol.getV().rows(), 1);
  contact_force.setZero();
  // TODO(SeanCurtis-TRI): Determine if a distance of zero should be reported
  //  as a zero-force contact.
  for (const auto& pair : pairs) {
    if (pair.distance < 0.0) {  // There is contact.
      CompliantMaterial parameters;
      const double s_a =
          CalcContactParameters(*pair.elementA, *pair.elementB, &parameters);

      // Define the contact point: the pair contains points on the *surfaces* of
      // bodies A and B, given as location vectors measured and expressed in the
      // respective body frames.  For penetration, these points will *not* be
      // coincident. We must define a common contact point at which relative
      // velocity is defined and the force is applied.

      const int body_a_index = pair.elementA->get_body()->get_body_index();
      const int body_b_index = pair.elementB->get_body()->get_body_index();
      // The reported point on A's surface (As) in the world frame (W).
      const Vector3<T> p_WAs =
          kinsol.get_element(body_a_index).transform_to_world * pair.ptA;
      // The reported point on B's surface (Bs) in the world frame (W).
      const Vector3<T> p_WBs =
          kinsol.get_element(body_b_index).transform_to_world * pair.ptB;
      // The point of contact in the world frame.  Interpolate between the two
      // surface points based on relative "squish" (see doxygen for
      // CalcContactParameters() for details). For equal squish
      // this becomes the mean point. But as one element gets all of the squish,
      // the contact point converges to the point on the surface of the _other_
      // object. The use of s_a *may* seem counter-intuitive. I.e., if *all*
      // compression is on element A, we are fully taking the position on
      // element B. The point *on* B is in fact the deepest penetrating point on
      // A, which represents the desired contact point. So, the apparent
      // backwardness is, in fact, correct.
      const Vector3<T> p_WC = (1.0 - s_a) * p_WAs + s_a * p_WBs;

      // The contact point in A's frame.
      const auto X_AW = kinsol.get_element(body_a_index)
                            .transform_to_world.inverse(Eigen::Isometry);
      const Vector3<T> p_AAc = X_AW * p_WC;
      // The contact point in B's frame.
      const auto X_BW = kinsol.get_element(body_b_index)
                            .transform_to_world.inverse(Eigen::Isometry);
      const Vector3<T> p_BBc = X_BW * p_WC;

      const auto JA =
          tree.transformPointsJacobian(kinsol, p_AAc, body_a_index, 0, false);
      const auto JB =
          tree.transformPointsJacobian(kinsol, p_BBc, body_b_index, 0, false);
      // This normal points *from* element B *to* element A.
      const Vector3<T> this_normal = pair.normal;

      // R_WC is a left-multiplied rotation matrix to transform a vector from
      // contact frame (C) to world (W), e.g., v_W = R_WC * v_C.
      const int z_axis = 2;
      const Matrix3<T> R_WC = math::ComputeBasisFromAxis(z_axis, this_normal);
      const auto J = R_WC.transpose() * (JA - JB);  // J = [ D1; D2; n ]

      // TODO(SeanCurtis-TRI): Coordinate with Paul Mitiguy to standardize this
      // notation.
      // The *relative* velocity of the contact point in A relative to that in
      // B, expressed in the contact frame, C.
      const auto v_CBcAc_C = J * kinsol.getV();

      // TODO(SeanCurtis-TRI): Move this documentation to the larger doxygen
      // discussion and simply reference it here.

      // See contact_model_doxygen.h for the details of this contact model.
      // Normal force fN = kx(1 + dẋ). We map the equation to the local
      // variables as follows:
      //  x = -pair.distance -- penetration depth.
      //  ̇ẋ = -v_CBcAc_C(2)  -- change of penetration (in normal direction).
      //  fK = kx -- force due to stiffness (aka elasticity).
      //  fD = fk dẋ -- force due to dissipation.
      //  fN = max(0, fK + fD ) -- total normal force; (skipped if fN < 0).
      //  fF = mu(v) * fN  - friction force magnitude.

      const T x = T(-pair.distance);
      const T x_dot = -v_CBcAc_C(2);

      const T fK = parameters.youngs_modulus() * x * characteristic_radius_;
      const T fD = fK * parameters.dissipation() * x_dot;
      const T fN = fK + fD;
      if (fN <= 0) continue;

      Vector3<T> fA;
      fA(2) = fN;
      // Friction force
      const auto slip_vector = v_CBcAc_C.template head<2>();
      T slip_speed_squared = slip_vector.squaredNorm();
      // Consider a value indistinguishable from zero if it is smaller
      // then 1e-14 and test against that value squared.
      const T kNonZeroSqd = T(1e-14 * 1e-14);
      if (slip_speed_squared > kNonZeroSqd) {
        const T slip_speed = sqrt(slip_speed_squared);
        const T friction_coefficient =
            ComputeFrictionCoefficient(slip_speed, parameters);
        const T fF = friction_coefficient * fN;
        fA.template head<2>() = -(fF / slip_speed) * slip_vector;
      } else {
        fA.template head<2>() << 0, 0;
      }

      // fB is equal and opposite to fA: fB = -fA.
      // Therefore the generalized forces tau_c due to contact are:
      // tau_c = (R_CW * JA)^T * fA + (R_CW * JB)^T * fB = J^T * fA.
      // With J computed as above: J = R_CW * (JA - JB).
      // Since right_hand_side has a negative sign when on the RHS of the
      // system of equations ([H,-J^T] * [vdot;f] + right_hand_side = 0),
      // this term needs to be subtracted.
      contact_force += J.transpose() * fA;
      if (contacts != nullptr) {
        ContactInfo<T>& contact_info = contacts->AddContact(
            pair.elementA->getId(), pair.elementB->getId());

        // TODO(SeanCurtis-TRI): Future feature: test against user-set flag
        // for whether the details should generally be captured or not and
        // make this function dependent.
        std::vector<std::unique_ptr<ContactDetail<T>>> details;
        ContactResultantForceCalculator<T> calculator(&details);

        // This contact model produces responses that only have a force
        // component (i.e., the torque portion of the wrench is zero.)
        // In contrast, other models (e.g., torsional friction model) can
        // also introduce a "pure torque" component to the wrench.
        const Vector3<T> force = R_WC * fA;
        const Vector3<T> normal = R_WC.template block<3, 1>(0, 2);

        calculator.AddForce(p_WC, normal, force);

        contact_info.set_resultant_force(calculator.ComputeResultant());
        // TODO(SeanCurtis-TRI): As with previous note, this line depends
        // on the eventual instantiation of the user-set flag for accumulating
        // contact details.
        contact_info.set_contact_details(move(details));
      }
    }
  }
  if (contacts != nullptr) {
    contacts->set_generalized_contact_force(contact_force);
  }
  return contact_force;
}

template <typename T>
T CompliantContactModel<T>::ComputeFrictionCoefficient(
    const T& v_tangent_BAc,
    const CompliantMaterial& parameters) const {
  DRAKE_ASSERT(v_tangent_BAc >= 0);
  const double mu_s = parameters.static_friction();
  const double mu_d = parameters.dynamic_friction();
  const T v = v_tangent_BAc * inv_v_stiction_tolerance_;
  if (v >= 3) {
    return mu_d;
  } else if (v >= 1) {
    return mu_s - (mu_s - mu_d) * step5((v - 1) / 2);
  } else {
    return mu_s * step5(v);
  }
}

template <typename T>
T CompliantContactModel<T>::step5(const T& x) {
  DRAKE_ASSERT(0 <= x && x <= 1);
  const T x3 = x * x * x;
  return x3 * (10 + x * (6 * x - 15));  // 10x³ - 15x⁴ + 6x⁵
}

template <typename T>
double CompliantContactModel<T>::CalcContactParameters(
    const multibody::collision::Element& a,
    const multibody::collision::Element& b,
    CompliantMaterial* parameters) const {
  DRAKE_DEMAND(parameters);

  const double kE = default_material_.youngs_modulus();
  const double kD = default_material_.dissipation();
  const double kMuS = default_material_.static_friction();
  const double kMuD = default_material_.dynamic_friction();

  // For the details on these relationships, see contact_model_doxygen.h at
  // @ref per_object_contact.

  const auto& paramA = a.compliant_material();
  const auto& paramB = b.compliant_material();
  const double k_a = paramA.youngs_modulus(kE);
  const double k_b = paramB.youngs_modulus(kE);
  // No divide-by-zero danger; the CompliantMaterial class prevents
  // setting a Young's modulus value that is not strictly positive.
  const double s_a = k_b / (k_a + k_b);
  const double s_b = 1.0 - s_a;
  parameters->set_youngs_modulus(s_a * k_a);
  parameters->set_dissipation(s_a * paramA.dissipation(kD) +
                              s_b * paramB.dissipation(kD));

  // Simple utility to detect 0 / 0. As it is used in this function, denom
  // can only be zero if num is also zero, so we'll simply return zero.
  auto safe_divide = [](double num, double denom) {
    return denom == 0.0 ? 0.0 : num / denom;
  };
  parameters->set_friction(
      safe_divide(
          2 * paramA.static_friction(kMuS) * paramB.static_friction(kMuS),
          paramA.static_friction(kMuS) + paramB.static_friction(kMuS)),
      safe_divide(
          2 * paramA.dynamic_friction(kMuD) * paramB.dynamic_friction(kMuD),
          paramA.dynamic_friction(kMuD) + paramB.dynamic_friction(kMuD)));
  return s_a;
}

// Explicit instantiates on the most common scalar types
template class CompliantContactModel<double>;

}  // namespace systems
}  // namespace drake
