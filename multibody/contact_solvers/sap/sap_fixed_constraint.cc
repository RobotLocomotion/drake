#include "drake/multibody/contact_solvers/sap/sap_fixed_constraint.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapFixedConstraint<T>::SapFixedConstraint(
    FixedConstraintKinematics<T> kinematics)
    // Since R_WNᵢ is identity for all i, p_PᵢQᵢ_W == p_PᵢQᵢ_Nᵢ, p_BQᵢ_W ==
    // p_BQᵢ_Nᵢ, p_APᵢ_W == p_APᵢ_Nᵢ, and J_W_PᵢQᵢ == J_Nᵢ_PᵢQᵢ.
    : SapHolonomicConstraint<T>(
          typename SapHolonomicConstraint<T>::Kinematics(
              std::move(kinematics.p_PQs_W), std::move(kinematics.J),
              VectorX<T>::Zero(kinematics.p_BQs_W.size())),
          MakeSapHolonomicConstraintParameters(kinematics.p_BQs_W.size()),
          {kinematics.objectA, kinematics.objectB}),
      num_constrained_point_pairs_(kinematics.p_BQs_W.size() / 3),
      p_APs_N_(std::move(kinematics.p_APs_W)),
      p_BQs_N_(std::move(kinematics.p_BQs_W)) {}

template <typename T>
typename SapHolonomicConstraint<T>::Parameters
SapFixedConstraint<T>::MakeSapHolonomicConstraintParameters(
    int num_constraint_equations) {
  // "Near-rigid" regime parameter, see [Castro et al., 2022].
  // TODO(amcastro-tri): consider exposing this parameter.
  constexpr double kBeta = 0.1;

  // Fixed constraints do not have impulse limits, they are bi-lateral
  // constraints. Each fixed point pair introduces three constraint
  // equations.
  constexpr double kInfinity = std::numeric_limits<double>::infinity();
  VectorX<T> gamma_lower =
      VectorX<T>::Constant(num_constraint_equations, -kInfinity);
  VectorX<T> gamma_upper =
      VectorX<T>::Constant(num_constraint_equations, kInfinity);

  VectorX<T> stiffness =
      VectorX<T>::Constant(num_constraint_equations, kInfinity);
  VectorX<T> relaxation_time = VectorX<T>::Zero(num_constraint_equations);

  return typename SapHolonomicConstraint<T>::Parameters{
      std::move(gamma_lower), std::move(gamma_upper), std::move(stiffness),
      std::move(relaxation_time), kBeta};
}

template <typename T>
void SapFixedConstraint<T>::DoAccumulateSpatialImpulses(
    int i, const Eigen::Ref<const VectorX<T>>& gamma,
    SpatialForce<T>* F) const {
  if (i == 0) {
    // Object A.
    // -gamma = gamma_APi_Ni = gamma_APi_W
    // Shift gamma_APi_W = to Ao and add in.
    for (int c = 0; c < num_constrained_point_pairs_; ++c) {
      const SpatialForce<T> gamma_APi_W(Vector3<T>::Zero(),
                                        -gamma.template segment<3>(3 * c));
      const auto p_APi_W = p_APs_N_.template segment<3>(3 * c);
      *F += gamma_APi_W.Shift(-p_APi_W);
    }
  } else {
    DRAKE_DEMAND(i == 1);
    // Object B.
    // gamma = gamma_BQi_Ni == gamma_BQi_W
    // Shift gamma_BQi_W to Bo and add in.
    for (int c = 0; c < num_constrained_point_pairs_; ++c) {
      const SpatialForce<T> gamma_BQi_W(Vector3<T>::Zero(),
                                        gamma.template segment<3>(3 * c));
      const auto p_BQi_W = p_BQs_N_.template segment<3>(3 * c);
      *F += gamma_BQi_W.Shift(-p_BQi_W);
    }
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapFixedConstraint)
