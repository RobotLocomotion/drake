#pragma once

#include <array>
#include <utility>
#include <vector>

#include "drake/multibody/fem/corotated_model.h"
#include "drake/multibody/fem/damping_model.h"
#include "drake/multibody/fem/fem_element.h"
#include "drake/multibody/fem/linear_constitutive_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Forward declaration needed for defining the Traits below. */
template <bool is_linear>
class DummyElement;

struct DummyData {
  using T = double;
  static constexpr int num_dofs = 12;
  double value{0};
  Vector<T, num_dofs> element_q;
  Vector<T, num_dofs> element_v;
  Vector<T, num_dofs> element_a;
};

/* The traits for the DummyElement. In this case, all of the traits are unique
 values so we can detect that each value is used in the expected context. */
template <bool is_linear>
struct FemElementTraits<DummyElement<is_linear>> {
  using T = double;
  static constexpr int num_quadrature_points = 1;
  static constexpr int num_natural_dimension = 2;
  static constexpr int num_nodes = 4;
  /* See `DoComputeData` below on how this dummy data is updated. */
  using Data = DummyData;
  static constexpr int num_dofs = Data::num_dofs;

  using ConstitutiveModel =
      std::conditional_t<is_linear, LinearConstitutiveModel<T>,
                         CorotatedModel<T>>;
};

/* A simple FemElement implementation. The calculation methods are implemented
 as returning a fixed value (which can independently be accessed by calling
 the corresponding dummy method -- e.g., CalcInverseDynamics() should return
 the value in inverse_dynamics_force().
 @tparam is_linear If true, FemModels consisting of this element returns true
 for is_linear(). */
template <bool is_linear>
class DummyElement final : public FemElement<DummyElement<is_linear>> {
 public:
  using Base = FemElement<DummyElement>;
  using Traits = FemElementTraits<DummyElement>;
  using ConstitutiveModel = typename Traits::ConstitutiveModel;
  using T = typename Base::T;
  using Data = typename Traits::Data;
  static constexpr int kNumDofs = Traits::num_dofs;

  DummyElement(const std::array<FemNodeIndex, Traits::num_nodes>& node_indices,
               ConstitutiveModel constitutive_model,
               DampingModel<T> damping_model)
      : Base(node_indices, std::move(constitutive_model),
             std::move(damping_model)) {}

  /* Provides a fixed return value for CalcInverseDynamics(). */
  static Vector<T, kNumDofs> inverse_dynamics_force() {
    return Vector<T, kNumDofs>::Constant(1.23456);
  }

  /* Creates an arbitrary SPD matrix. */
  static Eigen::Matrix<T, kNumDofs, kNumDofs> MakeSpdMatrix() {
    Eigen::Matrix<T, kNumDofs, kNumDofs> A;
    for (int i = 0; i < kNumDofs; ++i) {
      for (int j = 0; j < kNumDofs; ++j) {
        A(i, j) = 2.7 * i + 3.1 * j;
      }
    }
    // A * A^T is guaranteed PSD. Adding the identity matrix to it makes it SPD.
    return (A * A.transpose()) +
           Eigen::Matrix<T, kNumDofs, kNumDofs>::Identity();
  }

  /* Provides a fixed return value for CalcStiffnessMatrix(). */
  static Eigen::Matrix<T, kNumDofs, kNumDofs> stiffness_matrix() {
    return 1.23 * MakeSpdMatrix();
  }

  /* Provides a fixed return value for CalcMassMatrix(). */
  static Eigen::Matrix<T, kNumDofs, kNumDofs> mass_matrix() {
    return 7.89 * MakeSpdMatrix();
  }

 private:
  /* Friend the base class so that the interface in the CRTP base class can
   access the private implementations of this class. */
  friend Base;

  /* Implements FemElement::ComputeData(). Returns the sum of the last entries
   in each state. */
  typename Traits::Data DoComputeData(const FemState<T>& state) const {
    const int state_dofs = state.num_dofs();
    const auto& q = state.GetPositions();
    const auto& v = state.GetVelocities();
    const auto& a = state.GetAccelerations();
    typename Traits::Data data;
    data.value = q(state_dofs - 1);
    data.value += v(state_dofs - 1);
    data.value += a(state_dofs - 1);
    data.element_q = this->ExtractElementDofs(q);
    data.element_v = this->ExtractElementDofs(v);
    data.element_a = this->ExtractElementDofs(a);
    return data;
  }

  /* Implements FemElement::CalcInverseDynamics().
   The inverse dynamics force is equal to a dummy nonzero value if the element
   has zero velocity and zero acceleration. Otherwise the force is zero.*/
  void DoCalcInverseDynamics(
      const Data& data, EigenPtr<Vector<T, kNumDofs>> external_force) const {
    if (data.element_v.norm() == 0.0 && data.element_a.norm() == 0.0) {
      *external_force = this->inverse_dynamics_force();
    } else {
      external_force->setZero();
    }
  }

  /* Implements FemElement::AddScaledStiffnessMatrix(). */
  void DoAddScaledStiffnessMatrix(
      const Data&, const T& scale,
      EigenPtr<Eigen::Matrix<T, kNumDofs, kNumDofs>> K) const {
    *K += scale * stiffness_matrix();
  }

  /* Implements FemElement::AddScaledMassMatrix(). */
  void DoAddScaledMassMatrix(
      const Data&, const T& scale,
      EigenPtr<Eigen::Matrix<T, kNumDofs, kNumDofs>> M) const {
    *M += scale * mass_matrix();
  }

  /* Implements FemElement::DoAddScaledExternalForces(). Here we add the force
   density directly to the nodes. Even though this is unphysical (the units
   don't match), this gives us an easy way to test APIs without introducing
   extra complexities. */
  void DoAddScaledExternalForces(const Data& data,
                                 const FemPlantData<T>& plant_data,
                                 const T& scale,
                                 EigenPtr<Vector<T, kNumDofs>> result) const {
    for (int i = 0; i < this->num_nodes; ++i) {
      const auto node_q = data.element_q.template segment<3>(3 * i);
      for (const multibody::ForceDensityField<T>* force_density :
           plant_data.force_density_fields) {
        result->template segment<3>(3 * i) +=
            scale * force_density->EvaluateAt(plant_data.plant_context, node_q);
      }
    }
  }
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
