#pragma once

#include <array>
#include <vector>

#include "drake/multibody/fem/damping_model.h"
#include "drake/multibody/fem/fem_element.h"
#include "drake/multibody/fem/linear_constitutive_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {

/* The traits for the DummyElement. In this case, all of the traits are unique
 values so we can detect that each value is used in the expected context. */
struct DummyElementTraits {
  using T = double;
  /* See `DoComputeData` below on how this dummy data is updated. */
  struct Data {
    double value{0};
  };
  static constexpr int num_quadrature_points = 1;
  static constexpr int num_natural_dimension = 2;
  static constexpr int num_nodes = 4;
  static constexpr int num_dofs = 12;
  using ConstitutiveModel = LinearConstitutiveModel<T, num_quadrature_points>;
};

/* A simple FemElement implementation. The calculation methods are implemented
 as returning a fixed value (which can independently be accessed by calling
 the corresponding dummy method -- e.g., CalcResidual() should return the
 value in residual(). */
class DummyElement final : public FemElement<DummyElement, DummyElementTraits> {
 public:
  using Base = FemElement<DummyElement, DummyElementTraits>;
  using Traits = DummyElementTraits;
  using ConstitutiveModel = typename Traits::ConstitutiveModel;
  using T = typename Base::T;
  static constexpr int kNumDofs = Traits::num_dofs;

  DummyElement(FemElementIndex element_index,
               const std::array<FemNodeIndex, Traits::num_nodes>& node_indices,
               const ConstitutiveModel& constitutive_model,
               const DampingModel<T>& damping_model)
      : Base(element_index, node_indices, constitutive_model, damping_model) {}

  /* Provides a fixed return value for CalcResidual(). */
  static Vector<T, kNumDofs> residual() {
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
    // A + A^T is guaranteed PSD. Adding the identity matrix to it makes it SPD.
    return (A + A.transpose()) +
           Eigen::Matrix<T, kNumDofs, kNumDofs>::Identity();
  }

  /* Provides a fixed return value for CalcStiffnessMatrix(). */
  static Eigen::Matrix<T, kNumDofs, kNumDofs> stiffness_matrix() {
    return 1.23 * MakeSpdMatrix();
  }

  /* Provides a fixed return value for CalcDampingMatrix(). */
  static Eigen::Matrix<T, kNumDofs, kNumDofs> damping_matrix() {
    return 4.56 * MakeSpdMatrix();
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
    typename Traits::Data data;
    data.value = state.GetPositions()(state_dofs - 1);
    data.value += state.GetVelocities()(state_dofs - 1);
    data.value += state.GetAccelerations()(state_dofs - 1);
    return data;
  }

  /* Implements FemElement::CalcResidual().
   The residual is equal to a dummy nonzero value if the states are all zero.
   Otherwise the residual is zero.*/
  void DoCalcResidual(const FemState<T>& state,
                      EigenPtr<Vector<T, kNumDofs>> residual) const {
    if (state.GetPositions().norm() == 0.0 &&
        state.GetVelocities().norm() == 0.0 &&
        state.GetAccelerations().norm() == 0.0) {
      *residual = this->residual();
    } else {
      residual->setZero();
    }
  }

  /* Implements FemElement::AddScaledStiffnessMatrix(). */
  void DoAddScaledStiffnessMatrix(
      const FemState<T>&, const T& scale,
      EigenPtr<Eigen::Matrix<T, kNumDofs, kNumDofs>> K) const {
    *K += scale * stiffness_matrix();
  }

  /* Implements FemElement::AddScaledDampingMatrix(). */
  void DoAddScaledDampingMatrix(
      const FemState<T>&, const T& scale,
      EigenPtr<Eigen::Matrix<T, kNumDofs, kNumDofs>> D) const {
    *D += scale * damping_matrix();
  }

  /* Implements FemElement::AddScaledMassMatrix(). */
  void DoAddScaledMassMatrix(
      const FemState<T>&, const T& scale,
      EigenPtr<Eigen::Matrix<T, kNumDofs, kNumDofs>> M) const {
    *M += scale * mass_matrix();
  }
};

}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
