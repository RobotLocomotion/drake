#pragma once

#include <array>
#include <utility>
#include <vector>

#include "drake/multibody/fem/damping_model.h"
#include "drake/multibody/fem/fem_element.h"
#include "drake/multibody/fem/linear_constitutive_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Forward declaration needed for defining the Traits below. */
class DummyElement;

/* The traits for the DummyElement. In this case, all of the traits are unique
 values so we can detect that each value is used in the expected context. */
template <>
struct FemElementTraits<DummyElement> {
  using T = double;
  static constexpr int num_quadrature_points = 1;
  static constexpr int num_natural_dimension = 2;
  static constexpr int num_nodes = 4;
  static constexpr int num_dofs = 12;
  /* See `DoComputeData` below on how this dummy data is updated. */
  struct Data {
    double value{0};
    Vector<T, num_dofs> element_q;
    Vector<T, num_dofs> element_v;
    Vector<T, num_dofs> element_a;
  };
  using ConstitutiveModel = LinearConstitutiveModel<T, num_quadrature_points>;
};

/* A simple FemElement implementation. The calculation methods are implemented
 as returning a fixed value (which can independently be accessed by calling
 the corresponding dummy method -- e.g., CalcResidual() should return the
 value in residual(). */
class DummyElement final : public FemElement<DummyElement> {
 public:
  using Base = FemElement<DummyElement>;
  using Traits = FemElementTraits<DummyElement>;
  using ConstitutiveModel = typename Traits::ConstitutiveModel;
  using T = typename Base::T;
  static constexpr int kNumDofs = Traits::num_dofs;

  DummyElement(const std::array<FemNodeIndex, Traits::num_nodes>& node_indices,
               ConstitutiveModel constitutive_model,
               DampingModel<T> damping_model)
      : Base(node_indices, std::move(constitutive_model),
             std::move(damping_model)) {}

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

  /* Implements FemElement::CalcResidual().
   The residual is equal to a dummy nonzero value if the element states are all
   zero. Otherwise the residual is zero.*/
  void DoCalcResidual(const Data& data,
                      EigenPtr<Vector<T, kNumDofs>> residual) const {
    if (data.element_q.norm() == 0.0 && data.element_v.norm() == 0.0 &&
        data.element_a.norm() == 0.0) {
      *residual = this->residual();
    } else {
      residual->setZero();
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
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
