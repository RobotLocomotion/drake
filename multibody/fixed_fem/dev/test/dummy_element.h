#pragma once

#include <array>

#include "drake/multibody/fixed_fem/dev/element_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/fem_element.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace test {
/* The traits for the DummyElement. In this case, all of the traits are unique
 values so we can detect that each value is used in the expected context. */
struct DummyElementTraits {
  using T = double;
  struct Data {
    double value{0};
  };
  static constexpr int kNumNodes = 2;
  static constexpr int kNumQuadraturePoints = 3;
  static constexpr int kNaturalDimension = 4;
  static constexpr int kSpatialDimension = 5;
  static constexpr int kSolutionDimension = 6;
  static constexpr int kNumDofs = 7;
  static constexpr int kOdeOrder = 1;
};
/* A simple FemElement implementation. The calculation methods are implemented
 as returning a fixed value (which can independently be accessed by calling
 the corresponding dummy_* method -- e.g., CalcResidual() should return the
 value in dummy_residual(). */
class DummyElement final : public FemElement<DummyElement, DummyElementTraits> {
 public:
  using Base = FemElement<DummyElement, DummyElementTraits>;
  using T = typename Base::T;

  DummyElement(ElementIndex element_index,
               const std::array<NodeIndex, Traits::kNumNodes>& node_indices)
      : Base(element_index, node_indices) {}

  /* Provides a fixed return value for CalcResidual(). */
  static Vector<T, Traits::kNumDofs> dummy_residual() {
    return Vector<T, Traits::kNumDofs>::Constant(1.23456);
  }

  /* Provides a fixed return value for CalcStiffnessMatrix(). */
  static Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>
  dummy_stiffness_matrix() {
    return Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>::Constant(1.23);
  }

  /* Provides a fixed return value for CalcDampingMatrix(). */
  static Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>
  dummy_damping_matrix() {
    return Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>::Constant(4.56);
  }

  /* Provides a fixed return value for CalcMassMatrix(). */
  static Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>
  dummy_mass_matrix() {
    return Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>::Constant(7.89);
  }

  /* Provides a fixed value for the `Data` for `ComputeData()`. */
  static Traits::Data dummy_data() { return {1.732}; }

 private:
  /* Friend the base class so that the interface in the CRTP base class can
   access the private implementations of this class. */
  friend Base;

  /* Implements FemElement::ComputeData(). */
  Traits::Data DoComputeData(const FemState<DummyElement>& state) const {
    return dummy_data();
  }

  /* Implements FemElement::CalcResidual(). */
  void DoCalcResidual(const FemState<DummyElement>& state,
                      EigenPtr<Vector<T, Traits::kNumDofs>> residual) const {
    *residual = dummy_residual();
  }

  /* Implements FemElement::CalcStiffnessMatrix(). */
  void DoCalcStiffnessMatrix(
      const FemState<DummyElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    *K = dummy_stiffness_matrix();
  }

  /* Implements FemElement::CalcDampingMatrix(). */
  void DoCalcDampingMatrix(
      const FemState<DummyElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> D) const {
    *D = dummy_damping_matrix();
  }

  /* Implements FemElement::CalcMassMatrix(). */
  void DoCalcMassMatrix(
      const FemState<DummyElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> M) const {
    *M = dummy_mass_matrix();
  }
};
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
