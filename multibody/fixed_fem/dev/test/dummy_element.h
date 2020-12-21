#pragma once

#include <array>

#include "drake/multibody/fixed_fem/dev/element_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/fem_element.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace test {
// A dummy element cache entry.
class DummyElementCacheEntry final : public ElementCacheEntry {
 public:
  DummyElementCacheEntry(ElementIndex element_index, double data)
      : ElementCacheEntry(element_index), data_(data) {}

  explicit DummyElementCacheEntry(ElementIndex element_index)
      : DummyElementCacheEntry(element_index, 0.0) {}

  double data() const { return data_; }

  bool operator==(const DummyElementCacheEntry& other) const {
    return this->data() == other.data() &&
           this->element_index() == other.element_index();
  }

 private:
  // Some fake data for testing.
  double data_;
};

// Dummy traits for dummy element.
struct DummyElementTraits {
  using T = double;
  using ElementCacheEntryType = DummyElementCacheEntry;
  static constexpr int kNumNodes = 3;
  static constexpr int kNumQuadraturePoints = 1;
  static constexpr int kNaturalDimension = 2;
  static constexpr int kSpatialDimension = 3;
  static constexpr int kSolutionDimension = 3;
  static constexpr int kNumDofs = kSolutionDimension * kNumNodes;
  static constexpr int kODEOrder = 1;
};

// Dummy element.
class DummyElement final : public FemElement<DummyElement, DummyElementTraits> {
 public:
  using Base = FemElement<DummyElement, DummyElementTraits>;
  using T = typename Base::T;
  using Base::num_dofs;

  DummyElement(ElementIndex element_index,
               const std::array<NodeIndex, num_nodes()>& node_indices)
      : Base(element_index, node_indices) {}

  /* Provides a dummy value for the residual. */
  static Vector<T, num_dofs()> dummy_residual() {
    return Vector<T, num_dofs()>::Constant(1.23456);
  }

  /* Provides a dummy value for the stiffness matrix. */
  static Eigen::Matrix<T, num_dofs(), num_dofs()> dummy_stiffness_matrix() {
    return Eigen::Matrix<T, num_dofs(), num_dofs()>::Constant(1.23);
  }

  /* Provides a dummy value for the damping matrix. */
  static Eigen::Matrix<T, num_dofs(), num_dofs()> dummy_damping_matrix() {
    return Eigen::Matrix<T, num_dofs(), num_dofs()>::Constant(4.56);
  }

  /* Provides a dummy value for the mass matrix. */
  static Eigen::Matrix<T, num_dofs(), num_dofs()> dummy_mass_matrix() {
    return Eigen::Matrix<T, num_dofs(), num_dofs()>::Constant(7.89);
  }

 private:
  /* Friend the base class so that the interface in the CRTP base class can
   access the private implementations of this class. */
  friend Base;

  /* Implements FemElement::CalcResidual(). */
  void DoCalcResidual(const FemState<DummyElement>& state,
                      EigenPtr<Vector<T, num_dofs()>> residual) const {
    *residual = dummy_residual();
  }

  /* Implements FemElement::CalcStiffnessMatrix(). */
  void DoCalcStiffnessMatrix(
      const FemState<DummyElement>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> K) const {
    *K = dummy_stiffness_matrix();
  }

  /* Implements FemElement::CalcDampingMatrix(). */
  void DoCalcDampingMatrix(
      const FemState<DummyElement>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> D) const {
    *D = dummy_damping_matrix();
  }

  /* Implements FemElement::CalcMassMatrix(). */
  void DoCalcMassMatrix(
      const FemState<DummyElement>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> M) const {
    *M = dummy_mass_matrix();
  }
};
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
