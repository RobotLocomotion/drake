#include "drake/multibody/fixed_fem/dev/fem_element.h"

#include <gtest/gtest.h>

#include "drake/multibody/fixed_fem/dev/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace test {
namespace {
const ElementIndex kElementIndex = ElementIndex(0);
const std::array<NodeIndex, 3> kNodeIndices = {
    {NodeIndex(0), NodeIndex(1), NodeIndex(2)}};

class FemElementTest : public ::testing::Test {
 protected:
  using T = DummyElementTraits::T;

  static FemState<DummyElement> MakeFemState() {
    return FemState<DummyElement>(q(), qdot());
  }

  /* Default values for the state. */
  static VectorX<double> q() { return Vector3<double>(0.1, 0.2, 0.3); }
  static VectorX<double> qdot() { return Vector3<double>(0.3, 0.4, 0.5); }

  /* FemElement under test. */
  DummyElement element_{kElementIndex, kNodeIndices};
};

TEST_F(FemElementTest, Basic) {
  EXPECT_EQ(element_.natural_dimension(),
            DummyElementTraits::kNaturalDimension);
  EXPECT_EQ(element_.spatial_dimension(),
            DummyElementTraits::kSpatialDimension);
  EXPECT_EQ(element_.solution_dimension(),
            DummyElementTraits::kSolutionDimension);
  EXPECT_EQ(element_.num_quadrature_points(),
            DummyElementTraits::kNumQuadraturePoints);
  EXPECT_EQ(element_.num_nodes(), DummyElementTraits::kNumNodes);
  EXPECT_EQ(element_.num_dofs(), DummyElementTraits::kNumDofs);
  EXPECT_EQ(element_.ode_order(), DummyElementTraits::kODEOrder);
  EXPECT_EQ(element_.node_indices(), kNodeIndices);
  EXPECT_EQ(element_.element_index(), kElementIndex);
  EXPECT_EQ(element_.MakeElementCacheEntry(),
            DummyElementCacheEntry(kElementIndex));
}

TEST_F(FemElementTest, Residual) {
  FemState<DummyElement> state = MakeFemState();
  Vector<T, DummyElementTraits::kNumDofs> residual;
  element_.CalcResidual(state, &residual);
  EXPECT_EQ(residual, element_.dummy_residual());
}

TEST_F(FemElementTest, StiffnessMatrix) {
  FemState<DummyElement> state = MakeFemState();
  Eigen::Matrix<T, DummyElementTraits::kNumDofs, DummyElementTraits::kNumDofs>
      K;
  element_.CalcStiffnessMatrix(state, &K);
  EXPECT_EQ(K, element_.dummy_stiffness_matrix());
}

TEST_F(FemElementTest, DampingMatrix) {
  FemState<DummyElement> state = MakeFemState();
  Eigen::Matrix<T, DummyElementTraits::kNumDofs, DummyElementTraits::kNumDofs>
      D;
  element_.CalcDampingMatrix(state, &D);
  EXPECT_EQ(D, element_.dummy_damping_matrix());
}

TEST_F(FemElementTest, MassMatrix) {
  FemState<DummyElement> state = MakeFemState();
  Eigen::Matrix<T, DummyElementTraits::kNumDofs, DummyElementTraits::kNumDofs>
      M;
  element_.CalcMassMatrix(state, &M);
  EXPECT_EQ(M, element_.dummy_mass_matrix());
}
}  // namespace
}  // namespace test
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
