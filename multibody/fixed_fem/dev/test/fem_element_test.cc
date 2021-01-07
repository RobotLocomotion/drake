#include "drake/multibody/fixed_fem/dev/fem_element.h"

#include <gtest/gtest.h>

#include "drake/multibody/fixed_fem/dev/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace test {
namespace {
const ElementIndex kZeroIndex = ElementIndex(0);
const std::array<NodeIndex, DummyElementTraits::kNumNodes> kNodeIndices = {
    {NodeIndex(0), NodeIndex(1)}};

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
  DummyElement element_{kZeroIndex, kNodeIndices};
};

TEST_F(FemElementTest, Constructor) {
  EXPECT_EQ(element_.node_indices(), kNodeIndices);
  EXPECT_EQ(element_.element_index(), kZeroIndex);
}

/* Test that CalcResidual() is calling the expected DoCalcResidual(). */
TEST_F(FemElementTest, Residual) {
  FemState<DummyElement> state = MakeFemState();
  Vector<T, DummyElementTraits::kNumDofs> residual;
  element_.CalcResidual(state, &residual);
  EXPECT_EQ(residual, element_.dummy_residual());
}

/* Test that CalcStiffnessMatrix() is calling the expected
 DoCalcStiffnessMatrix(). */
TEST_F(FemElementTest, StiffnessMatrix) {
  FemState<DummyElement> state = MakeFemState();
  Eigen::Matrix<T, DummyElementTraits::kNumDofs, DummyElementTraits::kNumDofs>
      K;
  element_.CalcStiffnessMatrix(state, &K);
  EXPECT_EQ(K, element_.dummy_stiffness_matrix());
}

/* Test that CalcDampingMatrix() is calling the expected DoCalcDampingMatrix().
 */
TEST_F(FemElementTest, DampingMatrix) {
  FemState<DummyElement> state = MakeFemState();
  Eigen::Matrix<T, DummyElementTraits::kNumDofs, DummyElementTraits::kNumDofs>
      D;
  element_.CalcDampingMatrix(state, &D);
  EXPECT_EQ(D, element_.dummy_damping_matrix());
}

/* Test that CalcMassMatrix() is calling the expected DoCalcMassMatrix(). */
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
