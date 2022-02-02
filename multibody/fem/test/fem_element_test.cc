#include "drake/multibody/fem/fem_element.h"

#include <gtest/gtest.h>

#include "drake/multibody/fem/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace test {
namespace {

using T = DummyElementTraits::T;
const ElementIndex kZeroIndex = ElementIndex(0);
const std::array<NodeIndex, DummyElementTraits::num_nodes> kNodeIndices = {
    {NodeIndex(0), NodeIndex(1), NodeIndex(2), NodeIndex(3)}};
const DummyElementTraits::ConstitutiveModel kConstitutiveModel(5e4, 0.4);
const DampingModel<T> kDampingModel(0.01, 0.02);
constexpr int kNumDofs = DummyElementTraits::num_dofs;

class FemElementTest : public ::testing::Test {
 protected:
  /* Default values for the state. */
  static VectorX<double> q() {
    Vector<double, kNumDofs> q;
    q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2;
    return q;
  }
  static VectorX<double> v() {
    Vector<double, kNumDofs> v;
    v << 1.1, 1.2, 2.3, 2.4, 2.5, 2.6, 2.7, 1.8, 1.9, 2.0, 2.1, 2.2;
    return v;
  }
  static VectorX<double> a() {
    Vector<double, kNumDofs> a;
    a << 2.1, 2.2, 3.3, 3.4, 3.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2;
    return a;
  }

  /* FemElement under test. */
  DummyElement element_{kZeroIndex, kNodeIndices, kConstitutiveModel,
                        kDampingModel};
  FemStateImpl<DummyElement> state_{q(), v(), a()};
};

TEST_F(FemElementTest, Constructor) {
  EXPECT_EQ(element_.node_indices(), kNodeIndices);
  EXPECT_EQ(element_.element_index(), kZeroIndex);
}

/* The following tests confirm that CalcResidual(), AddScaledStiffnessMatrix,
 AddScaledDampingMatrix(), DoAddScaledMassMatrix(), correctly invoke their
 DoCalc and DoAdd counterparts. We confirm this with a custom subclass of
 FemElement whose implementation returns/adds a specific value. */
TEST_F(FemElementTest, Residual) {
  Vector<T, DummyElementTraits::num_dofs> residual;
  element_.CalcResidual(state_, &residual);
  const Vector<T, kNumDofs> zero_vector = Vector<T, kNumDofs>::Zero();
  EXPECT_EQ(residual, zero_vector);

  state_.SetPositions(zero_vector);
  state_.SetVelocities(zero_vector);
  state_.SetAccelerations(zero_vector);
  element_.CalcResidual(state_, &residual);
  EXPECT_EQ(residual, element_.dummy_residual());
}

TEST_F(FemElementTest, StiffnessMatrix) {
  Eigen::Matrix<T, DummyElementTraits::num_dofs, DummyElementTraits::num_dofs>
      K;
  K.setZero();
  const T scale = 3.14;
  element_.AddScaledStiffnessMatrix(state_, scale, &K);
  EXPECT_EQ(K, scale * element_.dummy_stiffness_matrix());
}

TEST_F(FemElementTest, DampingMatrix) {
  Eigen::Matrix<T, DummyElementTraits::num_dofs, DummyElementTraits::num_dofs>
      D;
  D.setZero();
  const T scale = 3.14;
  element_.AddScaledDampingMatrix(state_, scale, &D);
  EXPECT_EQ(D, scale * element_.dummy_damping_matrix());
}

/* Test that CalcMassMatrix() is calling the expected DoCalcMassMatrix(). */
TEST_F(FemElementTest, MassMatrix) {
  Eigen::Matrix<T, DummyElementTraits::num_dofs, DummyElementTraits::num_dofs>
      M;
  M.setZero();
  const T scale = 3.14;
  element_.AddScaledMassMatrix(state_, scale, &M);
  EXPECT_EQ(M, scale * element_.dummy_mass_matrix());
}

}  // namespace
}  // namespace test
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
