#include "drake/multibody/fixed_fem/dev/fem_element.h"

#include <gtest/gtest.h>

#include "drake/multibody/fixed_fem/dev/test/dummy_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace test {
namespace {
const ElementIndex kZeroIndex = ElementIndex(0);
const std::array<NodeIndex, DummyElementTraits<1>::kNumNodes> kNodeIndices = {
    {NodeIndex(0), NodeIndex(1)}};
constexpr double dummy_value = 3.14;

/* An minimal FemElement to test FemElement::CalcFoo() methods. */
class CalcFooElement final
    : public FemElement<CalcFooElement, DummyElementTraits<1>> {
 public:
  using Base = FemElement<CalcFooElement, DummyElementTraits<1>>;
  CalcFooElement(ElementIndex element_index,
                 const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
                 double value)
      : Base(element_index, node_indices), value_(value) {}

  Vector<T, Traits::kNumDofs> expected_residual() const {
    return Vector<T, Traits::kNumDofs>::Constant(value_);
  }

  Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> expected_matrix() const {
    return Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>::Constant(
        value_);
  }

 private:
  friend Base;

  void DoCalcResidual(const FemState<CalcFooElement>& state,
                      EigenPtr<Vector<T, Traits::kNumDofs>> residual) const {
    for (int i = 0; i < Traits::kNumDofs; ++i) {
      if ((*residual)(i) != 0) {
        throw std::runtime_error("Input vector non-zero!");
      }
      (*residual)(i) = value_;
    }
  }

  void DoCalcStiffnessMatrix(
      const FemState<CalcFooElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    VerifyInputIsZeroAndOverwriteWithConstant(K);
  }

  void DoCalcDampingMatrix(
      const FemState<CalcFooElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> D) const {
    VerifyInputIsZeroAndOverwriteWithConstant(D);
  }

  void DoCalcMassMatrix(
      const FemState<CalcFooElement>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> M) const {
    *M = expected_matrix();
  }

  void VerifyInputIsZeroAndOverwriteWithConstant(
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> matrix)
      const {
    for (int i = 0; i < Traits::kNumDofs; ++i) {
      for (int j = 0; j < Traits::kNumDofs; ++j) {
        if ((*matrix)(i, j) != 0) {
          throw std::runtime_error("Input vector non-zero!");
        }
        (*matrix)(i, j) = value_;
      }
    }
  }

  double value_;
};

class FemElementTest : public ::testing::Test {
 protected:
  using T = DummyElementTraits<1>::T;

  /* Default values for the state. */
  static VectorX<double> q() { return Vector3<double>(0.1, 0.2, 0.3); }
  static VectorX<double> qdot() { return Vector3<double>(0.3, 0.4, 0.5); }

  /* FemElement under test. */
  CalcFooElement element_{kZeroIndex, kNodeIndices, dummy_value};
  FemState<CalcFooElement> state_{q(), qdot()};
};

TEST_F(FemElementTest, Constructor) {
  EXPECT_EQ(element_.node_indices(), kNodeIndices);
  EXPECT_EQ(element_.element_index(), kZeroIndex);
}

/* The following tests confirm that CalcResidual(), CalcStiffnessMatrix,
 CalcDampingMatrix(), correctly invoke their DoCalc counterparts with a
 zeroed-out vector/matrix. We confirm this with a custom subclass of
 FemElement that tests the input vector in DoCalc methods and returns a
 specific value. */
TEST_F(FemElementTest, Residual) {
  Vector<T, DummyElementTraits<0>::kNumDofs> residual;
  element_.CalcResidual(state_, &residual);
  EXPECT_EQ(residual, element_.expected_residual());
}

TEST_F(FemElementTest, StiffnessMatrix) {
  Eigen::Matrix<T, DummyElementTraits<0>::kNumDofs,
                DummyElementTraits<0>::kNumDofs>
      K;
  element_.CalcStiffnessMatrix(state_, &K);
  EXPECT_EQ(K, element_.expected_matrix());
}

TEST_F(FemElementTest, DampingMatrix) {
  Eigen::Matrix<T, DummyElementTraits<0>::kNumDofs,
                DummyElementTraits<0>::kNumDofs>
      D;
  element_.CalcDampingMatrix(state_, &D);
  EXPECT_EQ(D, element_.expected_matrix());
}

/* Test that CalcMassMatrix() is calling the expected DoCalcMassMatrix(). */
TEST_F(FemElementTest, MassMatrix) {
  Eigen::Matrix<T, DummyElementTraits<0>::kNumDofs,
                DummyElementTraits<0>::kNumDofs>
      M;
  element_.CalcMassMatrix(state_, &M);
  EXPECT_EQ(M, element_.expected_matrix());
}
}  // namespace
}  // namespace test
}  // namespace fem
}  // namespace multibody
}  // namespace drake
