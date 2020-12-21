#include "drake/multibody/fixed_fem/dev/static_elasticity_element.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/linear_simplex_element.h"
#include "drake/multibody/fixed_fem/dev/simplex_gaussian_quadrature.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace {
constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kQuadratureOrder = 1;
const ElementIndex kDummyElementIndex(0);

class StaticElasticityElementTest : public ::testing::Test {
 protected:
  using QuadratureType =
      SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
  static constexpr int kNumQuads = QuadratureType::num_quadrature_points();
  using IsoparametricElementType =
      LinearSimplexElement<AutoDiffXd, kNaturalDimension, kSpatialDimension,
                           kNumQuads>;
  using ConstitutiveModelType = LinearConstitutiveModel<AutoDiffXd, kNumQuads>;
  using ElementType =
      StaticElasticityElement<IsoparametricElementType, QuadratureType,
                              ConstitutiveModelType>;
  static constexpr int kDofs = ElementType::Traits::kNumDofs;
  static constexpr int kNumNodes = ElementType::Traits::kNumNodes;
  const std::array<NodeIndex, kNumNodes> dummy_node_indices = {
      {NodeIndex(0), NodeIndex(1), NodeIndex(2), NodeIndex(3)}};

  void SetUp() override {
    SetupElement();
    SetupState();
  }

  void SetupElement() {
    Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes>
        reference_positions = get_reference_positions();
    ConstitutiveModelType model(1, 0.25);
    elements_.emplace_back(kDummyElementIndex, dummy_node_indices, model,
                           reference_positions);
  }

  void SetupState() {
    // Set arbitrary node positions and the gradient.
    Vector<double, kDofs> x;
    x << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67;
    const Eigen::Matrix<double, kDofs, Eigen::Dynamic> gradient =
        MatrixX<double>::Identity(kDofs, kDofs);
    Vector<AutoDiffXd, kDofs> x_autodiff;
    math::initializeAutoDiff(x, x_autodiff);
    state_ = std::make_unique<FemState<ElementType>>(x_autodiff);
    state_->MakeElementData(elements_);
  }

  /* Set an arbitrary reference positions such that the tetrahedron is not
   inverted. */
  Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes>
  get_reference_positions() const {
    Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes> X(kSpatialDimension,
                                                              kNumNodes);
    // clang-format off
    X << -0.10, 0.90, 0.02, 0.10,
         1.33, 0.23, 0.04, 0.01,
         0.20, 0.03, 2.31, -0.12;
    // clang-format on
    return X;
  }

  /* Get the one and only element. */
  const ElementType& element() const {
    DRAKE_ASSERT(elements_.size() == 1);
    return elements_[0];
  }

  std::vector<ElementType> elements_;
  std::unique_ptr<FemState<ElementType>> state_;
};

TEST_F(StaticElasticityElementTest, Constructor) {
  EXPECT_EQ(element().node_indices(), dummy_node_indices);
  EXPECT_EQ(element().element_index(), kDummyElementIndex);
}

TEST_F(StaticElasticityElementTest, ResidualIsNegativeEnergyDerivative) {
  AutoDiffXd energy = element().CalcElasticEnergy(*state_);
  Vector<AutoDiffXd, kDofs> residual;
  element().CalcResidual(*state_, &residual);

  EXPECT_TRUE(CompareMatrices(energy.derivatives(), residual,
                              std::numeric_limits<double>::epsilon()));
}

TEST_F(StaticElasticityElementTest, StiffnessMatrixIsResidualDerivative) {
  Vector<AutoDiffXd, kDofs> residual;
  element().CalcResidual(*state_, &residual);
  Eigen::Matrix<AutoDiffXd, kDofs, kDofs> stiffness_matrix;
  element().CalcStiffnessMatrix(*state_, &stiffness_matrix);

  for (int i = 0; i < kDofs; ++i) {
    EXPECT_TRUE(CompareMatrices(residual(i).derivatives().transpose(),
                                stiffness_matrix.row(i),
                                std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(StaticElasticityElementTest, NoDampingMatrix) {
  Eigen::Matrix<AutoDiffXd, kDofs, kDofs> damping_matrix;
  DRAKE_EXPECT_THROWS_MESSAGE(
      element().CalcDampingMatrix(*state_, &damping_matrix), std::exception,
      "Static elasticity forms a zero-th order ODE and does not provide a "
      "damping matrix.");
}

TEST_F(StaticElasticityElementTest, NoMassMatrix) {
  Eigen::Matrix<AutoDiffXd, kDofs, kDofs> mass_matrix;
  DRAKE_EXPECT_THROWS_MESSAGE(
      element().CalcMassMatrix(*state_, &mass_matrix), std::exception,
      "Static elasticity forms a zero-th order ODE and does not provide a mass "
      "matrix.");
}
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
