#include "drake/multibody/fixed_fem/dev/static_elasticity_element.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"

namespace drake {
namespace multibody {
namespace fem {
class StaticElasticityElementTest : public ::testing::Test {
 protected:
  static constexpr int kNaturalDimension = 3;
  static constexpr int kSpatialDimension = 3;
  static constexpr int kQuadratureOrder = 1;
  const ElementIndex kZeroIndex{0};
  using T = AutoDiffXd;
  using QuadratureType =
      internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
  static constexpr int kNumQuads = QuadratureType::num_quadrature_points;
  using IsoparametricElementType =
      internal::LinearSimplexElement<T, kNaturalDimension, kSpatialDimension,
                                     kNumQuads>;
  using ConstitutiveModelType = internal::LinearConstitutiveModel<T, kNumQuads>;
  using ElementType =
      StaticElasticityElement<IsoparametricElementType, QuadratureType,
                              ConstitutiveModelType>;
  static constexpr int kNumDofs = ElementType::Traits::kNumDofs;
  static constexpr int kNumNodes = ElementType::Traits::kNumNodes;
  const std::array<NodeIndex, kNumNodes> dummy_node_indices = {
      {NodeIndex(0), NodeIndex(1), NodeIndex(2), NodeIndex(3)}};
  const T kDensity{0.123};
  const Vector3<T> kGravity_W{0, 0, -9.8};

  void SetUp() override {
    SetupElement();
    SetupState();
  }

  void SetupElement() {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> reference_positions =
        get_reference_positions();
    ConstitutiveModelType model(1, 0.25);
    elements_.emplace_back(kZeroIndex, dummy_node_indices, model,
                           reference_positions, kDensity, kGravity_W);
  }

  /* Set up a state with arbitrary positions. */
  void SetupState() {
    /* Set arbitrary node positions and the gradient. */
    Vector<double, kNumDofs> x;
    x << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67;
    Vector<T, kNumDofs> x_autodiff;
    math::InitializeAutoDiff(x, &x_autodiff);
    state_ = std::make_unique<FemState<ElementType>>(x_autodiff);
    state_->MakeElementData(elements_);
  }

  /* Set arbitrary reference positions such that the tetrahedron is not
   inverted. */
  Eigen::Matrix<T, kSpatialDimension, kNumNodes> get_reference_positions()
      const {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> X(kSpatialDimension,
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

  /* Calculates the negative elastic force evaluted at `state_`. */
  Vector<T, kNumDofs> CalcNegativeElasticForce() const {
    Vector<T, kNumDofs> neg_force = Vector<T, kNumDofs>::Zero();
    element().AddNegativeElasticForce(*state_, &neg_force);
    return neg_force;
  }

  /* Calculates the negative elastic force derivatives with respect to positions
   evaluted at `state_`. */
  Eigen::Matrix<T, kNumDofs, kNumDofs> CalcNegativeElasticForceDerivative()
      const {
    Eigen::Matrix<T, kNumDofs, kNumDofs> neg_force_derivative =
        Eigen::Matrix<T, kNumDofs, kNumDofs>::Zero();
    element().AddNegativeElasticForceDerivative(*state_, &neg_force_derivative);
    return neg_force_derivative;
  }

  std::vector<ElementType> elements_;
  std::unique_ptr<FemState<ElementType>> state_;
};

namespace {
TEST_F(StaticElasticityElementTest, Constructor) {
  EXPECT_EQ(element().node_indices(), dummy_node_indices);
  EXPECT_EQ(element().element_index(), kZeroIndex);
  ElementType move_constructed_element(std::move(elements_[0]));
  EXPECT_EQ(move_constructed_element.node_indices(), dummy_node_indices);
  EXPECT_EQ(move_constructed_element.element_index(), kZeroIndex);
}

/* Tests that the calculation of the residual calls the calculation of the
 negative elastic force and the external force. */
TEST_F(StaticElasticityElementTest,
       ResidualIsNegativeElasticForceMinusExternalForce) {
  Vector<T, kNumDofs> residual;
  element().CalcResidual(*state_, &residual);
  Vector<T, kNumDofs> external_force = Vector<T, kNumDofs>::Zero();
  element().AddScaledExternalForce(*state_, 1.0, &external_force);
  EXPECT_TRUE(CompareMatrices(CalcNegativeElasticForce() - external_force,
                              residual, 0));
}

/* Tests that the calculation of the stiffness matrix calls the calculation of
 the negative elastic force derivatives. */
TEST_F(StaticElasticityElementTest, StiffnessMatrixIsNegativeElasticForce) {
  Eigen::Matrix<T, kNumDofs, kNumDofs> stiffness_matrix;
  element().CalcStiffnessMatrix(*state_, &stiffness_matrix);
  EXPECT_TRUE(CompareMatrices(CalcNegativeElasticForceDerivative(),
                              stiffness_matrix, 0));
}

/* Tests that the stiffness matrix is the derivative of the residual with
 respect to the generalized positions. */
TEST_F(StaticElasticityElementTest, StiffnessMatrixIsPositionDerivative) {
  Vector<T, kNumDofs> residual;
  element().CalcResidual(*state_, &residual);
  Eigen::Matrix<T, kNumDofs, kNumDofs> stiffness_matrix;
  element().CalcStiffnessMatrix(*state_, &stiffness_matrix);
  for (int i = 0; i < kNumDofs; ++i) {
    EXPECT_TRUE(CompareMatrices(
        residual(i).derivatives().transpose().head(kNumDofs),
        stiffness_matrix.row(i), std::numeric_limits<double>::epsilon()));
  }
}

TEST_F(StaticElasticityElementTest, NoDampingMatrix) {
  Eigen::Matrix<T, kNumDofs, kNumDofs> damping_matrix;
  DRAKE_EXPECT_THROWS_MESSAGE(
      element().CalcDampingMatrix(*state_, &damping_matrix),
      "Static elasticity forms a zero-th order ODE and does not provide a "
      "damping matrix.");
}

TEST_F(StaticElasticityElementTest, NoMassMatrix) {
  Eigen::Matrix<T, kNumDofs, kNumDofs> mass_matrix;
  DRAKE_EXPECT_THROWS_MESSAGE(
      element().CalcMassMatrix(*state_, &mass_matrix),
      "Static elasticity forms a zero-th order ODE and does not provide a mass "
      "matrix.");
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
