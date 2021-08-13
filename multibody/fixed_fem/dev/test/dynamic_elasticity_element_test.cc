#include "drake/multibody/fixed_fem/dev/dynamic_elasticity_element.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"

namespace drake {
namespace multibody {
namespace fem {
class DynamicElasticityElementTest : public ::testing::Test {
 protected:
  using T = AutoDiffXd;
  static constexpr int kNaturalDimension = 3;
  static constexpr int kSpatialDimension = 3;
  static constexpr int kQuadratureOrder = 1;
  const T kDummyDensity{1.23};
  const ElementIndex kZeroIndex{0};
  using QuadratureType =
      internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
  static constexpr int kNumQuads = QuadratureType::num_quadrature_points;
  using IsoparametricElementType =
      internal::LinearSimplexElement<T, kNaturalDimension, kSpatialDimension,
                                     kNumQuads>;
  using ConstitutiveModelType = internal::LinearConstitutiveModel<T, kNumQuads>;
  using ElementType =
      DynamicElasticityElement<IsoparametricElementType, QuadratureType,
                               ConstitutiveModelType>;
  static constexpr int kNumNodes = ElementType::Traits::kNumNodes;
  static constexpr int kNumDofs = ElementType::Traits::kNumDofs;
  const std::array<NodeIndex, kNumNodes> dummy_node_indices = {
      {NodeIndex(0), NodeIndex(1), NodeIndex(2), NodeIndex(3)}};
  const Vector3<T> kGravity_W{0, 0, -9.8};
  const DampingModel<T> dummy_damping_model{0.001, 0.002};

  void SetUp() override {
    SetupElement();
    SetupState();
  }

  void SetupElement() {
    Eigen::Matrix<T, kSpatialDimension, kNumNodes> reference_positions =
        get_reference_positions();
    ConstitutiveModelType model(1, 0.25);
    elements_.emplace_back(kZeroIndex, dummy_node_indices, model,
                           reference_positions, kDummyDensity, kGravity_W,
                           dummy_damping_model);
  }

  void SetupState() {
    /* Set arbitrary node positions, velocities and accelerations. */
    Vector<double, kNumDofs> x;
    x << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67;
    Vector<double, kNumDofs> v;
    v << 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67, 0.18, 0.63, 0.54, 0.13;
    Vector<double, kNumDofs> a;
    a << 0.03, 0.86, 0.85, 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.25, 0.53, 0.67;
    /* Set the derivatives of positions, velocities and accelerations. */
    Vector<T, kNumDofs> x_autodiff;
    Vector<T, kNumDofs> v_autodiff;
    Vector<T, kNumDofs> a_autodiff;
    math::InitializeAutoDiff(x, 3 * kNumDofs, 0, &x_autodiff);
    math::InitializeAutoDiff(v, 3 * kNumDofs, kNumDofs, &v_autodiff);
    math::InitializeAutoDiff(a, 3 * kNumDofs, 2 * kNumDofs, &a_autodiff);
    state_ = std::make_unique<FemState<ElementType>>(x_autodiff, v_autodiff,
                                                     a_autodiff);
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

  const ElementType& element() const {
    DRAKE_ASSERT(elements_.size() == 1);
    return elements_[0];
  }

  const DampingModel<T>& damping_model(const ElementType& e) const {
    return e.damping_model_;
  }

  /* Get the negative elastic force evaluated at `state_`. */
  Vector<T, kNumDofs> negative_elastic_force() const {
    Vector<T, kNumDofs> f = Vector<T, kNumDofs>::Zero();
    element().AddNegativeElasticForce(*state_, &f);
    return f;
  }

  /* Get the negative damping force evaluated at `state_`. */
  Vector<T, kNumDofs> negative_damping_force() const {
    Vector<T, kNumDofs> f = Vector<T, kNumDofs>::Zero();
    element().AddNegativeDampingForce(*state_, &f);
    return f;
  }

  const Vector<T, kNumDofs>& gravity_force() const {
    return element().gravity_force();
  }

  std::vector<ElementType> elements_;
  std::unique_ptr<FemState<ElementType>> state_;
};

namespace {
TEST_F(DynamicElasticityElementTest, Constructor) {
  EXPECT_EQ(element().node_indices(), dummy_node_indices);
  EXPECT_EQ(element().element_index(), kZeroIndex);
  EXPECT_EQ(damping_model(element()).mass_coeff(),
            dummy_damping_model.mass_coeff());
  EXPECT_EQ(damping_model(element()).stiffness_coeff(),
            dummy_damping_model.stiffness_coeff());

  ElementType move_constructed_element(std::move(elements_[0]));
  EXPECT_EQ(move_constructed_element.node_indices(), dummy_node_indices);
  EXPECT_EQ(move_constructed_element.element_index(), kZeroIndex);
  EXPECT_EQ(damping_model(move_constructed_element).mass_coeff(),
            dummy_damping_model.mass_coeff());
  EXPECT_EQ(damping_model(move_constructed_element).stiffness_coeff(),
            dummy_damping_model.stiffness_coeff());
}

/* Tests that the calculation of the residual calls the calculation of the
 negative elastic and damping forces and include the momentum term and the
 external force term. */
TEST_F(DynamicElasticityElementTest, Residual) {
  Vector<T, kNumDofs> residual;
  element().CalcResidual(*state_, &residual);
  Eigen::Matrix<T, kNumDofs, kNumDofs> mass_matrix;
  element().CalcMassMatrix(*state_, &mass_matrix);
  Vector<T, kNumDofs> momentum_change = mass_matrix * state_->qddot();
  Vector<T, kNumDofs> negative_fe = negative_elastic_force();
  Vector<T, kNumDofs> negative_fv = negative_damping_force();
  Vector<T, kNumDofs> external_force = Vector<T, kNumDofs>::Zero();
  element().AddScaledExternalForce(*state_, 1.0, &external_force);
  EXPECT_TRUE(CompareMatrices(
      residual, momentum_change + negative_fe + negative_fv - external_force,
      0));
}

/* Tests that the stiffness matrix for DynamicElasticityElement is the
 derivative of the residual with respect to the generalized positions when the
 constitutive model is linear. */
TEST_F(DynamicElasticityElementTest, StiffnessMatrixIsPositionDerivative) {
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

/* Tests that the damping matrix is the velocity derivative of the residual. */
TEST_F(DynamicElasticityElementTest, DampingMatrixIsVelocityDerivative) {
  Vector<T, kNumDofs> residual;
  element().CalcResidual(*state_, &residual);
  Eigen::Matrix<T, kNumDofs, kNumDofs> damping_matrix;
  element().CalcDampingMatrix(*state_, &damping_matrix);
  for (int i = 0; i < kNumDofs; ++i) {
    EXPECT_TRUE(CompareMatrices(
        residual(i).derivatives().transpose().segment<kNumDofs>(kNumDofs),
        damping_matrix.row(i), std::numeric_limits<double>::epsilon()));
  }
}

/* Tests that the mass matrix is the acceleration derivative of the residual. */
TEST_F(DynamicElasticityElementTest, MassMatrixIsAccelerationDerivative) {
  Vector<T, kNumDofs> residual;
  element().CalcResidual(*state_, &residual);
  Eigen::Matrix<T, kNumDofs, kNumDofs> mass_matrix;
  mass_matrix.setZero();
  element().CalcMassMatrix(*state_, &mass_matrix);
  for (int i = 0; i < kNumDofs; ++i) {
    EXPECT_TRUE(CompareMatrices(
        residual(i).derivatives().transpose().tail(kNumDofs),
        mass_matrix.row(i), std::numeric_limits<double>::epsilon()));
  }
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
