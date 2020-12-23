#include "drake/multibody/fixed_fem/dev/dynamic_elasticity_element.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
constexpr int kNumNodes = 4;
constexpr int kDofs = kSpatialDimension * kNumNodes;
const ElementIndex kDummyElementIndex(0);
const std::array<NodeIndex, kNumNodes> dummy_node_indices = {
    {NodeIndex(0), NodeIndex(1), NodeIndex(2), NodeIndex(3)}};

class DynamicElasticityElementTest : public ::testing::Test {
 protected:
  using QuadratureType =
      SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
  static constexpr int kNumQuads = QuadratureType::num_quadrature_points();
  using IsoparametricElementType =
      LinearSimplexElement<AutoDiffXd, kNaturalDimension, kSpatialDimension,
                           kNumQuads>;
  using ConstitutiveModelType = LinearConstitutiveModel<AutoDiffXd, kNumQuads>;
  using ElementType =
      DynamicElasticityElement<IsoparametricElementType, QuadratureType,
                               ConstitutiveModelType>;

  void SetUp() override {
    SetupElement();
    SetupState();
  }

  void SetupElement() {
    Eigen::Matrix<AutoDiffXd, kSpatialDimension, kNumNodes>
        reference_positions = get_reference_positions();
    ConstitutiveModelType model(1, 0.25);
    AutoDiffXd dummy_density(1.23);
    DampingModel damping_model;
    damping_model.mass_damping_ = 0.01;
    damping_model.stiffness_damping_ = 0.02;

    dynamic_elasticity_element_ = std::make_unique<ElementType>(
        kDummyElementIndex, dummy_node_indices, model, reference_positions,
        dummy_density, damping_model);
  }

  void SetupState() {
    // Set arbitrary node positions and the gradient.
    Vector<double, kDofs> x;
    x << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67;
    const Eigen::Matrix<double, kDofs, Eigen::Dynamic> gradient =
        MatrixX<double>::Identity(kDofs, kDofs);
    Vector<AutoDiffXd, kDofs> x_autodiff;
    Vector<AutoDiffXd, kDofs> v = Vector<AutoDiffXd, kDofs>::Zero();
    Vector<AutoDiffXd, kDofs> a = Vector<AutoDiffXd, kDofs>::Zero();
    math::initializeAutoDiff(x, x_autodiff);
    state_ = std::make_unique<FemState<ElementType>>(x_autodiff, v, a);
    // Set up the element cache.
    std::vector<typename ElementType::ElementCacheEntryType> cache(
        1, dynamic_elasticity_element_->MakeElementCacheEntry());
    state_->ResetElementCache(cache);
  }

  // Set an arbitrary reference positions such that the tetrahedron is not
  // inverted.
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

  std::unique_ptr<ElementType> dynamic_elasticity_element_;
  std::unique_ptr<FemState<ElementType>> state_;
};

TEST_F(DynamicElasticityElementTest, Basic) {
  EXPECT_EQ(dynamic_elasticity_element_->natural_dimension(),
            kNaturalDimension);
  EXPECT_EQ(dynamic_elasticity_element_->solution_dimension(),
            kSpatialDimension);
  EXPECT_EQ(dynamic_elasticity_element_->spatial_dimension(),
            kSpatialDimension);
  EXPECT_EQ(dynamic_elasticity_element_->num_quadrature_points(), kNumQuads);
  EXPECT_EQ(dynamic_elasticity_element_->num_nodes(), kNumNodes);
  EXPECT_EQ(dynamic_elasticity_element_->num_dofs(), kDofs);
  EXPECT_EQ(dynamic_elasticity_element_->ode_order(), 2);
  EXPECT_EQ(dynamic_elasticity_element_->node_indices(), dummy_node_indices);
  EXPECT_EQ(dynamic_elasticity_element_->element_index(), kDummyElementIndex);
}

TEST_F(DynamicElasticityElementTest, Smoke) {
  Vector<AutoDiffXd, kDofs> residual;
  dynamic_elasticity_element_->CalcResidual(*state_, &residual);
  Eigen::Matrix<AutoDiffXd, kDofs, kDofs> stiffness_matrix;
  dynamic_elasticity_element_->CalcStiffnessMatrix(*state_, &stiffness_matrix);
  Eigen::Matrix<AutoDiffXd, kDofs, kDofs> damping_matrix;
  dynamic_elasticity_element_->CalcDampingMatrix(*state_, &damping_matrix);
  Eigen::Matrix<AutoDiffXd, kDofs, kDofs> mass_matrix;
  dynamic_elasticity_element_->CalcMassMatrix(*state_, &mass_matrix);
}
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
