#include "drake/multibody/fem/dev/fem_elasticity.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/dev/fem_state.h"
#include "drake/multibody/fem/dev/linear_elasticity_model.h"
#include "drake/multibody/fem/dev/linear_simplex_element.h"
#include "drake/multibody/fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
constexpr int kNaturalDim = 3;
constexpr int kSpatialDim = 3;
constexpr int kQuadratureOrder = 1;
constexpr int kNumVertices = 4;
constexpr int kDof = kSpatialDim * kNumVertices;
const ElementIndex kDummyElementIndex(0);

class FemElasticityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    SetupElement();
    SetupState();
  }

  void SetupElement() {
    quadrature_ = std::make_unique<
        SimplexGaussianQuadrature<AutoDiffXd, kQuadratureOrder, kSpatialDim>>();
    shape_ = std::make_unique<LinearSimplexElement<AutoDiffXd, kNaturalDim>>(
        quadrature_->get_points());
    std::vector<NodeIndex> node_indices = {NodeIndex(0), NodeIndex(1),
                                           NodeIndex(2), NodeIndex(3)};
    Matrix3X<AutoDiffXd> reference_positions = get_reference_positions();
    linear_elasticity_ =
        std::make_unique<LinearElasticityModel<AutoDiffXd>>(100, 0.25);
    fem_elasticity_ = std::make_unique<FemElasticity<AutoDiffXd, kNaturalDim>>(
        kDummyElementIndex, *shape_, *quadrature_, node_indices,
        reference_positions, *linear_elasticity_);
  }

  void SetupState() {
    state_.Resize(kDof);
    state_.mutable_v() = VectorX<AutoDiffXd>::Zero(kDof);
    // Set arbitrary node positions and the gradient.
    Eigen::Matrix<double, kDof, 1> x;
    x << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67;
    const Eigen::Matrix<double, kDof, Eigen::Dynamic> gradient =
        MatrixX<double>::Identity(kDof, kDof);
    const VectorX<AutoDiffXd> x_autodiff =
        math::initializeAutoDiffGivenGradientMatrix(x, gradient);
    state_.mutable_x() = x_autodiff;
    // Set up the element cache.
    auto& cache = state_.mutable_cache();
    auto linear_elasticity_cache =
        std::make_unique<LinearElasticityModelCache<AutoDiffXd>>(
            kDummyElementIndex, quadrature_->num_points());
    cache.emplace_back(std::make_unique<ElasticityElementCache<AutoDiffXd>>(
        kDummyElementIndex, quadrature_->num_points(),
        std::move(linear_elasticity_cache)));
  }

  // Set an arbitrary reference position such that the tetrahedron is not
  // inverted.
  Matrix3X<AutoDiffXd> get_reference_positions() const {
    Matrix3X<AutoDiffXd> Q(kSpatialDim, kNumVertices);
    Q << -0.10, 0.90, 0.02, 0.10, 1.33, 0.23, 0.04, 0.01, 0.20, 0.03, 2.31,
        -0.12;
    return Q;
  }

  // Calculates the elastic force at state_.
  VectorX<AutoDiffXd> CalcElasticForce() const {
      VectorX<AutoDiffXd> force(kDof);
      fem_elasticity_->CalcElasticForce(state_, &force);
      return force;
  }

  std::unique_ptr<FemElasticity<AutoDiffXd, kNaturalDim>> fem_elasticity_;
  std::unique_ptr<IsoparametricElement<AutoDiffXd, kNaturalDim>> shape_;
  std::unique_ptr<Quadrature<AutoDiffXd, kNaturalDim>> quadrature_;
  std::unique_ptr<LinearElasticityModel<AutoDiffXd>> linear_elasticity_;
  FemState<AutoDiffXd> state_;
};

namespace {
TEST_F(FemElasticityTest, Basic) {
  EXPECT_EQ(fem_elasticity_->num_nodes(), kNumVertices);
  EXPECT_EQ(fem_elasticity_->num_quads(), quadrature_->num_points());
  EXPECT_EQ(fem_elasticity_->num_spatial_dim(), kSpatialDim);
}

TEST_F(FemElasticityTest, ElasticForceIsNegativeEnergyDerivative) {
  AutoDiffXd energy = fem_elasticity_->CalcElasticEnergy(state_);
  VectorX<AutoDiffXd> residual = CalcElasticForce();
  EXPECT_TRUE(CompareMatrices(energy.derivatives(), -residual, 1e-13));
}
}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
