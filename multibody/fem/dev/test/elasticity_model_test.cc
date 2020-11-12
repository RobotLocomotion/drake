#include "drake/multibody/fem/dev/elasticity_model.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/dev/fem_state.h"
#include "drake/multibody/fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fem/dev/linear_simplex_element.h"
#include "drake/multibody/fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {
constexpr int kNaturalDim = 3;
constexpr int kSpatialDim = 3;
constexpr int kQuadratureOrder = 1;
constexpr int kNumQuads = 1;
constexpr int kNumVertices = 8;
constexpr int kDof = kSpatialDim * kNumVertices;
constexpr double kMassDensity(10);
constexpr double kYoungsModulus(2);
constexpr double kPoissonRatio(0.25);

class ElasticityModelTest : public ::testing::Test {
 protected:
  using Scalar = AutoDiffXd;
  using IsoparametricElementType = LinearSimplexElement<Scalar, kNaturalDim>;
  using QuadratureType =
      SimplexGaussianQuadrature<Scalar, kQuadratureOrder, kNaturalDim>;

  void SetUp() override {
    /* Make a unit cube and subdivide it into tetrahedra. */
    const double length = 1;
    geometry::Box box(length, length, length);
    mesh_ = std::make_unique<geometry::VolumeMesh<Scalar>>(
        geometry::internal::MakeBoxVolumeMesh<Scalar>(box, 1));
    LinearConstitutiveModel<Scalar> linear_elasticity_model(kYoungsModulus,
                                                            kPoissonRatio);
    Scalar density(kMassDensity);
    elasticity_model_.AddElasticityElementsFromTetMesh(
        *mesh_, density, linear_elasticity_model, kQuadratureOrder);
  }

  /* Creates arbitrary node positions along with derivatives 1. The position
   could potentially cause elements to invert, but that should not affect the
   result of the tests. */
  VectorX<Scalar> MakePositions() {
    VectorX<double> x(kDof);
    x << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67,
        0.81, 0.36, 0.45, 0.31, 0.29, 0.71, 0.30, 0.68, 0.58, 0.52, 0.35, 0.76;
    VectorX<AutoDiffXd> x_autodiff(kDof);
    math::initializeAutoDiff(x, x_autodiff);
    return x_autodiff;
  }

  /* The ElasticityModel under test. */
  ElasticityModel<Scalar> elasticity_model_;
  /* The geometry of the model. */
  std::unique_ptr<geometry::VolumeMesh<Scalar>> mesh_;
};

TEST_F(ElasticityModelTest, Basic) {
  EXPECT_EQ(elasticity_model_.num_nodes(), kNumVertices);
  /* Each cube is split into 6 tetrahedra. */
  EXPECT_EQ(elasticity_model_.num_elements(), 6);
  EXPECT_EQ(elasticity_model_.solution_dimension(), 3);
}

TEST_F(ElasticityModelTest, MakeState) {
  std::unique_ptr<FemState<Scalar>> state = elasticity_model_.MakeFemState();
  ASSERT_TRUE(state != nullptr);
  EXPECT_EQ(state->element_cache_size(), elasticity_model_.num_elements());
  for (ElementIndex i(0); i < state->element_cache_size(); ++i) {
    /* The element cache should be of type ElasticityElementCacheEntry. */
    const auto* element_cache_entry =
        dynamic_cast<const ElasticityElementCacheEntry<Scalar>*>(
            &state->element_cache_entry(i));
    ASSERT_TRUE(element_cache_entry != nullptr);
    /* The DeformationGradientCacheEntry should be of type
     LinearConstitutiveModelCacheEntry. */
    const auto* linear_elasticity_model_cache_entry =
        dynamic_cast<const LinearConstitutiveModelCacheEntry<Scalar>*>(
            &element_cache_entry->deformation_gradient_cache_entry());
    /* Verify that the cache is properly set up. */
    ASSERT_TRUE(linear_elasticity_model_cache_entry != nullptr);
    EXPECT_EQ(linear_elasticity_model_cache_entry->element_index(), i);
    EXPECT_EQ(linear_elasticity_model_cache_entry->num_quadrature_points(),
              kNumQuads);
    EXPECT_EQ(linear_elasticity_model_cache_entry->strain().size(), kNumQuads);
    EXPECT_EQ(linear_elasticity_model_cache_entry->trace_strain().size(),
              kNumQuads);
    EXPECT_EQ(
        linear_elasticity_model_cache_entry->deformation_gradient().size(),
        kNumQuads);
  }
  EXPECT_EQ(state->num_generalized_positions(), kDof);
  /* The default qdot should be zero. */
  EXPECT_EQ(state->qdot(), VectorX<Scalar>::Zero(kDof));
  /* The default q should be the reference position of the mesh. */
  for (int i = 0; i < kNumVertices; ++i) {
    EXPECT_EQ(state->q().segment<3>(3 * i),
              mesh_->vertex(geometry::VolumeVertexIndex(i)).r_MV());
  }
  EXPECT_EQ(state->qdot(), VectorX<Scalar>::Zero(kDof));
}

TEST_F(ElasticityModelTest, ResidualIsEnergyDerivative) {
  std::unique_ptr<FemState<Scalar>> state = elasticity_model_.MakeFemState();
  /* Move to arbitrary positions. */
  state->set_q(MakePositions());
  Scalar energy = elasticity_model_.CalcElasticEnergy(*state);
  VectorX<Scalar> residual(state->num_generalized_positions());
  elasticity_model_.CalcResidual(*state, &residual);
  EXPECT_TRUE(CompareMatrices(energy.derivatives(), residual,
                              std::numeric_limits<double>::epsilon()));
}

TEST_F(ElasticityModelTest, TangentMatrixIsResidualDerivative) {
  std::unique_ptr<FemState<Scalar>> state = elasticity_model_.MakeFemState();
  /* Move to arbitrary positions. */
  state->set_q(MakePositions());
  VectorX<Scalar> residual(state->num_generalized_positions());
  elasticity_model_.CalcResidual(*state, &residual);
  Eigen::SparseMatrix<Scalar> tangent_matrix;
  elasticity_model_.SetTangentMatrixSparsityPattern(&tangent_matrix);
  elasticity_model_.CalcTangentMatrix(*state, &tangent_matrix);
  /* In the discretization of the unit cube by 6 tetrahedra, there are 19 edges,
   and 8 nodes, creating 19*2 + 8 blocks of 3-by-3 nonzero entries. Hence the
   number of nonzero entries of the tangent matrix should be (19*2+8)*9. */
  const int nnz = (19 * 2 + 8) * 9;
  EXPECT_EQ(tangent_matrix.nonZeros(), nnz);
  MatrixX<Scalar> dense_tangent_matrix(tangent_matrix);
  for (int i = 0; i < state->num_generalized_positions(); ++i) {
    EXPECT_TRUE(CompareMatrices(residual(i).derivatives(),
                                dense_tangent_matrix.col(i),
                                std::numeric_limits<double>::epsilon()));
  }
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
