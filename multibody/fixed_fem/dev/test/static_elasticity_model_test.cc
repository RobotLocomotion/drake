#include "drake/multibody/fixed_fem/dev/static_elasticity_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/static_elasticity_element.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr int kNaturalDimension = 3;
constexpr int kSpatialDimension = 3;
constexpr int kSolutionDimension = 3;
constexpr int kQuadratureOrder = 1;

using QuadratureType =
    internal::SimplexGaussianQuadrature<kNaturalDimension, kQuadratureOrder>;
constexpr int kNumQuads = QuadratureType::num_quadrature_points;

using AutoDiffIsoparametricElement =
    internal::LinearSimplexElement<AutoDiffXd, kNaturalDimension,
                                   kSpatialDimension, kNumQuads>;
using DoubleIsoparametricElement =
    internal::LinearSimplexElement<double, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;

using AutoDiffConstitutiveModel =
    internal::LinearConstitutiveModel<AutoDiffXd, kNumQuads>;
using DoubleConstitutiveModel =
    internal::LinearConstitutiveModel<double, kNumQuads>;

using AutoDiffElement =
    StaticElasticityElement<AutoDiffIsoparametricElement, QuadratureType,
                            AutoDiffConstitutiveModel>;
using DoubleElement =
    StaticElasticityElement<DoubleIsoparametricElement, QuadratureType,
                            DoubleConstitutiveModel>;

const double kYoungsModulus = 1.23;
const double kPoissonRatio = 0.456;
/* The geometry of the model under test is a cube and it has 8 vertices and 6
 elements. */
constexpr int kNumCubeVertices = 8;
constexpr int kNumDofs = kNumCubeVertices * kSolutionDimension;
constexpr int kNumElements = 6;
const double kDensity{0.123};

class StaticElasticityModelTest : public ::testing::Test {
 protected:
  /* Make a box and subdivide it into 6 tetrahedra. */
  template <typename T>
  geometry::VolumeMesh<T> MakeBoxTetMesh() {
    const double length = 0.1;
    geometry::Box box(length, length, length);
    geometry::VolumeMesh<T> mesh =
        geometry::internal::MakeBoxVolumeMesh<T>(box, length);
    return mesh;
  }

  void SetUp() override { AddBoxToModel(&model_); }

  template <typename FemModel>
  void AddBoxToModel(FemModel* fem_model) {
    using T = typename FemModel::T;
    geometry::VolumeMesh<T> mesh = MakeBoxTetMesh<T>();
    const typename FemModel::ConstitutiveModel constitutive_model(
        kYoungsModulus, kPoissonRatio);
    fem_model->AddStaticElasticityElementsFromTetMesh(mesh, constitutive_model,
                                                      kDensity);
  }

  /* Returns an arbitrary perturbation to the reference configuration of the
   cube geometry. */
  static Vector<double, kNumDofs> perturbation() {
    Vector<double, kNumDofs> dx;
    dx << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53,
        0.67, 0.81, 0.36, 0.45, 0.31, 0.29, 0.71, 0.30, 0.68, 0.58, 0.52, 0.35,
        0.76;
    return dx;
  }

  /* Returns a perturbed FemState whose generalized positions are different
   from reference positions. */
  template <typename FemModelType>
  typename FemModelType::State MakeDeformedState(
      const FemModelType& fem_model) const {
    using State = typename FemModelType::State;
    using T = typename FemModelType::T;

    State state = fem_model.MakeFemState();
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      const Vector<double, kNumDofs> perturbed_q =
          math::ExtractValue(state.q()) + perturbation();
      Vector<AutoDiffXd, kNumDofs> perturbed_q_autodiff;
      math::InitializeAutoDiff(perturbed_q, &perturbed_q_autodiff);
      state.SetQ(perturbed_q_autodiff);
    } else {
      const Vector<double, kNumDofs> perturbed_q = state.q() + perturbation();
      state.SetQ(perturbed_q);
    }
    return state;
  }

  /* The model under test. */
  StaticElasticityModel<AutoDiffElement> model_;
};

/* Tests the mesh has been successfully converted to elements. */
TEST_F(StaticElasticityModelTest, Geometry) {
  EXPECT_EQ(model_.num_nodes(), kNumCubeVertices);
  EXPECT_EQ(model_.num_elements(), kNumElements);
}

/* Tests that the residual of the model is the derivative of the elastic
 energy with respect to the generalized positions plus external force. */
TEST_F(StaticElasticityModelTest,
       ResidualIsEnergyDerivativeMinusExternalForce) {
  using T = AutoDiffXd;
  FemState<AutoDiffElement> state = MakeDeformedState(model_);
  T energy = model_.CalcElasticEnergy(state);
  VectorX<T> residual(state.num_generalized_positions());
  model_.CalcResidual(state, &residual);
  VectorX<T> external_force(state.num_generalized_positions());
  model_.CalcExternalForce(state, &external_force);
  EXPECT_TRUE(CompareMatrices(energy.derivatives() - external_force, residual,
                              std::numeric_limits<double>::epsilon()));
}

/* Tests that the tangent matrix of the model is the derivative of the
 residual with respect to generalized positions. */
TEST_F(StaticElasticityModelTest, TangentMatrixIsResidualDerivative) {
  using T = AutoDiffXd;
  FemState<AutoDiffElement> state = MakeDeformedState(model_);

  VectorX<T> residual(state.num_generalized_positions());
  model_.CalcResidual(state, &residual);

  Eigen::SparseMatrix<T> tangent_matrix;
  model_.SetTangentMatrixSparsityPattern(&tangent_matrix);
  model_.CalcTangentMatrix(state, &tangent_matrix);

  /* In the discretization of the unit cube by 6 tetrahedra, there are 19
   edges, and 8 nodes, creating 19*2 + 8 blocks of 3-by-3 nonzero entries.
   Hence the number of nonzero entries of the tangent matrix should be
   (19*2+8)*9. */
  const int nnz = (19 * 2 + 8) * 9;
  EXPECT_EQ(tangent_matrix.nonZeros(), nnz);

  MatrixX<T> dense_tangent_matrix(tangent_matrix);
  for (int i = 0; i < state.num_generalized_positions(); ++i) {
    EXPECT_TRUE(CompareMatrices(residual(i).derivatives(),
                                dense_tangent_matrix.col(i),
                                std::numeric_limits<double>::epsilon()));
  }
}

/* Verifies that the tangent matrix calculated as PETSc matrix is the same as
 that calculated as Eigen::SparseMatrix. */
TEST_F(StaticElasticityModelTest, TangentMatrixParity) {
  FemState<AutoDiffElement> autodiff_state = MakeDeformedState(model_);
  Eigen::SparseMatrix<AutoDiffXd> eigen_tangent_matrix;
  model_.SetTangentMatrixSparsityPattern(&eigen_tangent_matrix);
  model_.CalcTangentMatrix(autodiff_state, &eigen_tangent_matrix);
  const MatrixX<AutoDiffXd> eigen_dense_autodiff_matrix = eigen_tangent_matrix;
  const MatrixXd eigen_dense_matrix =
      math::ExtractValue(eigen_dense_autodiff_matrix);

  StaticElasticityModel<DoubleElement> double_model;
  AddBoxToModel(&double_model);
  FemState<DoubleElement> double_state = MakeDeformedState(double_model);
  std::unique_ptr<internal::PetscSymmetricBlockSparseMatrix>
      petsc_tangent_matrix =
          double_model.MakePetscSymmetricBlockSparseTangentMatrix();
  double_model.CalcTangentMatrix(double_state, petsc_tangent_matrix.get());
  petsc_tangent_matrix->AssembleIfNecessary();
  const MatrixXd petsc_dense_matrix = petsc_tangent_matrix->MakeDenseMatrix();

  EXPECT_TRUE(CompareMatrices(eigen_dense_matrix, petsc_dense_matrix,
                              std::numeric_limits<double>::epsilon()));
}

/* Tests that the total external force exerted on a model matches the expected
 value. */
TEST_F(StaticElasticityModelTest, ExternalForce) {
  using T = AutoDiffXd;
  FemState<AutoDiffElement> state = MakeDeformedState(model_);
  const Vector3<T> gravity{1, 0, 0};
  model_.SetGravity(gravity);
  VectorX<T> external_force(state.num_generalized_positions());
  model_.CalcExternalForce(state, &external_force);
  Vector3<T> total_external_force = Vector3<T>::Zero();
  for (int i = 0; i < model_.num_nodes(); ++i) {
    total_external_force += external_force.template segment<3>(i * 3);
  }
  T volume = MakeBoxTetMesh<T>().CalcVolume();
  // TODO(xuchenhan-tri) We are assuming that the only external force is gravity
  //  now. This might change in the future. Change this test accordingly.
  Vector3<T> expected_external_force = gravity * kDensity * volume;
  EXPECT_TRUE(CompareMatrices(total_external_force, expected_external_force,
                              std::numeric_limits<double>::epsilon()));
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
