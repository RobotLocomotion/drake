#include "drake/multibody/fixed_fem/dev/dynamic_elasticity_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fixed_fem/dev/dynamic_elasticity_element.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/linear_constitutive_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace {

using Eigen::MatrixXd;

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
using AutoDiffConstitutiveModel =
    internal::LinearConstitutiveModel<AutoDiffXd, kNumQuads>;
using AutoDiffElement =
    DynamicElasticityElement<AutoDiffIsoparametricElement, QuadratureType,
                             AutoDiffConstitutiveModel>;

using DoubleIsoparametricElement =
    internal::LinearSimplexElement<double, kNaturalDimension, kSpatialDimension,
                                   kNumQuads>;
using DoubleConstitutiveModel =
    internal::LinearConstitutiveModel<double, kNumQuads>;
using DoubleElement =
    DynamicElasticityElement<DoubleIsoparametricElement, QuadratureType,
                             DoubleConstitutiveModel>;

const double kYoungsModulus = 1.23;
const double kPoissonRatio = 0.456;
const double kDensity = 0.789;
/* The geometry of the model under test is a cube and it has 8 vertices and 6
 elements. */
constexpr int kNumCubeVertices = 8;
constexpr int kNumDofs = kNumCubeVertices * kSolutionDimension;
constexpr int kNumElements = 6;
/* Parameters for Newmark scheme. */
const double kDt = 1e-3;
/* Parameters for the damping model. */
const double kMassDamping = 0.01;
const double kStiffnessDamping = 0.02;

class DynamicElasticityModelTest : public ::testing::Test {
 protected:
  /* Make a box and subdivide it into 6 tetrahedra. */
  template <typename T>
  geometry::VolumeMesh<T> MakeBoxTetMesh() {
    const double length = 0.1;
    geometry::Box box(length, length, length);
    geometry::VolumeMesh<T> mesh =
        geometry::internal::MakeBoxVolumeMesh<T>(box, length);
    DRAKE_DEMAND(mesh.num_elements() == 6);
    return mesh;
  }

  template <typename FemModel>
  void AddBoxToModel(FemModel* fem_model) {
    using T = typename FemModel::T;
    geometry::VolumeMesh<T> mesh = MakeBoxTetMesh<T>();
    const typename FemModel::ConstitutiveModel constitutive_model(
        kYoungsModulus, kPoissonRatio);
    const DampingModel<T> damping_model(kMassDamping, kStiffnessDamping);
    fem_model->AddDynamicElasticityElementsFromTetMesh(mesh, constitutive_model,
                                                       kDensity, damping_model);
  }

  void SetUp() override { AddBoxToModel(&model_); }

  /* Returns an arbitrary perturbation to the reference configuration of the
   cube geometry. */
  static Vector<double, kNumDofs> perturbation() {
    Vector<double, kNumDofs> delta;
    delta << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53,
        0.67, 0.81, 0.36, 0.45, 0.31, 0.29, 0.71, 0.30, 0.68, 0.58, 0.52, 0.35,
        0.76;
    return delta;
  }

  /* Returns an arbitrary FemState whose generalized positions are different
   from reference positions and whose velocities and acclerations are nonzero.
   In addition, set up AutoDiff derivatives for qddot if the scalar type is
   AutoDiffXd. */
  template <typename FemModelType>
  typename FemModelType::State MakeDeformedState(
      const FemModelType& fem_model) const {
    using State = typename FemModelType::State;
    using T = typename FemModelType::T;

    const State reference_state = fem_model.MakeFemState();
    State deformed_state = fem_model.MakeFemState();
    /* Perturb qddot and set up derivatives if necessary. */
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      const Vector<double, kNumDofs> perturbed_qddot =
          math::ExtractValue(deformed_state.qddot()) + perturbation();
      Vector<AutoDiffXd, kNumDofs> perturbed_qddot_autodiff;
      math::InitializeAutoDiff(perturbed_qddot, &perturbed_qddot_autodiff);
      /* It's important to set up the `deformed_state` with AdvanceOneTimeStep()
       so that the derivatives such as dq/dqddot are set up. */
      fem_model.AdvanceOneTimeStep(reference_state, perturbed_qddot_autodiff,
                                   &deformed_state);
    } else {
      const Vector<double, kNumDofs> perturbed_qddot =
          deformed_state.qddot() + perturbation();
      fem_model.AdvanceOneTimeStep(reference_state, perturbed_qddot,
                                   &deformed_state);
    }

    return deformed_state;
  }

  /* The model under test. */
  DynamicElasticityModel<AutoDiffElement> model_{kDt};
};

/* Tests the mesh has been successfully converted to elements. */
TEST_F(DynamicElasticityModelTest, Geometry) {
  EXPECT_EQ(model_.num_nodes(), kNumCubeVertices);
  EXPECT_EQ(model_.num_elements(), kNumElements);
}

/* Tests that the tangent matrix of the model is the derivative of the residual
 with respect to the change in qddot. */
TEST_F(DynamicElasticityModelTest, TangentMatrixIsResidualDerivative) {
  using T = AutoDiffXd;

  const FemState<AutoDiffElement> state = MakeDeformedState(model_);
  VectorX<T> residual(state.num_generalized_positions());
  model_.CalcResidual(state, &residual);

  Eigen::SparseMatrix<T> tangent_matrix;
  model_.SetTangentMatrixSparsityPattern(&tangent_matrix);
  model_.CalcTangentMatrix(state, &tangent_matrix);

  /* In the discretization of the unit cube by 6 tetrahedra, there are 19 edges,
   and 8 nodes, creating 19*2 + 8 blocks of 3-by-3 nonzero entries. Hence the
   number of nonzero entries of the tangent matrix should be (19*2+8)*9. */
  const int nnz = (19 * 2 + 8) * 9;
  EXPECT_EQ(tangent_matrix.nonZeros(), nnz);

  const MatrixX<T> dense_tangent_matrix(tangent_matrix);
  for (int i = 0; i < state.num_generalized_positions(); ++i) {
    /* The tangent matrix should be the derivative of the residual. Notice that
     here we are comparing a VectorX<double> and the value of a
     VectorX<AutoDiffXd>. */
    EXPECT_TRUE(CompareMatrices(residual(i).derivatives(),
                                dense_tangent_matrix.col(i),
                                4 * std::numeric_limits<double>::epsilon()));
  }
}

/* Verifies that the tangent matrix calculated as PETSc matrix is the same as
 that calculated as Eigen::SparseMatrix. */
TEST_F(DynamicElasticityModelTest, TangentMatrixParity) {
  const FemState<AutoDiffElement> state = MakeDeformedState(model_);
  Eigen::SparseMatrix<AutoDiffXd> eigen_tangent_matrix;
  model_.SetTangentMatrixSparsityPattern(&eigen_tangent_matrix);
  model_.CalcTangentMatrix(state, &eigen_tangent_matrix);
  const MatrixX<AutoDiffXd> eigen_dense_autodiff_matrix = eigen_tangent_matrix;
  const MatrixXd eigen_dense_matrix =
      math::ExtractValue(eigen_dense_autodiff_matrix);

  DynamicElasticityModel<DoubleElement> double_model(kDt);
  AddBoxToModel(&double_model);
  const FemState<DoubleElement> double_state = MakeDeformedState(double_model);
  std::unique_ptr<internal::PetscSymmetricBlockSparseMatrix>
      petsc_tangent_matrix =
          double_model.MakePetscSymmetricBlockSparseTangentMatrix();
  double_model.CalcTangentMatrix(double_state, petsc_tangent_matrix.get());
  petsc_tangent_matrix->AssembleIfNecessary();
  const MatrixXd petsc_dense_matrix = petsc_tangent_matrix->MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(eigen_dense_matrix, petsc_dense_matrix,
                              std::numeric_limits<double>::epsilon()));
}

/* Adds two copies of the same set of elements and tests that the residual for
 the two copies are identical. In particular, tests that the node offsets in
 AddDynamicElasticityElementsFromTetMesh() are working as intended. */
TEST_F(DynamicElasticityModelTest, MultipleMesh) {
  using T = AutoDiffXd;
  /* Add a second box mesh to the model. */
  AddBoxToModel(&model_);
  EXPECT_EQ(model_.num_nodes(), 2 * kNumCubeVertices);
  /* Each cube is split into 6 tetrahedra. */
  EXPECT_EQ(model_.num_elements(), 2 * kNumElements);

  FemState<AutoDiffElement> state = model_.MakeFemState();
  EXPECT_EQ(state.num_generalized_positions(), 2 * kNumDofs);

  VectorX<T> residual(state.num_generalized_positions());
  model_.CalcResidual(state, &residual);
  EXPECT_TRUE(
      CompareMatrices(residual.head(kNumDofs), residual.tail(kNumDofs), 0));
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
