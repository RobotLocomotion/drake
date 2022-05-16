#include "drake/multibody/fem/fem_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/test/dummy_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;

GTEST_TEST(FemModelTest, Constructor) {
  DummyModel model;
  DummyModel::DummyBuilder builder(&model);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  constexpr int kNumNodes = 6;
  constexpr int kNumElements = 2;
  EXPECT_EQ(model.num_elements(), kNumElements);
  EXPECT_EQ(model.num_nodes(), kNumNodes);
  EXPECT_EQ(model.num_dofs(), kNumNodes * 3);
}

GTEST_TEST(FemModelTest, CalcResidual) {
  DummyModel model;
  DummyModel::DummyBuilder builder(&model);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  unique_ptr<FemState<double>> fem_state = model.MakeFemState();
  VectorXd residual(model.num_dofs());
  model.CalcResidual(*fem_state, &residual);

  VectorXd expected_residual = VectorXd::Zero(model.num_dofs());
  expected_residual.head<DummyElement::kNumDofs>() +=
      DummyElement::inverse_dynamics_force();
  expected_residual.tail<DummyElement::kNumDofs>() +=
      DummyElement::inverse_dynamics_force();

  /* The residual for each element is set to a dummy value if all states are
   zero (see DummyElement::CalcResidual). */
  EXPECT_EQ(residual, expected_residual);
}

GTEST_TEST(FemModelTest, CalcTangentMatrix) {
  DummyModel model;
  DummyModel::DummyBuilder builder(&model);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  unique_ptr<FemState<double>> fem_state = model.MakeFemState();
  unique_ptr<internal::PetscSymmetricBlockSparseMatrix> tangent_matrix =
      model.MakePetscSymmetricBlockSparseTangentMatrix();
  ASSERT_EQ(tangent_matrix->rows(), model.num_dofs());
  ASSERT_EQ(tangent_matrix->cols(), model.num_dofs());
  const Vector3d weights(0.1, 0.2, 0.3);
  model.CalcTangentMatrix(*fem_state, weights, tangent_matrix.get());
  tangent_matrix->AssembleIfNecessary();
  const MatrixXd tangent_matrix_dense = tangent_matrix->MakeDenseMatrix();

  MatrixXd expected_mass_matrix =
      MatrixXd::Zero(model.num_dofs(), model.num_dofs());
  expected_mass_matrix.topLeftCorner(DummyElement::kNumDofs,
                                     DummyElement::kNumDofs) +=
      DummyElement::mass_matrix();
  expected_mass_matrix.bottomRightCorner(DummyElement::kNumDofs,
                                         DummyElement::kNumDofs) +=
      DummyElement::mass_matrix();
  MatrixXd expected_stiffness_matrix =
      MatrixXd::Zero(model.num_dofs(), model.num_dofs());
  expected_stiffness_matrix.topLeftCorner(DummyElement::kNumDofs,
                                          DummyElement::kNumDofs) +=
      DummyElement::stiffness_matrix();
  expected_stiffness_matrix.bottomRightCorner(DummyElement::kNumDofs,
                                              DummyElement::kNumDofs) +=
      DummyElement::stiffness_matrix();

  const MatrixXd expected_damping_matrix =
      DummyModel::kMassDamping * expected_mass_matrix +
      DummyModel::kStiffnessDamping * expected_stiffness_matrix;
  const MatrixXd expected_tangent_matrix =
      weights(0) * expected_stiffness_matrix +
      weights(1) * expected_damping_matrix + weights(2) * expected_mass_matrix;
  EXPECT_TRUE(CompareMatrices(tangent_matrix_dense, expected_tangent_matrix,
                              std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

/* Verifies that performing calculations on incompatible model and states throws
 an exception. */
GTEST_TEST(FemModelTest, IncompatibleModelState) {
  /* Build a model with two elements and make a compatible state. */
  DummyModel model;
  DummyModel::DummyBuilder builder(&model);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  unique_ptr<FemState<DummyModel::T>> fem_state = model.MakeFemState();
  /* Add another element so that the model and the state are no longer
   compatible. */
  DummyModel::DummyBuilder builder2(&model);
  builder2.AddElementWithDistinctNodes();
  builder2.Build();

  /* Trying to calculate residual with the old state causes an exception. */
  VectorXd residual(model.num_dofs());
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcResidual(*fem_state, &residual),
      "CalcResidual.* model and state are not compatible.");

  /* Trying to calculate tangent matrix with the old state causes an exception.
   */
  unique_ptr<internal::PetscSymmetricBlockSparseMatrix> tangent_matrix =
      model.MakePetscSymmetricBlockSparseTangentMatrix();
  ASSERT_EQ(tangent_matrix->rows(), model.num_dofs());
  ASSERT_EQ(tangent_matrix->cols(), model.num_dofs());
  const Vector3d weights(0.1, 0.2, 0.3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcTangentMatrix(*fem_state, weights, tangent_matrix.get()),
      "CalcTangentMatrix.* model and state are not compatible.");
}

/* Verifies that multiple builders can build into the same FemModel. */
GTEST_TEST(FemModelTest, MultipleBuilders) {
  DummyModel model;
  DummyModel::DummyBuilder builder0(&model);
  builder0.AddElementWithDistinctNodes();
  builder0.Build();
  EXPECT_EQ(model.num_nodes(), DummyElement::Traits::num_nodes);

  DummyModel::DummyBuilder builder1(&model);
  builder1.AddElementWithDistinctNodes();
  builder1.Build();
  EXPECT_EQ(model.num_nodes(), 2 * DummyElement::Traits::num_nodes);

  /* Reusing builder throws an exception. */
  DRAKE_EXPECT_THROWS_MESSAGE(builder0.AddElementWithDistinctNodes(),
                              "Build.* has been called.*");
}

GTEST_TEST(FemModelTest, Gravity) {
  DummyModel model;
  EXPECT_EQ(model.gravity_vector(), Vector3<double>(0, 0, -9.81));
  model.set_gravity_vector(Vector3<double>(1, 2, 3));
  EXPECT_EQ(model.gravity_vector(), Vector3<double>(1, 2, 3));
}

/* Verifies we can add a Dirichlet boundary condition to FEM models, and it is
 correctly invoked on the state, residual, and tangent matrix. */
GTEST_TEST(FemModelTest, DirichletBoundaryCondition) {
  DummyModel model;
  DummyModel::DummyBuilder builder(&model);
  builder.AddElementWithDistinctNodes();
  builder.Build();

  using T = DummyModel::T;

  auto are_same_states = [](const FemState<T>& a, const FemState<T>& b) {
    return a.GetAccelerations() == b.GetAccelerations() &&
           a.GetVelocities() == b.GetVelocities() &&
           a.GetPositions() == b.GetPositions();
  };

  /* ApplyBoundaryCondition() is a no-op if no BC is prescribed. */
  {
    unique_ptr<FemState<T>> state0 = model.MakeFemState();
    unique_ptr<FemState<T>> state1 = model.MakeFemState();
    model.ApplyBoundaryCondition(state1.get());
    EXPECT_TRUE(are_same_states(*state0, *state1));
  }

  /* Create a BC and add it to the model. */
  DirichletBoundaryCondition<double> bc;
  bc.AddBoundaryCondition(0, Vector3<double>(3, 2, 1));
  model.SetDirichletBoundaryCondition(bc);
  /* Verify that BC is applied to the state, but it doesn't matter whether BC is
   applied from the model or directly from the BC.*/
  {
    unique_ptr<FemState<T>> state0 = model.MakeFemState();
    unique_ptr<FemState<T>> state1 = model.MakeFemState();
    model.ApplyBoundaryCondition(state1.get());
    unique_ptr<FemState<T>> state2 = model.MakeFemState();
    bc.ApplyBoundaryConditionToState(state2.get());
    EXPECT_FALSE(are_same_states(*state0, *state1));
    EXPECT_TRUE(are_same_states(*state1, *state2));
  }

  DummyModel model_without_bc;
  DummyModel::DummyBuilder builder1(&model_without_bc);
  builder1.AddElementWithDistinctNodes();
  builder1.Build();
  /* Verify that BC is applied to the residual, but it doesn't matter whether BC
   is applied from the model or directly from the BC.*/
  {
    unique_ptr<FemState<T>> state0 = model.MakeFemState();
    /* DummyModel gives all zero residual for nonzero state, so we set to the
     zero state so that we have a non-zero residual. */
    state0->SetPositions(VectorX<T>::Zero(model.num_dofs()));
    state0->SetVelocities(VectorX<T>::Zero(model.num_dofs()));
    state0->SetAccelerations(VectorX<T>::Zero(model.num_dofs()));
    VectorXd residual0(model.num_dofs());
    model.CalcResidual(*state0, &residual0);

    unique_ptr<FemState<T>> state1 = model_without_bc.MakeFemState();
    state1->SetPositions(VectorX<T>::Zero(model.num_dofs()));
    state1->SetVelocities(VectorX<T>::Zero(model.num_dofs()));
    state1->SetAccelerations(VectorX<T>::Zero(model.num_dofs()));
    VectorXd residual1(model_without_bc.num_dofs());
    model_without_bc.CalcResidual(*state1, &residual1);
    EXPECT_FALSE(CompareMatrices(residual0, residual1));

    bc.ApplyHomogeneousBoundaryCondition(&residual1);
    EXPECT_TRUE(CompareMatrices(residual0, residual1));
  }

  /* Verify that BC is applied to the tangent matrix, but it doesn't matter
   whether BC is applied from the model or directly from the BC.*/
  {
    const Vector3d weights(0.1, 0.2, 0.3);

    unique_ptr<FemState<T>> state0 = model.MakeFemState();
    unique_ptr<internal::PetscSymmetricBlockSparseMatrix> tangent_matrix0 =
        model.MakePetscSymmetricBlockSparseTangentMatrix();
    model.CalcTangentMatrix(*state0, weights, tangent_matrix0.get());
    tangent_matrix0->AssembleIfNecessary();
    const MatrixX<T> dense_tangent_matrix0 = tangent_matrix0->MakeDenseMatrix();

    unique_ptr<FemState<T>> state1 = model_without_bc.MakeFemState();
    unique_ptr<internal::PetscSymmetricBlockSparseMatrix> tangent_matrix1 =
        model.MakePetscSymmetricBlockSparseTangentMatrix();
    model_without_bc.CalcTangentMatrix(*state1, weights, tangent_matrix1.get());
    tangent_matrix1->AssembleIfNecessary();
    MatrixX<T> dense_tangent_matrix1 = tangent_matrix1->MakeDenseMatrix();
    EXPECT_FALSE(CompareMatrices(dense_tangent_matrix0, dense_tangent_matrix1));

    bc.ApplyBoundaryConditionToTangentMatrix(tangent_matrix1.get());
    dense_tangent_matrix1 = tangent_matrix1->MakeDenseMatrix();
    EXPECT_TRUE(CompareMatrices(dense_tangent_matrix0, dense_tangent_matrix1));
  }
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
