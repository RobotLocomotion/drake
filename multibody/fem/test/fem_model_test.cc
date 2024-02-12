#include "drake/multibody/fem/fem_model.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/linear_corotated_model.h"
#include "drake/multibody/fem/linear_simplex_element.h"
#include "drake/multibody/fem/simplex_gaussian_quadrature.h"
#include "drake/multibody/fem/test/dummy_model.h"
#include "drake/multibody/fem/volumetric_element.h"
#include "drake/multibody/fem/volumetric_model.h"
#include "drake/multibody/plant/multibody_plant.h"

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
using LinearDummyElement = DummyElement<true>;
using LinearDummyModel = DummyModel<true>;

GTEST_TEST(FemModelTest, Constructor) {
  LinearDummyModel model;
  LinearDummyModel::DummyBuilder builder(&model);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  constexpr int kNumNodes = 6;
  constexpr int kNumElements = 2;
  EXPECT_EQ(model.num_elements(), kNumElements);
  EXPECT_EQ(model.num_nodes(), kNumNodes);
  EXPECT_EQ(model.num_dofs(), kNumNodes * 3);
  /* Dummy model uses linear constitutive model and is therefore linear. */
  EXPECT_TRUE(model.is_linear());
}

GTEST_TEST(FemModelTest, CalcResidual) {
  LinearDummyModel model;
  LinearDummyModel::DummyBuilder builder(&model);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  unique_ptr<FemState<double>> fem_state = model.MakeFemState();
  VectorXd residual(model.num_dofs());
  const systems::LeafContext<double> dummy_context;
  const FemPlantData<double> dummy_data{dummy_context, {}};
  model.CalcResidual(*fem_state, dummy_data, &residual);

  VectorXd expected_residual = VectorXd::Zero(model.num_dofs());
  expected_residual.head<LinearDummyElement::kNumDofs>() +=
      LinearDummyElement::inverse_dynamics_force();
  expected_residual.tail<LinearDummyElement::kNumDofs>() +=
      LinearDummyElement::inverse_dynamics_force();

  /* The residual for each element is set to a dummy value if all states are
   zero (see DummyElement::CalcResidual). */
  EXPECT_EQ(residual, expected_residual);
}

GTEST_TEST(FemModelTest, CalcResidualWithExternalForce) {
  LinearDummyModel model;
  LinearDummyModel::DummyBuilder builder(&model);
  /* Add a few elements that don't share nodes. The non-overlapping elements
   simplifies the computation for easier testing. */
  builder.AddElementWithDistinctNodes();
  builder.AddElementWithDistinctNodes();
  builder.Build();
  unique_ptr<FemState<double>> fem_state = model.MakeFemState();

  const double mass_density = 2.7;
  Vector3d gravity_vector(0, 0, -9.81);
  GravityForceField<double> gravity_field(gravity_vector, mass_density);
  /* The gravity force field doesn't depend on Context, but a Context is needed
   formally. So we create a dummy Context that's unused. */
  MultibodyPlant<double> plant(0.01);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  const FemPlantData<double> plant_data{*context, {&gravity_field}};
  VectorXd residual(model.num_dofs());
  model.CalcResidual(*fem_state, plant_data, &residual);

  /* Since DummyElement adds the external force density directly to the
   residual (see DummyElement::DoAddScaledExternalForces for detail), we expect
   the difference between inverse dynamics force and the residual to be equal
   to the gravity force density. */
  VectorXd expected_residual = VectorXd::Zero(model.num_dofs());
  expected_residual.head<LinearDummyElement::kNumDofs>() +=
      LinearDummyElement::inverse_dynamics_force();
  expected_residual.tail<LinearDummyElement::kNumDofs>() +=
      LinearDummyElement::inverse_dynamics_force();
  for (int i = 0; i < model.num_nodes(); ++i) {
    expected_residual.segment<3>(3 * i) -= gravity_vector * mass_density;
  }

  EXPECT_TRUE(CompareMatrices(expected_residual, residual));
}

/* Similar to CalcResidualWithExternalForce, but focuses on
 testing the data flow from context to residual when the force density field
 depends on the context. */
GTEST_TEST(FemModelTest, CalcResidualWithContextDependentExternalForce) {
  LinearDummyModel model;
  LinearDummyModel::DummyBuilder builder(&model);
  builder.AddElementWithDistinctNodes();
  builder.Build();
  unique_ptr<FemState<double>> fem_state = model.MakeFemState();

  /* A force field where the magnitude of the force density depends on time. */
  class TimeScaledForceDensityField final : public ForceDensityField<double> {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TimeScaledForceDensityField)

    /* Constructs a force field that implements the test force f = time *
     unit_vector`,· where `time` is the time currently stored in the MbP's
     context  */
    explicit TimeScaledForceDensityField(const Vector3d unit_vector)
        : unit_vector_(unit_vector) {}

   private:
    Vector3<double> DoEvaluateAt(const systems::Context<double>& context,
                                 const Vector3<double>&) const final {
      return context.get_time() * unit_vector_;
    };

    std::unique_ptr<ForceDensityField<double>> DoClone() const final {
      return std::make_unique<TimeScaledForceDensityField>(*this);
    }

    Vector3d unit_vector_;
  };

  const Vector3d unit_vector = Vector3d(1, 2, 3).normalized();
  TimeScaledForceDensityField force_field(unit_vector);

  MultibodyPlant<double> plant(0.01);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  VectorXd residual_without_external_force(model.num_dofs());
  const systems::LeafContext<double> dummy_context;
  const FemPlantData<double> dummy_data{dummy_context, {}};
  model.CalcResidual(*fem_state, dummy_data, &residual_without_external_force);
  /* We evaluate the residual at two different times and verifies that the ratio
   between the external forces would be equal to the ratio of times. */
  const double kTime = 1.23;
  context->SetTime(kTime);
  VectorXd external_force;
  {
    const fem::FemPlantData<double> plant_data{*context, {&force_field}};
    VectorXd residual(model.num_dofs());
    model.CalcResidual(*fem_state, plant_data, &residual);
    external_force = residual - residual_without_external_force;
  }

  context->SetTime(2.0 * kTime);
  VectorXd external_force2;
  {
    const fem::FemPlantData<double> plant_data{*context, {&force_field}};
    VectorXd residual(model.num_dofs());
    model.CalcResidual(*fem_state, plant_data, &residual);
    external_force2 = residual - residual_without_external_force;
  }

  EXPECT_TRUE(CompareMatrices(2.0 * external_force, external_force2,
                              4.0 * std::numeric_limits<double>::epsilon()));
}

GTEST_TEST(FemModelTest, CalcTangentMatrix) {
  LinearDummyModel model;
  LinearDummyModel::DummyBuilder builder(&model);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  unique_ptr<FemState<double>> fem_state = model.MakeFemState();
  unique_ptr<contact_solvers::internal::Block3x3SparseSymmetricMatrix>
      tangent_matrix = model.MakeTangentMatrix();
  ASSERT_EQ(tangent_matrix->rows(), model.num_dofs());
  ASSERT_EQ(tangent_matrix->cols(), model.num_dofs());
  const Vector3d weights(0.1, 0.2, 0.3);
  model.CalcTangentMatrix(*fem_state, weights, tangent_matrix.get());

  MatrixXd expected_mass_matrix =
      MatrixXd::Zero(model.num_dofs(), model.num_dofs());
  expected_mass_matrix.topLeftCorner(LinearDummyElement::kNumDofs,
                                     LinearDummyElement::kNumDofs) +=
      LinearDummyElement::mass_matrix();
  expected_mass_matrix.bottomRightCorner(LinearDummyElement::kNumDofs,
                                         LinearDummyElement::kNumDofs) +=
      LinearDummyElement::mass_matrix();
  MatrixXd expected_stiffness_matrix =
      MatrixXd::Zero(model.num_dofs(), model.num_dofs());
  expected_stiffness_matrix.topLeftCorner(LinearDummyElement::kNumDofs,
                                          LinearDummyElement::kNumDofs) +=
      LinearDummyElement::stiffness_matrix();
  expected_stiffness_matrix.bottomRightCorner(LinearDummyElement::kNumDofs,
                                              LinearDummyElement::kNumDofs) +=
      LinearDummyElement::stiffness_matrix();

  const MatrixXd expected_damping_matrix =
      LinearDummyModel::kMassDamping * expected_mass_matrix +
      LinearDummyModel::kStiffnessDamping * expected_stiffness_matrix;
  const MatrixXd expected_tangent_matrix =
      weights(0) * expected_stiffness_matrix +
      weights(1) * expected_damping_matrix + weights(2) * expected_mass_matrix;
  EXPECT_TRUE(CompareMatrices(tangent_matrix->MakeDenseMatrix(),
                              expected_tangent_matrix,
                              4.0 * std::numeric_limits<double>::epsilon(),
                              MatrixCompareType::relative));
}

GTEST_TEST(FemModelTest, CalcTangentMatrixNoAutoDiff) {
  using T = AutoDiffXd;
  constexpr int kNaturalDimension = 3;
  constexpr int kSpatialDimension = 3;
  constexpr int kQuadratureOrder = 1;
  using QuadratureType =
      fem::internal::SimplexGaussianQuadrature<kNaturalDimension,
                                               kQuadratureOrder>;
  constexpr int kNumQuads = QuadratureType::num_quadrature_points;
  using IsoparametricElementType =
      fem::internal::LinearSimplexElement<T, kNaturalDimension,
                                          kSpatialDimension, kNumQuads>;
  using ConstitutiveModelType =
      fem::internal::LinearCorotatedModel<T, kNumQuads>;
  using FemElementType =
      fem::internal::VolumetricElement<IsoparametricElementType, QuadratureType,
                                       ConstitutiveModelType>;
  using FemModelType = fem::internal::VolumetricModel<FemElementType>;
  auto fem_model = std::make_unique<FemModelType>();
  DRAKE_EXPECT_THROWS_MESSAGE(fem_model->MakeTangentMatrix(),
                              ".*only.*double.*");
  unique_ptr<FemState<T>> fem_state = fem_model->MakeFemState();
  contact_solvers::internal::BlockSparsityPattern empty_pattern({}, {});
  contact_solvers::internal::Block3x3SparseSymmetricMatrix tangent_matrix(
      empty_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(
      fem_model->CalcTangentMatrix(*fem_state, Vector3<T>(0.1, 0.2, 0.3),
                                   &tangent_matrix),
      ".*only.*double.*");
}

/* Verifies that performing calculations on incompatible model and states throws
 an exception. */
GTEST_TEST(FemModelTest, IncompatibleModelState) {
  /* Build a model with two elements and make a compatible state. */
  LinearDummyModel model;
  LinearDummyModel::DummyBuilder builder(&model);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  unique_ptr<FemState<LinearDummyModel::T>> fem_state = model.MakeFemState();
  /* Add another element so that the model and the state are no longer
   compatible. */
  LinearDummyModel::DummyBuilder builder2(&model);
  builder2.AddElementWithDistinctNodes();
  builder2.Build();

  /* Trying to calculate residual with the old state causes an exception. */
  VectorXd residual(model.num_dofs());
  const systems::LeafContext<double> dummy_context;
  const FemPlantData<double> dummy_data{dummy_context, {}};
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcResidual(*fem_state, dummy_data, &residual),
      "CalcResidual.* model and state are not compatible.");

  /* Trying to calculate tangent matrix with the old state causes an exception.
   */
  unique_ptr<contact_solvers::internal::Block3x3SparseSymmetricMatrix>
      tangent_matrix = model.MakeTangentMatrix();
  ASSERT_EQ(tangent_matrix->rows(), model.num_dofs());
  ASSERT_EQ(tangent_matrix->cols(), model.num_dofs());
  const Vector3d weights(0.1, 0.2, 0.3);
  DRAKE_EXPECT_THROWS_MESSAGE(
      model.CalcTangentMatrix(*fem_state, weights, tangent_matrix.get()),
      "CalcTangentMatrix.* model and state are not compatible.");
}

/* Verifies that multiple builders can build into the same FemModel. */
GTEST_TEST(FemModelTest, MultipleBuilders) {
  LinearDummyModel model;
  LinearDummyModel::DummyBuilder builder0(&model);
  builder0.AddElementWithDistinctNodes();
  builder0.Build();
  EXPECT_EQ(model.num_nodes(), LinearDummyElement::Traits::num_nodes);

  LinearDummyModel::DummyBuilder builder1(&model);
  builder1.AddElementWithDistinctNodes();
  builder1.Build();
  EXPECT_EQ(model.num_nodes(), 2 * LinearDummyElement::Traits::num_nodes);

  /* Reusing builder throws an exception. */
  DRAKE_EXPECT_THROWS_MESSAGE(builder0.AddElementWithDistinctNodes(),
                              "Build.* has been called.*");
}

/* Verifies we can add a Dirichlet boundary condition to FEM models, and it is
 correctly invoked on the state, residual, and tangent matrix. */
GTEST_TEST(FemModelTest, DirichletBoundaryCondition) {
  LinearDummyModel model;
  LinearDummyModel::DummyBuilder builder(&model);
  builder.AddElementWithDistinctNodes();
  builder.Build();

  using T = LinearDummyModel::T;

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
  bc.AddBoundaryCondition(FemNodeIndex(0),
                          {Vector3<double>(1, 1, 1), Vector3<double>(2, 2, 2),
                           Vector3<double>(3, 3, 3)});
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

  LinearDummyModel model_without_bc;
  LinearDummyModel::DummyBuilder builder1(&model_without_bc);
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
    const systems::LeafContext<double> dummy_context;
    const FemPlantData<double> dummy_data{dummy_context, {}};
    model.CalcResidual(*state0, dummy_data, &residual0);

    unique_ptr<FemState<T>> state1 = model_without_bc.MakeFemState();
    state1->SetPositions(VectorX<T>::Zero(model.num_dofs()));
    state1->SetVelocities(VectorX<T>::Zero(model.num_dofs()));
    state1->SetAccelerations(VectorX<T>::Zero(model.num_dofs()));
    VectorXd residual1(model_without_bc.num_dofs());
    model_without_bc.CalcResidual(*state1, dummy_data, &residual1);
    EXPECT_FALSE(CompareMatrices(residual0, residual1));

    bc.ApplyHomogeneousBoundaryCondition(&residual1);
    EXPECT_TRUE(CompareMatrices(residual0, residual1));
  }

  /* Verify that BC is applied to the tangent matrix, but it doesn't matter
   whether BC is applied from the model or directly from the BC.*/
  {
    const Vector3d weights(0.1, 0.2, 0.3);

    unique_ptr<FemState<T>> state0 = model.MakeFemState();
    unique_ptr<contact_solvers::internal::Block3x3SparseSymmetricMatrix>
        tangent_matrix0 = model.MakeTangentMatrix();
    model.CalcTangentMatrix(*state0, weights, tangent_matrix0.get());
    const MatrixX<T> dense_tangent_matrix0 = tangent_matrix0->MakeDenseMatrix();

    unique_ptr<FemState<T>> state1 = model_without_bc.MakeFemState();
    unique_ptr<contact_solvers::internal::Block3x3SparseSymmetricMatrix>
        tangent_matrix1 = model.MakeTangentMatrix();
    model_without_bc.CalcTangentMatrix(*state1, weights, tangent_matrix1.get());
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
