#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {

using multibody::Parser;
using systems::Context;

namespace multibody {
namespace {

// We verify the computation of the mass matrix by comparing two significantly
// different implementations:
//   - CalcMassMatrix(): uses the Composite Body Algorithm.
//   - CalcMassMatrixViaInverseDynamics(): uses inverse dynamics to compute each
//     column of the mass matrix at a time.
GTEST_TEST(MultibodyPlantMassMatrix, IiwaRobot) {
  // TODO(amcastro-tri): add unit testing for branched tree structures and
  // chains of massless bodies.
  const std::string model_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  parser.AddModelFromFile(model_path);
  plant.Finalize();

  // We did not weld the arm to the world, therefore we expect it to be free.
  EXPECT_EQ(plant.num_velocities(), 13);

  // Create a context and store an arbitrary configuration.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  const VectorX<double> q0 = VectorX<double>::LinSpaced(
      plant.num_positions(), 1, plant.num_positions());
  plant.SetPositions(context.get(), q0);

  // Compute mass matrix via the Composite Body Algorithm.
  MatrixX<double> Mcba(plant.num_velocities(), plant.num_velocities());
  plant.CalcMassMatrix(*context, &Mcba);

  // Compute mass matrix using inverse dynamics for each column.
  MatrixX<double> Mid(plant.num_velocities(), plant.num_velocities());
  plant.CalcMassMatrixViaInverseDynamics(*context, &Mid);

  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(
      CompareMatrices(Mcba, Mid, kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
