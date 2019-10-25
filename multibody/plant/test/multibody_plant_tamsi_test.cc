#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using Eigen::Translation3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::Parser;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

namespace multibody {
namespace {

// We build a model representative of a world that only contains anchored
// geometry, and thus zero DOFs. The setup has a robot table, an objects table
// and a mug welded to the objects table. We then test we can do discrete
// updates even when the model has no DOFs. Relates to #12066.
GTEST_TEST(MbpWithTamsiSolver, EmptyWorld) {
  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");
  const std::string mug_sdf_path =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");

  systems::DiagramBuilder<double> builder;

  // We create a discrete model that uses the TAMSI solver for contact.
  const double discrete_update_period = 1.0e-3;
  auto items = AddMultibodyPlantSceneGraph(
      &builder,
      std::make_unique<MultibodyPlant<double>>(discrete_update_period));
  MultibodyPlant<double>& plant = items.plant;

  // Load a model of a table for a robot.
  Parser parser(&plant);
  const ModelInstanceIndex robot_table_model =
      parser.AddModelFromFile(table_sdf_path, "robot_table");
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", robot_table_model));

  // Load a second table for objects.
  const ModelInstanceIndex objects_table_model =
      parser.AddModelFromFile(table_sdf_path, "objects_table");
  const RigidTransformd X_WT(Vector3d(0.8, 0.0, 0.0));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", objects_table_model), X_WT);

  // Define a fixed frame on the -x, -y corner of the objects table.
  const double table_top_z_in_world =
      // table's top height
      0.736 +
      // table's top width
      0.057 / 2;
  const RigidTransformd X_TO(RotationMatrixd::MakeXRotation(-M_PI_2),
                             Vector3d(-0.3, -0.3, table_top_z_in_world));
  const auto& objects_frame_O =
      plant.AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          "objects_frame", plant.GetFrameByName("link", objects_table_model),
          X_TO));

  // Add a mug and weld it to the table.
  const ModelInstanceIndex mug_model = parser.AddModelFromFile(mug_sdf_path);
  const Body<double>& mug = plant.GetBodyByName("main_body", mug_model);
  // Weld the mug to the table with its center 5 cm above the table, i.e. with
  // its base on the table.
  plant.WeldFrames(objects_frame_O, mug.body_frame(),
                   Translation3d(0.0, 0.0, 0.05));

  // Done defining the world.
  plant.Finalize();

  // And build the Diagram:
  auto diagram = builder.Build();

  // The model has zero dofs.
  EXPECT_EQ(plant.num_velocities(), 0);
  EXPECT_EQ(plant.num_positions(), 0);

  // However it is not empty, it has all anchored geometry.
  EXPECT_EQ(plant.num_collision_geometries(), 3);

  auto context = diagram->CreateDefaultContext();

  auto& discrete_state_vector = context->get_discrete_state_vector();
  EXPECT_EQ(discrete_state_vector.size(), 0);

  auto new_discrete_state = diagram->AllocateDiscreteVariables();
  const systems::VectorBase<double>& new_discrete_state_vector =
      new_discrete_state->get_vector();
  EXPECT_EQ(new_discrete_state_vector.size(), 0);

  // Verify we can to do discrete updates even if we have zero DOFs.
  DRAKE_EXPECT_NO_THROW(
      diagram->CalcDiscreteVariableUpdates(*context, new_discrete_state.get()));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
