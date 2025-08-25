#pragma once

#include <memory>
#include <string>

#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {

using Eigen::Translation3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::Parser;

namespace multibody {
namespace test {

// Builds a model representative of a world that only contains anchored
// geometry, and thus zero DOFs, and adds that world to the plant. The setup
// has a robot table, a table that holds objects and a mug welded to that table.
template <typename T>
void AddFixedObjectsToPlant(MultibodyPlant<T>* plant) {
  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");
  const std::string mug_sdf_path =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");

  // Load a model of a table for a robot.
  const ModelInstanceIndex robot_table_model =
      Parser(plant, "robot").AddModels(table_sdf_path).at(0);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("link", robot_table_model));

  // Load a second table for objects.
  Parser objects_parser(plant, "objects");
  const ModelInstanceIndex objects_table_model =
      objects_parser.AddModels(table_sdf_path).at(0);
  const RigidTransformd X_WT(Vector3d(0.8, 0.0, 0.0));
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("link", objects_table_model), X_WT);

  // Define a fixed frame on the -x, -y corner of the objects table.
  const double table_top_z_in_world =
      // table's top height
      0.736 +
      // table's top width
      0.057 / 2;
  const RigidTransformd X_TO(RotationMatrixd::MakeXRotation(-M_PI_2),
                             Vector3d(-0.3, -0.3, table_top_z_in_world));
  const auto& objects_frame_O =
      plant->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          "objects_frame", plant->GetFrameByName("link", objects_table_model),
          X_TO));

  // Add a mug and weld it to the table.
  objects_parser.AddModels(mug_sdf_path);
  const RigidBody<double>& mug = plant->GetBodyByName("simple_mug");

  // Weld the mug to the table with its center 5 cm above the table, i.e. with
  // its base on the table.
  plant->WeldFrames(objects_frame_O, mug.body_frame(),
                    Translation3d(0.0, 0.0, 0.05));
}

}  // namespace test
}  // namespace multibody
}  // namespace drake
