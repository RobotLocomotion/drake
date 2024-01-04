#include "drake/multibody/rational/test/rational_forward_kinematics_test_utilities.h"

#include <algorithm>
#include <optional>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
using drake::VectorX;
using drake::math::RigidTransformd;
using drake::multibody::BodyIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& iiwa_sdf_name, bool finalize) {
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/" + iiwa_sdf_name;

  auto plant = std::make_unique<MultibodyPlant<double>>(0.);
  Parser parser(plant.get());
  parser.AddModels(drake::FindResourceOrThrow(file_path));
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
  if (finalize) {
    plant->Finalize();
  }
  return plant;
}

Eigen::Matrix<double, 3, 8> GenerateBoxVertices(const Eigen::Vector3d& size,
                                                const RigidTransformd& pose) {
  Eigen::Matrix<double, 3, 8> vertices;
  // clang-format off
  vertices << 1, 1, 1, 1, -1, -1, -1, -1,
              1, 1, -1, -1, 1, 1, -1, -1,
              1, -1, 1, -1, 1, -1, 1, -1;
  // clang-format on
  for (int i = 0; i < 3; ++i) {
    DRAKE_DEMAND(size(i) > 0);
    vertices.row(i) *= size(i) / 2;
  }
  vertices = pose.rotation().matrix() * vertices +
             pose.translation() * Eigen::Matrix<double, 1, 8>::Ones();

  return vertices;
}

std::unique_ptr<MultibodyPlant<double>> ConstructDualArmIiwaPlant(
    const std::string& iiwa_sdf_name, const RigidTransformd& X_WL,
    const RigidTransformd& X_WR, ModelInstanceIndex* left_iiwa_instance,
    ModelInstanceIndex* right_iiwa_instance) {
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/" + iiwa_sdf_name);
  auto plant = std::make_unique<MultibodyPlant<double>>(0);
  *left_iiwa_instance = Parser(&*plant, "left").AddModels(iiwa_path).at(0);
  *right_iiwa_instance = Parser(&*plant, "right").AddModels(iiwa_path).at(0);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", *left_iiwa_instance),
                    X_WL);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", *right_iiwa_instance),
                    X_WR);

  plant->Finalize();
  return plant;
}

IiwaTest::IiwaTest()
    : iiwa_(ConstructIiwaPlant("iiwa14_no_collision.sdf", false)),
      scene_graph_{new geometry::SceneGraph<double>()},
      iiwa_tree_(drake::multibody::internal::GetInternalTree(*iiwa_)),
      world_{iiwa_->world_body().index()} {
  iiwa_->RegisterAsSourceForSceneGraph(scene_graph_.get());
  for (int i = 0; i < 8; ++i) {
    iiwa_link_[i] =
        iiwa_->GetBodyByName("iiwa_link_" + std::to_string(i)).index();
    iiwa_joint_[i] = iiwa_tree_.get_topology()
                         .get_rigid_body(iiwa_link_[i])
                         .inboard_mobilizer;
  }
}

FinalizedIiwaTest::FinalizedIiwaTest()
    : iiwa_(ConstructIiwaPlant("iiwa14_no_collision.sdf", true)),
      iiwa_tree_(drake::multibody::internal::GetInternalTree(*iiwa_)),
      world_{iiwa_->world_body().index()} {
  for (int i = 0; i < 8; ++i) {
    iiwa_link_[i] =
        iiwa_->GetBodyByName("iiwa_link_" + std::to_string(i)).index();
    iiwa_joint_[i] = iiwa_tree_.get_topology()
                         .get_rigid_body(iiwa_link_[i])
                         .inboard_mobilizer;
  }
}

void AddIiwaWithSchunk(const RigidTransformd& X_7S,
                       MultibodyPlant<double>* plant) {
  DRAKE_DEMAND(plant != nullptr);
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf";
  Parser(plant).AddModels(drake::FindResourceOrThrow(file_path));
  Parser(plant).AddModels(
      FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/"
                          "schunk_wsg_50_welded_fingers.sdf"));
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
  // weld the schunk gripper to iiwa link 7.
  plant->WeldFrames(plant->GetFrameByName("iiwa_link_7"),
                    plant->GetFrameByName("body"), X_7S);
}

void AddDualArmIiwa(const RigidTransformd& X_WL, const RigidTransformd& X_WR,
                    const RigidTransformd& X_7S, MultibodyPlant<double>* plant,
                    ModelInstanceIndex* left_iiwa_instance,
                    ModelInstanceIndex* right_iiwa_instance) {
  DRAKE_DEMAND(plant != nullptr);
  Parser left_parser(plant, "left");
  Parser right_parser(plant, "right");

  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  *left_iiwa_instance = left_parser.AddModels(iiwa_path).at(0);
  *right_iiwa_instance = right_parser.AddModels(iiwa_path).at(0);

  const std::string schunk_path = FindResourceOrThrow(
      "models/schunk/schunk_wsg_50_welded_fingers.sdf");
  const auto left_schunk_instance = left_parser.AddModels(schunk_path).at(0);
  const auto right_schunk_instance = right_parser.AddModels(schunk_path).at(0);

  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", *left_iiwa_instance),
                    X_WL);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", *right_iiwa_instance),
                    X_WR);
  // weld the schunk gripper to iiwa link 7.
  plant->WeldFrames(plant->GetFrameByName("iiwa_link_7", *left_iiwa_instance),
                    plant->GetFrameByName("body", left_schunk_instance), X_7S);
  plant->WeldFrames(plant->GetFrameByName("iiwa_link_7", *right_iiwa_instance),
                    plant->GetFrameByName("body", right_schunk_instance), X_7S);
}

}  // namespace multibody
}  // namespace drake
