#include "drake/multibody/dev/c_iris/test/rational_forward_kinematics_test_utilities.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace drake {
namespace multibody {
namespace c_iris {
using drake::VectorX;
using drake::math::RigidTransformd;
using drake::multibody::BodyIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

const double kInf = std::numeric_limits<double>::infinity();

std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& iiwa_sdf_name, bool finalize) {
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/" + iiwa_sdf_name;

  auto plant = std::make_unique<MultibodyPlant<double>>(0.);
  Parser parser(plant.get());
  parser.AddModelFromFile(drake::FindResourceOrThrow(file_path));
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
    DRAKE_ASSERT(size(i) > 0);
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
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/" + iiwa_sdf_name;
  auto plant = std::make_unique<MultibodyPlant<double>>(0);
  *left_iiwa_instance =
      Parser(plant.get())
          .AddModelFromFile(drake::FindResourceOrThrow(file_path), "left_iiwa");
  *right_iiwa_instance =
      Parser(plant.get())
          .AddModelFromFile(drake::FindResourceOrThrow(file_path),
                            "right_iiwa");
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
    iiwa_joint_[i] =
        iiwa_tree_.get_topology().get_body(iiwa_link_[i]).inboard_mobilizer;
  }
}

FinalizedIiwaTest::FinalizedIiwaTest()
    : iiwa_(ConstructIiwaPlant("iiwa14_no_collision.sdf", true)),
      iiwa_tree_(drake::multibody::internal::GetInternalTree(*iiwa_)),
      world_{iiwa_->world_body().index()} {
  for (int i = 0; i < 8; ++i) {
    iiwa_link_[i] =
        iiwa_->GetBodyByName("iiwa_link_" + std::to_string(i)).index();
    iiwa_joint_[i] =
        iiwa_tree_.get_topology().get_body(iiwa_link_[i]).inboard_mobilizer;
  }
}

void AddIiwaWithSchunk(const RigidTransformd& X_7S,
                       MultibodyPlant<double>* plant) {
  DRAKE_DEMAND(plant != nullptr);
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf";
  Parser(plant).AddModelFromFile(drake::FindResourceOrThrow(file_path));
  Parser(plant).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/wsg_50_description/sdf/"
                          "schunk_wsg_50_fixed_joint.sdf"));
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
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf";
  *left_iiwa_instance = Parser(plant).AddModelFromFile(
      drake::FindResourceOrThrow(file_path), "left_iiwa");
  *right_iiwa_instance = Parser(plant).AddModelFromFile(
      drake::FindResourceOrThrow(file_path), "right_iiwa");
  const auto left_schunk_instance = Parser(plant).AddModelFromFile(
      FindResourceOrThrow("models/schunk/schunk_wsg_50_fixed_joint.sdf"),
      "left_schunk");
  const auto right_schunk_instance = Parser(plant).AddModelFromFile(
      FindResourceOrThrow("models/schunk/schunk_wsg_50_fixed_joint.sdf"),
      "right_schunk");
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

void SetDiffuse(const MultibodyPlant<double>& plant,
                geometry::SceneGraph<double>* scene_graph,
                const BodyIndex body_index,
                const std::optional<std::string>& geometry_name,
                std::optional<double> rgba_r, std::optional<double> rgba_g,
                std::optional<double> rgba_b, std::optional<double> rgba_a) {
  const auto& inspector = scene_graph->model_inspector();
  const std::optional<geometry::FrameId> frame_id =
      plant.GetBodyFrameIdIfExists(body_index);
  if (frame_id.has_value()) {
    for (const auto& geometry_id : inspector.GetGeometries(
             frame_id.value(), geometry::Role::kIllustration)) {
      if (geometry_name.has_value()) {
        if (inspector.GetName(geometry_id) != *geometry_name) {
          continue;
        }
      }
      const geometry::GeometryProperties* props =
          inspector.GetProperties(geometry_id, geometry::Role::kIllustration);
      if (props == nullptr || !props->HasProperty("phong", "diffuse")) {
        DRAKE_DEMAND(rgba_r.has_value());
        DRAKE_DEMAND(rgba_g.has_value());
        DRAKE_DEMAND(rgba_b.has_value());
        DRAKE_DEMAND(rgba_a.has_value());
        geometry::IllustrationProperties new_props =
            geometry::MakePhongIllustrationProperties(
                Eigen::Vector4d(rgba_r.value(), rgba_g.value(), rgba_b.value(),
                                rgba_a.value()));
        scene_graph->AssignRole(*plant.get_source_id(), geometry_id, new_props,
                                geometry::RoleAssign::kReplace);
      } else {
        const auto old_rgba =
            props->GetProperty<geometry::Rgba>("phong", "diffuse");
        double rgba_r_val = rgba_r.has_value() ? rgba_r.value() : old_rgba.r();
        double rgba_g_val = rgba_g.has_value() ? rgba_g.value() : old_rgba.g();
        double rgba_b_val = rgba_b.has_value() ? rgba_b.value() : old_rgba.b();
        double rgba_a_val = rgba_a.has_value() ? rgba_a.value() : old_rgba.a();
        geometry::IllustrationProperties new_props =
            geometry::MakePhongIllustrationProperties(Eigen::Vector4d(
                rgba_r_val, rgba_g_val, rgba_b_val, rgba_a_val));
        scene_graph->AssignRole(*plant.get_source_id(), geometry_id, new_props,
                                geometry::RoleAssign::kReplace);
      }
    }
  }
}
}  // namespace c_iris
}  // namespace multibody
}  // namespace drake
