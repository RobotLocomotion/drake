#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {
template <typename T>
std::unique_ptr<T> ConstructTwoFreeBodiesHelper(const std::string& body1_name,
                                                const std::string& body2_name) {
  auto model = std::make_unique<T>();

  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);

  model->AddRigidBody(body1_name, M_AAo_A);
  model->AddRigidBody(body2_name, M_AAo_A);

  return model;
}
}  // namespace
template <typename T>
std::unique_ptr<MultibodyTree<T>> ConstructTwoFreeBodies() {
  auto model = ConstructTwoFreeBodiesHelper<MultibodyTree<T>>("body1", "body2");
  model->Finalize();
  return model;
}

template <typename T>
std::unique_ptr<multibody_plant::MultibodyPlant<T>>
ConstructTwoFreeBodiesPlant() {
  auto model = ConstructTwoFreeBodiesHelper<multibody_plant::MultibodyPlant<T>>(
      "body1", "body2");
  model->Finalize();
  return model;
}

namespace internal {
TwoFreeSpheresConstraintTest::TwoFreeSpheresConstraintTest() {
  using T = AutoDiffXd;
  systems::DiagramBuilder<T> builder;
  scene_graph_ = builder.AddSystem<geometry::SceneGraph<T>>();
  auto model = ConstructTwoFreeBodiesHelper<multibody_plant::MultibodyPlant<T>>(
      "sphere1", "sphere2");
  two_spheres_plant_ = builder.AddSystem(std::move(model));
  two_spheres_plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  const auto& sphere1 = two_spheres_plant_->GetBodyByName("sphere1");
  const auto& sphere2 = two_spheres_plant_->GetBodyByName("sphere2");
  two_spheres_plant_->RegisterCollisionGeometry(
      sphere1, Eigen::Isometry3d::Identity(), geometry::Sphere(radius1_),
      "sphere1_collision",
      multibody::multibody_plant::CoulombFriction<double>(), scene_graph_);
  two_spheres_plant_->RegisterCollisionGeometry(
      sphere2, Eigen::Isometry3d::Identity(), geometry::Sphere(radius2_),
      "sphere2_collision",
      multibody::multibody_plant::CoulombFriction<double>(), scene_graph_);
  sphere1_index_ = sphere1.body_frame().index();
  sphere2_index_ = sphere2.body_frame().index();
  two_spheres_plant_->Finalize(scene_graph_);
  builder.Connect(
      two_spheres_plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(*two_spheres_plant_->get_source_id()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  two_spheres_plant_->get_geometry_query_input_port());
  diagram_ = builder.Build();
  diagram_context_autodiff_ = diagram_->CreateDefaultContext();
  plant_context_autodiff_ = &(diagram_->GetMutableSubsystemContext(
      *two_spheres_plant_, diagram_context_autodiff_.get()));
}
}  // namespace internal

std::unique_ptr<multibody_plant::MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& iiwa_sdf_name, double time_step) {
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/" + iiwa_sdf_name;
  auto plant =
      std::make_unique<multibody_plant::MultibodyPlant<double>>(time_step);
  parsing::AddModelFromSdfFile(FindResourceOrThrow(file_path), plant.get());
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
  plant->Finalize();
  return plant;
}

Eigen::Vector4d QuaternionToVectorWxyz(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

template std::unique_ptr<MultibodyTree<double>>
ConstructTwoFreeBodies<double>();
template std::unique_ptr<MultibodyTree<AutoDiffXd>>
ConstructTwoFreeBodies<AutoDiffXd>();
template std::unique_ptr<multibody_plant::MultibodyPlant<double>>
ConstructTwoFreeBodiesPlant<double>();
template std::unique_ptr<multibody_plant::MultibodyPlant<AutoDiffXd>>
ConstructTwoFreeBodiesPlant<AutoDiffXd>();
}  // namespace multibody
}  // namespace drake
