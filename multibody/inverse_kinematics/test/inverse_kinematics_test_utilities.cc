#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

#include <utility>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::SceneGraph;

// TODO(eric.cousineau): Replace `TwoFreeBodies*` methods with
// `two_bodies.sdf`; consider using geometry queries to get sphere radii, etc.

namespace drake {
namespace multibody {

IiwaKinematicConstraintTest::IiwaKinematicConstraintTest() {
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf");
  systems::DiagramBuilder<double> builder{};
  plant_ = builder.AddSystem<MultibodyPlant<double>>();
  plant_->RegisterAsSourceForSceneGraph(
      builder.AddSystem<SceneGraph<double>>());
  multibody::Parser parser{plant_};
  parser.AddModelFromFile(iiwa_path, "iiwa");
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("iiwa_link_0"));
  plant_->Finalize();

  drake::log()->info("plant_->num_positions = {}", plant_->num_positions());

  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  plant_context_ =
      &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

  plant_autodiff_ =
      systems::System<double>::ToAutoDiffXd(*ConstructIiwaPlant(iiwa_path, 0.));
  plant_context_autodiff_ = plant_autodiff_->CreateDefaultContext();
}

TwoFreeBodiesConstraintTest::TwoFreeBodiesConstraintTest() {
  systems::DiagramBuilder<double> builder{};
  plant_ = builder.AddSystem(ConstructTwoFreeBodiesPlant<double>());
  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  plant_context_ =
      &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
  body1_index_ = plant_->GetBodyByName("body1").body_frame().index();
  body2_index_ = plant_->GetBodyByName("body2").body_frame().index();

  plant_autodiff_ = ConstructTwoFreeBodiesPlant<AutoDiffXd>();
  plant_context_autodiff_ = plant_autodiff_->CreateDefaultContext();
}

template <typename T>
void AddTwoFreeBodiesToPlant(MultibodyPlant<T>* model) {
  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);

  model->AddRigidBody("body1", M_AAo_A);
  model->AddRigidBody("body2", M_AAo_A);
}

template <typename T>
std::unique_ptr<MultibodyPlant<T>> ConstructTwoFreeBodiesPlant() {
  auto model = std::make_unique<MultibodyPlant<T>>();
  AddTwoFreeBodiesToPlant(model.get());
  model->Finalize();
  return model;
}

std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& file_path, double time_step) {
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  Parser(plant.get()).AddModelFromFile(file_path);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
  plant->Finalize();
  return plant;
}

Eigen::Vector4d QuaternionToVectorWxyz(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

namespace {
template <typename T>
std::unique_ptr<systems::Diagram<T>> BuildTwoFreeSpheresDiagram(
    double radius1, double radius2, const Eigen::Isometry3d& X_B1S1,
    const Eigen::Isometry3d& X_B2S2, MultibodyPlant<T>** plant,
    geometry::SceneGraph<T>** scene_graph, FrameIndex* sphere1_index,
    FrameIndex* sphere2_index) {
  systems::DiagramBuilder<T> builder;
  std::tie(*plant, *scene_graph) = AddMultibodyPlantSceneGraph(&builder);
  AddTwoFreeBodiesToPlant(*plant);
  const auto& sphere1 = (*plant)->GetBodyByName("body1");
  const auto& sphere2 = (*plant)->GetBodyByName("body2");
  (*plant)->RegisterCollisionGeometry(
      sphere1, X_B1S1, geometry::Sphere(radius1), "sphere1_collision",
      multibody::multibody_plant::CoulombFriction<double>(), *scene_graph);
  (*plant)->RegisterCollisionGeometry(
      sphere2, X_B2S2, geometry::Sphere(radius2), "sphere2_collision",
      multibody::multibody_plant::CoulombFriction<double>(), *scene_graph);
  *sphere1_index = sphere1.body_frame().index();
  *sphere2_index = sphere2.body_frame().index();
  (*plant)->Finalize(*scene_graph);

  return builder.Build();
}
}  // namespace

TwoFreeSpheresTest::TwoFreeSpheresTest() {
  X_B1S1_.setIdentity();
  X_B1S1_.translation() << 0.01, 0.02, 0.03;
  X_B2S2_.setIdentity();
  X_B2S2_.translation() << 0.02, -0.01, -0.02;
  diagram_double_ = BuildTwoFreeSpheresDiagram(
      radius1_, radius2_, X_B1S1_, X_B2S2_, &plant_double_,
      &scene_graph_double_, &sphere1_index_, &sphere2_index_);
  diagram_context_double_ = diagram_double_->CreateDefaultContext();
  plant_context_double_ = &(diagram_double_->GetMutableSubsystemContext(
      *plant_double_, diagram_context_double_.get()));

  diagram_autodiff_ = BuildTwoFreeSpheresDiagram(
      radius1_, radius2_, X_B1S1_, X_B2S2_, &plant_autodiff_,
      &scene_graph_autodiff_, &sphere1_index_, &sphere2_index_);

  diagram_context_autodiff_ = diagram_autodiff_->CreateDefaultContext();
  plant_context_autodiff_ = &(diagram_autodiff_->GetMutableSubsystemContext(
      *plant_autodiff_, diagram_context_autodiff_.get()));
}

template std::unique_ptr<MultibodyPlant<double>>
ConstructTwoFreeBodiesPlant<double>();
template std::unique_ptr<MultibodyPlant<AutoDiffXd>>
ConstructTwoFreeBodiesPlant<AutoDiffXd>();

}  // namespace multibody
}  // namespace drake
