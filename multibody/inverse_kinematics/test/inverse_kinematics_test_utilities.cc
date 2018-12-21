#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::SceneGraph;

namespace drake {
namespace multibody {

namespace internal {
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

  plant_autodiff_ = systems::System<double>::ToAutoDiffXd(
      *ConstructIiwaPlant(iiwa_path, 0.));
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

}  // namespace internal

template <typename T>
std::unique_ptr<MultibodyPlant<T>> ConstructTwoFreeBodiesPlant() {
  auto model = std::make_unique<MultibodyPlant<T>>();
  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);

  model->AddRigidBody("body1", M_AAo_A);
  model->AddRigidBody("body2", M_AAo_A);

  model->Finalize();
  return model;
}

std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& file_path, double time_step) {
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  Parser(plant.get()).AddModelFromFile(file_path);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0"));
  plant->Finalize();
  return plant;
}

Eigen::Vector4d QuaternionToVectorWxyz(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

template std::unique_ptr<MultibodyPlant<double>>
ConstructTwoFreeBodiesPlant<double>();
template std::unique_ptr<MultibodyPlant<AutoDiffXd>>
ConstructTwoFreeBodiesPlant<AutoDiffXd>();
}  // namespace multibody
}  // namespace drake
