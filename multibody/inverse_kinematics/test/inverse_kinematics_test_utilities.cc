#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::SceneGraph;

namespace drake {
namespace multibody {
namespace {
template <typename T>
std::unique_ptr<T> ConstructTwoFreeBodiesHelper() {
  auto model = std::make_unique<T>();

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
}  // namespace

namespace internal {
IiwaKinematicConstraintTest::IiwaKinematicConstraintTest()
    : iiwa_autodiff_(benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel<AutoDiffXd>(
          true /* finalized model. */)),
      iiwa_double_(benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel<double>(
          true /* finalized model. */)),
      context_autodiff_(iiwa_autodiff_.CreateDefaultContext()),
      context_double_(iiwa_double_.CreateDefaultContext()) {
  systems::DiagramBuilder<double> builder{};
  plant_ = builder.AddSystem<MultibodyPlant<double>>();
  plant_->RegisterAsSourceForSceneGraph(
      builder.AddSystem<SceneGraph<double>>());
  multibody::Parser parser{plant_};
  parser.AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/"
                          "iiwa14_no_collision.sdf"),
      "iiwa");
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("iiwa_link_0"));
  plant_->Finalize();

  drake::log()->info("plant_->num_positions = {}", plant_->num_positions());

  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  plant_context_ =
      &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
}

TwoFreeBodiesConstraintTest::TwoFreeBodiesConstraintTest()
    : two_bodies_autodiff_(ConstructTwoFreeBodies<AutoDiffXd>()),
      two_bodies_double_(ConstructTwoFreeBodies<double>()),
      body1_index_(two_bodies_autodiff_.tree()
                       .GetBodyByName("body1")
                       .body_frame()
                       .index()),
      body2_index_(two_bodies_autodiff_.tree()
                       .GetBodyByName("body2")
                       .body_frame()
                       .index()),
      context_autodiff_(two_bodies_autodiff_.CreateDefaultContext()),
      context_double_(two_bodies_double_.CreateDefaultContext()) {
  systems::DiagramBuilder<double> builder{};
  plant_ = builder.AddSystem(ConstructTwoFreeBodiesPlant<double>());
  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  plant_context_ =
      &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
}
}  // namespace internal

template <typename T>
std::unique_ptr<MultibodyTree<T>> ConstructTwoFreeBodies() {
  return ConstructTwoFreeBodiesHelper<MultibodyTree<T>>();
}

template <typename T>
std::unique_ptr<MultibodyPlant<T>> ConstructTwoFreeBodiesPlant() {
  return ConstructTwoFreeBodiesHelper<MultibodyPlant<T>>();
}

std::unique_ptr<MultibodyPlant<double>> ConstructIiwaPlant(
    const std::string& iiwa_sdf_name, double time_step) {
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/" + iiwa_sdf_name;
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  Parser(plant.get()).AddModelFromFile(FindResourceOrThrow(file_path));
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0"));
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
template std::unique_ptr<MultibodyPlant<double>>
ConstructTwoFreeBodiesPlant<double>();
template std::unique_ptr<MultibodyPlant<AutoDiffXd>>
ConstructTwoFreeBodiesPlant<AutoDiffXd>();
}  // namespace multibody
}  // namespace drake
