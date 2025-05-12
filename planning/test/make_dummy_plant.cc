#include "drake/planning/test/make_dummy_plant.h"

#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace planning {

using multibody::Body;
using multibody::BodyIndex;
using multibody::default_model_instance;
using multibody::Joint;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::RevoluteJoint;

// Creates plant with 7 DoFs:
// - 1 DoFs belongs to default model instance
// - 1 DoFs belongs to main model instance
// - 5 DoFs belongs to another model; however, the outboard bodies are
//   influenced by the main model instance.
std::pair<std::unique_ptr<MultibodyPlant<double>>, ModelInstanceIndex>
MakeDummyPlant() {
  const double time_step = 0.0;
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  const Eigen::Vector3d dummy_axis(1.0, 0.0, 0.0);

  const ModelInstanceIndex model_instance = plant->AddModelInstance("my_model");
  const ModelInstanceIndex other_instance =
      plant->AddModelInstance("other_model");

  auto add_joint_and_body =
      [&](ModelInstanceIndex model, std::string parent_body_name,
          std::string joint_name, std::string child_body_name) {
        const Body<double>& parent = plant->GetBodyByName(parent_body_name);
        const Body<double>& child = plant->AddRigidBody(child_body_name, model);
        const Joint<double>& joint = plant->AddJoint<RevoluteJoint>(
            joint_name, parent, {}, child, {}, dummy_axis);
        DRAKE_DEMAND(joint.model_instance() == model);
      };

  // Create joint_a and body_a which are part of the default model instance
  // (which we don't really care about).
  add_joint_and_body(default_model_instance(), "world", "joint_a", "body_a");
  // Create joint_b and body_b which are part of our custom model instance.
  add_joint_and_body(model_instance, "body_a", "joint_b", "body_b");
  // Create additional joints which are not part of our custom model instance,
  // but whose kinematics are influenced by joint_b.
  add_joint_and_body(other_instance, "body_b", "joint_c", "body_c");
  // - Serial.
  add_joint_and_body(other_instance, "body_c", "joint_d", "body_d");
  // - Branch from body_b.
  add_joint_and_body(other_instance, "body_b", "joint_e", "body_e");
  add_joint_and_body(other_instance, "body_b", "joint_f", "body_f");
  // - Serial on branch from body_e.
  add_joint_and_body(other_instance, "body_e", "joint_g", "body_g");
  // Do some minor checks.
  plant->Finalize();
  DRAKE_DEMAND(plant->num_positions() == 7);
  DRAKE_DEMAND(plant->num_velocities() == 7);
  DRAKE_DEMAND(plant->num_positions(model_instance) == 1);
  return std::make_pair(std::move(plant), model_instance);
}

std::unique_ptr<MultibodyPlant<double>> MakeDummyFloatingBodyPlant() {
  const double time_step = 0.0;
  auto plant = std::make_unique<MultibodyPlant<double>>(time_step);
  plant->AddRigidBody("floating_body", default_model_instance());
  plant->Finalize();
  DRAKE_DEMAND(plant->num_positions() == 7);
  DRAKE_DEMAND(plant->num_velocities() == 6);
  return plant;
}

}  // namespace planning
}  // namespace drake
