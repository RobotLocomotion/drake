#include "drake/multibody/plant/test_utilities/multibody_plant_remodeling.h"

#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"

namespace drake {
namespace multibody {

using Eigen::Vector3d;
using systems::DiagramBuilder;
using systems::Simulator;

template <template <typename> class JointType>
void MultibodyPlantRemodeling::BuildModel() {
  builder_ = std::make_unique<DiagramBuilder<double>>();
  MultibodyPlantConfig config = {.time_step = kTimeStep,
                                 .discrete_contact_approximation = "sap"};
  std::tie(plant_, scene_graph_) = AddMultibodyPlant(config, builder_.get());

  plant_->AddRigidBody("body0", SpatialInertia<double>::MakeUnitary());
  plant_->AddRigidBody("body1", SpatialInertia<double>::MakeUnitary());
  plant_->AddRigidBody("body2", SpatialInertia<double>::MakeUnitary());
  plant_->template AddJoint<JointType>("joint0", plant_->world_body(), {},
                                       plant_->GetBodyByName("body0"), {},
                                       Vector3d::UnitZ());
  plant_->template AddJoint<JointType>("joint1", plant_->GetBodyByName("body0"),
                                       {}, plant_->GetBodyByName("body1"), {},
                                       Vector3d::UnitZ());
  plant_->template AddJoint<JointType>("joint2", plant_->GetBodyByName("body1"),
                                       {}, plant_->GetBodyByName("body2"), {},
                                       Vector3d::UnitZ());
  plant_->AddJointActuator(
      "actuator0", plant_->template GetJointByName<JointType>("joint0"), 1);
  plant_->AddJointActuator(
      "actuator1", plant_->template GetJointByName<JointType>("joint1"), 1);
  plant_->AddJointActuator(
      "actuator2", plant_->template GetJointByName<JointType>("joint2"), 1);
}

void MultibodyPlantRemodeling::DoRemoval(bool remove_actuator,
                                         bool remove_joint) {
  if (remove_actuator) {
    plant_->RemoveJointActuator(plant_->GetJointActuatorByName("actuator1"));
  }

  if (remove_joint) {
    plant_->RemoveJoint(plant_->GetJointByName("joint1"));
  }
}

void MultibodyPlantRemodeling::FinalizeAndBuild() {
  plant_->Finalize();

  diagram_ = builder_->Build();
  builder_.reset();

  simulator_ = std::make_unique<Simulator<double>>(*diagram_);
  plant_context_ =
      &plant_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
}

template void MultibodyPlantRemodeling::BuildModel<RevoluteJoint>();
template void MultibodyPlantRemodeling::BuildModel<PrismaticJoint>();

}  // namespace multibody
}  // namespace drake
