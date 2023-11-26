#include "drake/multibody/plant/test_utilities/multibody_plant_remodeling.h"

#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {

using Eigen::Vector3d;
using systems::DiagramBuilder;
using systems::Simulator;

void MultibodyPlantRemodeling::SetUp() {
  builder_ = std::make_unique<DiagramBuilder<double>>();
  MultibodyPlantConfig config = {.time_step = kTimeStep,
                                 .discrete_contact_approximation = "sap"};
  std::tie(plant_, scene_graph_) = AddMultibodyPlant(config, builder_.get());

  plant_->AddRigidBody("body0", SpatialInertia<double>::MakeUnitary());
  plant_->AddRigidBody("body1", SpatialInertia<double>::MakeUnitary());
  plant_->AddRigidBody("body2", SpatialInertia<double>::MakeUnitary());
  plant_->AddJoint<RevoluteJoint>("joint0", plant_->world_body(), {},
                                  plant_->GetBodyByName("body0"), {},
                                  Vector3d::UnitZ());
  plant_->AddJoint<RevoluteJoint>("joint1", plant_->GetBodyByName("body0"), {},
                                  plant_->GetBodyByName("body1"), {},
                                  Vector3d::UnitZ());
  plant_->AddJoint<RevoluteJoint>("joint2", plant_->GetBodyByName("body1"), {},
                                  plant_->GetBodyByName("body2"), {},
                                  Vector3d::UnitZ());
  plant_->AddJointActuator("actuator0",
                           plant_->GetJointByName<RevoluteJoint>("joint0"), 1);
  plant_->AddJointActuator("actuator1",
                           plant_->GetJointByName<RevoluteJoint>("joint1"), 1);
  plant_->AddJointActuator("actuator2",
                           plant_->GetJointByName<RevoluteJoint>("joint2"), 1);

  // Remove an actuator in the middle of the array.
  plant_->RemoveJointActuator(plant_->GetJointActuatorByName("actuator1"));
}

void MultibodyPlantRemodeling::FinalizeAndBuild() {
  plant_->Finalize();

  diagram_ = builder_->Build();
  builder_.reset();

  simulator_ = std::make_unique<Simulator<double>>(*diagram_);
  plant_context_ =
      &plant_->GetMyMutableContextFromRoot(&simulator_->get_mutable_context());
}
}  // namespace multibody
}  // namespace drake
