#pragma once

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {

using Eigen::Vector3d;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;

// The purpose of this fixture is to unit test various MultibodyPlant remodeling
// features. It sets up a generic multibody plant with arbitrary bodies, joints,
// actuators etc. and the exercises remodeling functionaltiy (removal, etc.)
// Users can add/remove additional elements to `plant_` and then call
// FinalizeAndBuild() to observe pose Finalize() behavior.
// TODO(joemasterjohn) Update this class and documentation for removal of other
// multibody elements when support is added.
// TODO(joemasterjohn) Consider adding a test param or user functor to specify
// the model/remodeling to be done.
class MultibodyPlantRemodeling : public ::testing::Test {
 public:
  // This fixture sets up a plant with a serial chain of 3 bodies connected by
  // revolute joints. An actuator is added to each joint. THen the actuator
  // controlling `joint1` is removed.
  void SetUp() override {
    builder_ = std::make_unique<DiagramBuilder<double>>();
    MultibodyPlantConfig config = {.time_step = kTimeStep_,
                                   .discrete_contact_approximation = "sap"};
    std::tie(plant_, scene_graph_) = AddMultibodyPlant(config, builder_.get());

    plant_->AddRigidBody("body0", SpatialInertia<double>::MakeUnitary());
    plant_->AddRigidBody("body1", SpatialInertia<double>::MakeUnitary());
    plant_->AddRigidBody("body2", SpatialInertia<double>::MakeUnitary());
    plant_->AddJoint<RevoluteJoint>("joint0", plant_->world_body(), {},
                                    plant_->GetBodyByName("body0"), {},
                                    Vector3d::UnitZ());
    plant_->AddJoint<RevoluteJoint>("joint1", plant_->GetBodyByName("body0"),
                                    {}, plant_->GetBodyByName("body1"), {},
                                    Vector3d::UnitZ());
    plant_->AddJoint<RevoluteJoint>("joint2", plant_->GetBodyByName("body1"),
                                    {}, plant_->GetBodyByName("body2"), {},
                                    Vector3d::UnitZ());
    plant_->AddJointActuator(
        "actuator0", plant_->GetJointByName<RevoluteJoint>("joint0"), 1);
    plant_->AddJointActuator(
        "actuator1", plant_->GetJointByName<RevoluteJoint>("joint1"), 1);
    plant_->AddJointActuator(
        "actuator2", plant_->GetJointByName<RevoluteJoint>("joint2"), 1);

    // Remove an actuator in the middle of the array.
    plant_->RemoveJointActuator(plant_->GetJointActuatorByName("actuator1"));
  }

  void FinalizeAndBuild() {
    plant_->Finalize();

    diagram_ = builder_->Build();

    simulator_ = std::make_unique<Simulator<double>>(*diagram_);
    plant_context_ = &plant_->GetMyMutableContextFromRoot(
        &simulator_->get_mutable_context());

    simulator_->Initialize();
  }

 protected:
  std::unique_ptr<DiagramBuilder<double>> builder_;
  std::unique_ptr<Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  geometry::SceneGraph<double>* scene_graph_{nullptr};
  Context<double>* plant_context_{nullptr};
  std::unique_ptr<Simulator<double>> simulator_;

  const double kTimeStep_{0.1};  // Discrete time step of plant_
};
}  // namespace multibody
}  // namespace drake
