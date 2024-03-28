#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {

// The purpose of this fixture is to unit test various MultibodyPlant remodeling
// features. It sets up a generic multibody plant with arbitrary bodies, joints,
// actuators etc. and then exercises remodeling functionality (removal, etc.)
// Users can add/remove additional elements to `plant_` and then call
// FinalizeAndBuild() to observe post Finalize() behavior.
// TODO(joemasterjohn) Update this class and documentation for removal of other
// multibody elements when support is added.
// TODO(joemasterjohn) Consider adding a test param or user functor to specify
// the model/remodeling to be done.
class MultibodyPlantRemodeling : public ::testing::Test {
 public:
  // This fixture sets up a plant with a serial chain of 3 bodies connected by
  // revolute joints. An actuator is added to each joint. Then the actuator
  // controlling `joint1` is removed.
  void SetUp() override;

  // Finalize the plant, build the diagram and initialize `simulator_`
  void FinalizeAndBuild();

 protected:
  // These members are only valid after SetUp().
  // `builder_` is invalid after calling FinalizeAndBuild().
  std::unique_ptr<systems::DiagramBuilder<double>> builder_;
  MultibodyPlant<double>* plant_{nullptr};
  geometry::SceneGraph<double>* scene_graph_{nullptr};

  // These members are only valid after FinalizeAndBuild().
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<systems::Simulator<double>> simulator_;
  systems::Context<double>* plant_context_{nullptr};

  const double kTimeStep{0.1};  // Discrete time step of plant_
};
}  // namespace multibody
}  // namespace drake
