#pragma once

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
// TODO(naveenoid): These methods must be merged with those in
// /examples/toyota_hsrb/hsrb_diagram_factories.h and moved to a common
// library.

/// Builds a diagram composed of a `RigidBodyPlant` and `DrakeVisualizer` from
/// a `RigidBodyTree` and returns it. The plant's output port zero is connected
/// to
/// the systems::DrakeVisualizer's input port zero. The returned
/// systems::Diagram has the same input and output ports as the plant.
std::unique_ptr<systems::Diagram<double>> BuildPlantAndVisualizerDiagram(
    std::unique_ptr<RigidBodyTree<double>> rigid_body_tree,
    double penetration_stiffness, double penetration_damping,
    double friction_coefficient, lcm::DrakeLcmInterface* lcm,
    systems::RigidBodyPlant<double>** plant);

/// Builds and returns a systems::Diagram consisting of a
/// systems::ConstantVectorSource connected to input port zero of
/// @p plant_diagram. The returned systems::Diagram has no input ports and the
/// same output ports as @p plant_diagram. Typically, @p plant_diagram is built
/// using BuildPlantAndVisualizerDiagram().
std::unique_ptr<systems::Diagram<double>> BuildConstantSourceToPlantDiagram(
    std::unique_ptr<systems::Diagram<double>> plant_visualizer_diagram);

/// Sets the zero configuration in a `RigidBodyPlant` underlying an assembled
/// diagram by accessing the context from the `Simulator`. Assumes that the
/// diagram
/// was built by one of the factory methods above.
void SetZeroConfiguration(systems::Simulator<double>* simulator,
                          const systems::Diagram<double>* diagram);
