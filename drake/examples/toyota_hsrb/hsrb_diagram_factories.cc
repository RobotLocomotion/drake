#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"

#include <memory>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/examples/examples_package_map.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/multibody/parsers/parser_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::make_unique;
using std::unique_ptr;
using std::string;

namespace drake {

using multibody::AddFlatTerrainToWorld;
using systems::ConstantVectorSource;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using parsers::urdf::AddModelInstanceFromUrdfStringSearchingInRosPackages;

namespace examples {
namespace toyota_hsrb {

unique_ptr<systems::Diagram<double>> BuildPlantAndVisualizerDiagram(
    const string& urdf_string, double penetration_stiffness,
    double penetration_damping, double friction_coefficient,
    lcm::DrakeLcmInterface* lcm, RigidBodyPlant<double>** plant) {
  DiagramBuilder<double> builder;
  RigidBodyPlant<double>* plant_ptr{nullptr};

  // The following curly brace defines a namespace scope for `tree_ptr`.
  // Ownership of `tree_ptr` is passed to the `RigidBodyPlant`; the scope
  // prevents downstream code from accessing `tree_ptr` since that'll result in
  // a segmentation fault.
  {
    // Instantiates a model of the world.
    auto tree_ptr = make_unique<RigidBodyTreed>();
    parsers::PackageMap package_map;
    AddExamplePackages(&package_map);
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        urdf_string, package_map, "." /* root directory */,
        multibody::joints::kQuaternion, nullptr /* weld to frame */,
        tree_ptr.get());
    AddFlatTerrainToWorld(tree_ptr.get());

    // Instantiates a RigidBodyPlant containing the tree.
    plant_ptr = builder.template AddSystem<RigidBodyPlant<double>>(
        move(tree_ptr));
    DRAKE_DEMAND(plant_ptr != nullptr);

    // Sets the name and contact parameters.
    plant_ptr->set_contact_parameters(
        penetration_stiffness, penetration_damping, friction_coefficient);
  }

  const RigidBodyTreed& tree = plant_ptr->get_rigid_body_tree();

  // Instantiates a system for visualizing the model.
  const auto visualizer = builder.AddSystem<DrakeVisualizer>(tree, lcm);
  builder.Connect(plant_ptr->get_output_port(0), visualizer->get_input_port(0));

  // Exports all of the RigidBodyPlant's input and output ports.
  for (int i = 0; i < plant_ptr->get_num_input_ports(); ++i) {
    builder.ExportInput(plant_ptr->get_input_port(i));
  }

  for (int i = 0; i < plant_ptr->get_num_output_ports(); ++i) {
    builder.ExportOutput(plant_ptr->get_output_port(i));
  }

  *plant = plant_ptr;
  return builder.Build();
}

std::unique_ptr<Diagram<double>> BuildConstantSourceToPlantDiagram(
    std::unique_ptr<Diagram<double>> plant_diagram) {
  DiagramBuilder<double> builder;
  Diagram<double>* plant_diagram_ptr =
      builder.AddSystem(std::move(plant_diagram));
  VectorX<double> constant_vector(
      plant_diagram_ptr->get_input_port(0).get_size());
  constant_vector.setZero();
  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(constant_vector);
  builder.Connect(constant_zero_source->get_output_port(),
                  plant_diagram_ptr->get_input_port(0));

  // Exports all of the RigidBodyPlant's output ports.
  for (int i = 0; i < plant_diagram_ptr->get_num_output_ports(); ++i) {
    builder.ExportOutput(plant_diagram_ptr->get_output_port(i));
  }

  return builder.Build();
}

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
