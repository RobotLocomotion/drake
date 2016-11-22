#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"

#include <memory>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_tree.h";
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

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
using parsers::urdf::AddModelInstanceFromUrdfString;

namespace examples {
namespace toyota_hsrb {

unique_ptr<systems::Diagram<double>> CreateHsrbPlantDiagram(
    const string& hsrb_urdf_string, double penetration_stiffness,
    double penetration_damping, double friction_coefficient, lcm::DrakeLcm* lcm,
    RigidBodyPlant<double>** plant) {
  DiagramBuilder<double> builder;
  RigidBodyPlant<double>* plant_ptr{nullptr};

  // The following curly brace prevents leakage of variable `tree` outside of
  // this scope. Ownership of `tree` is passed to the RigidBodyPlant, and we
  // want to avoid downstream code from trying to access a `tree` that is a
  // nullptr since that'll result in a seg fault.
  {
    // Instantiates a model of the world.
    auto tree = make_unique<RigidBodyTreed>();
    AddModelInstanceFromUrdfString(hsrb_urdf_string, "." /* root directory */,
                                   multibody::joints::kQuaternion,
                                   nullptr /* weld to frame */, tree.get());
    AddFlatTerrainToWorld(tree.get());

    // Instantiates a RigidBodyPlant containing the tree.
    plant_ptr = builder.template AddSystem<RigidBodyPlant<double>>(move(tree));
    DRAKE_DEMAND(plant_ptr != nullptr);

    // Sets the name and contact parameters.
    plant_ptr->set_contact_parameters(
        penetration_stiffness, penetration_damping, friction_coefficient);
  }

  const RigidBodyTreed& tree = plant_ptr->get_rigid_body_tree();

  // Instantiates a system for visualizing the model.
  auto visualizer = builder.AddSystem<DrakeVisualizer>(tree, lcm);
  builder.Connect(plant_ptr->get_output_port(0), visualizer->get_input_port(0));

  builder.ExportInput(plant_ptr->get_input_port(0));
  builder.ExportOutput(plant_ptr->get_output_port(0));

  *plant = plant_ptr;
  return builder.Build();
}

std::unique_ptr<Diagram<double>> CreateHsrbDemo1Diagram(
    const RigidBodyPlant<double>& plant,
    std::unique_ptr<Diagram<double>> plant_diagram) {
  DiagramBuilder<double> builder;
  Diagram<double>* plant_diagram_ptr =
      builder.AddSystem(std::move(plant_diagram));
  VectorX<double> constant_vector(plant.get_input_port(0).get_size());
  constant_vector.setZero();
  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(constant_vector);
  builder.Cascade(*constant_zero_source, *plant_diagram_ptr);
  builder.ExportOutput(plant_diagram_ptr->get_output_port(0));
  return builder.Build();
}

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
