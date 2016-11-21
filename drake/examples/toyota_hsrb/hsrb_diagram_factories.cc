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

namespace drake {

using multibody::AddFlatTerrainToWorld;
using std::make_unique;
using systems::ConstantVectorSource;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using parsers::urdf::AddModelInstanceFromUrdfString;

namespace examples {
namespace toyota_hsrb {

// const char* const kPlantName = "RigidBodyPlant";

// const RigidBodyPlant<double>* GetRigidBodyPlant(
//     const Diagram<double>* diagram) {
//   std::vector<const systems::System<double>*> systems =
//       diagram->GetSystems();
//   const RigidBodyPlant<double>* plant{nullptr};
//   for (unsigned int i = 0; i < systems.size(); ++i) {
//     if (systems[i]->get_name() == kPlantName) {
//       plant = dynamic_cast<const RigidBodyPlant<double>*>(systems[i]);
//     }
//   }
//   DRAKE_DEMAND(plant != nullptr);
//   return plant;
// }

std::tuple<std::unique_ptr<Diagram<double>, RigidBodyPlant*>>
CreateHsrPlantDiagram(const std::string& urdf_string,
    double penetration_stiffness,
    double penetration_damping,
    double friction_coefficient,
    lcm::DrakeLcm* lcm) {
  DiagramBuilder<double> builder;
  RigidBodyPlant<double>* plant{nullptr};

  // The following curly brace prevents leakage of variable `tree` outside of
  // this scope. Ownership of `tree` is passed to the RigidBodyPlant, and we
  // want to avoid downstream code from trying to access a `tree` that is a
  // nullptr since that'll result in a seg fault.
  {
    // Instantiates a model of the world.
    auto tree = make_unique<RigidBodyTreed>();
    AddModelInstanceFromUrdfString(
        urdf_string,
        "." /* root directory */,
        multibody::joints::kQuaternion,
        nullptr /* weld to frame */,
        tree.get());
    AddFlatTerrainToWorld(tree.get());

    // Instantiates a RigidBodyPlant containing the tree.
    plant = builder.template AddSystem<RigidBodyPlant<double>>(move(tree));

    // Sets the name and contact parameters.
    plant->set_contact_parameters(
        penetration_stiffness, penetration_damping, friction_coefficient);
  }

  DRAKE_ASSERT(plant != nullptr);

  const RigidBodyTreed& tree = plant->get_rigid_body_tree();

  // Instantiates a system for visualizing the model.
  auto visualizer = builder.AddSystem<DrakeVisualizer>(tree, lcm);
  builder.Connect(plant->get_output_port(0),
                  visualizer->get_input_port(0));

  builder.ExportInput(plant.get_input_port(0))
  builder.ExportOutput(plant->get_output_port(0));

  return std::make_tuple(builder.Build(), plant);
}

std::tuple<std::unique_ptr<Diagram<double>, RigidBodyPlant*>>
CreateDemo1Diagram(const Diagram<double>& diagram) {
  DiagramBuilder<double> builder;
  RigidBodyPlant<double>* plant{nullptr};

  // The following curly brace prevents leakage of variable `tree` outside of
  // this scope. Ownership of `tree` is passed to the RigidBodyPlant, and we
  // want to avoid downstream code from trying to access a `tree` that is a
  // nullptr since that'll result in a seg fault.
  {
    // Instantiates a model of the world.
    auto tree = make_unique<RigidBodyTreed>();
    AddModelInstanceFromUrdfString(
        urdf_string,
        "." /* root directory */,
        multibody::joints::kQuaternion,
        nullptr /* weld to frame */,
        tree.get());
    AddFlatTerrainToWorld(tree.get());

    // Instantiates a RigidBodyPlant containing the tree.
    plant = builder.template AddSystem<RigidBodyPlant<double>>(move(tree));

    // Sets the name and contact parameters.
    plant->set_contact_parameters(
        penetration_stiffness, penetration_damping, friction_coefficient);
  }

  DRAKE_ASSERT(plant != nullptr);

  const RigidBodyTreed& tree = plant->get_rigid_body_tree();

  // Instantiates a system for visualizing the model.
  auto visualizer = builder.AddSystem<DrakeVisualizer>(tree, lcm);
  builder.Connect(plant->get_output_port(0),
                  visualizer->get_input_port(0));

  // Instantiates a constant vector source for issuing zero-torque commands to
  // the RigidBodyPlant.
  VectorX<double> constant_vector(plant->get_input_port(0).get_size());
  constant_vector.setZero();
  auto constant_zero_source =
      builder.template AddSystem<ConstantVectorSource<double>>(constant_vector);
  builder.Cascade(*constant_zero_source, *plant);

  builder.ExportOutput(plant->get_output_port(0));

  return builder.Build();
}

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
