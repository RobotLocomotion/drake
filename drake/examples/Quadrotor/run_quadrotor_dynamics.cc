/// @file
///
/// This demo sets up a passive Quadrotor plant in a world described by the
/// warehouse model. The robot simply rests on the floor within the walls
/// of the warehouse.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using multibody::joints::kFixed;
using multibody::joints::kRollPitchYaw;
using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using parsers::sdf::AddModelInstancesFromSdfFile;
using systems::InputPortDescriptor;

namespace examples {
namespace quadrotor {
namespace {

DEFINE_double(duration, 3, "Total duration of simulation.");

template <typename T>
class Quadrotor : public systems::Diagram<T> {
 public:
  Quadrotor() {
    this->set_name("Quadrotor");

    auto tree = std::make_unique<RigidBodyTree<T>>();
    ModelInstanceIdTable model_id_table = AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow("drake/examples/Quadrotor/quadrotor.urdf"),
        kRollPitchYaw, tree.get());
    const int quadrotor_id = model_id_table.at("quadrotor");
    AddModelInstancesFromSdfFile(
        FindResourceOrThrow("drake/examples/Quadrotor/warehouse.sdf"),
        kFixed, nullptr /* weld to frame */, tree.get());
    drake::multibody::AddFlatTerrainToWorld(tree.get());

    systems::DiagramBuilder<T> builder;

    plant_ =
        builder.template AddSystem<systems::RigidBodyPlant<T>>(std::move(tree));
    plant_->set_name("plant");

    // Verifies that the quadrotor has no actuators.
    DRAKE_DEMAND(plant_->get_num_actuators() == 0);
    DRAKE_DEMAND(plant_->get_num_actuators(quadrotor_id) == 0);

    VectorX<T> hover_input(plant_->get_input_size());
    hover_input.setZero();

    systems::DrakeVisualizer* publisher =
        builder.template AddSystem<systems::DrakeVisualizer>(
            plant_->get_rigid_body_tree(), &lcm_);

    builder.Connect(plant_->get_output_port(0), publisher->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    systems::Diagram<T>::SetDefaultState(context, state);
    systems::State<T>& plant_state =
        this->GetMutableSubsystemState(*plant_, state);
    VectorX<T> x0(plant_->get_num_states());
    x0.setZero();
    /* x0 is the initial state where
     * x0(0), x0(1), x0(2) are the quadrotor's x, y, z -states
     * x0(3), x0(4), x0(5) are the quedrotor's Euler angles phi, theta, psi
     */
    x0(2) = 0.2;  // Sets arbitrary z-position. This is the initial height of
                  // the quadrotor in the world frame.
    plant_->set_state_vector(&plant_state, x0);
  }

 private:
  systems::RigidBodyPlant<T>* plant_{};
  lcm::DrakeLcm lcm_;
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  Quadrotor<double> model;
  systems::Simulator<double> simulator(model);

  simulator.Initialize();
  simulator.StepTo(FLAGS_duration);
  return 0;
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::quadrotor::do_main(argc, argv);
}
