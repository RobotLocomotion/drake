#include <iostream>
#include <limits>

#include <gflags/gflags.h>
#include "drake/common/text_logging.h"
#include <drake/systems/plants/RigidBodySystem.h>
#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace quadrotor {
namespace {

template<typename T>
class Quadrotor : public systems::Diagram<T> {
 public:
  Quadrotor() {
    this->set_name("Quadrotor");

    auto tree = std::make_unique<RigidBodyTree>();
    parsers::ModelInstanceIdTable vehicle_instance_id_table =
        drake::parsers::urdf::AddModelInstanceFromUrdfFile(
            drake::GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
            systems::plants::joints::kRollPitchYaw,
            nullptr, tree.get());




    systems::DiagramBuilder<T> builder;

    plant_ = builder.template AddSystem<systems::RigidBodyPlant<T>>(
        std::move(tree));

    Eigen::VectorXd hover_input = Eigen::VectorXd::Constant(4,0.5*9.81/4.0);
    std::cout << hover_input <<"\n";
    source_ = builder.template AddSystem<systems::ConstantVectorSource<T>>(
        hover_input);

    publisher_ = builder.template AddSystem<systems::DrakeVisualizer>(
        plant_->get_rigid_body_tree(), &lcm_);

    builder.Connect(source_->get_output_port(), plant_->get_input_port(0));
    builder.Connect(plant_->get_output_port(0), publisher_->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetDefaultState(systems::Context<T>* context) const {
    systems::Context<T>* plant_context =
        this->GetMutableSubsystemContext(context, plant_);
    plant_->SetZeroConfiguration(plant_context);
  }

  const systems::RigidBodyPlant<T>& get_rigid_body_plant() {
    return *plant_;
  }

 private:
  systems::RigidBodyPlant<T>* plant_;
  systems::DrakeVisualizer* publisher_;
  lcm::DrakeLcm lcm_;
  systems::ConstantVectorSource<T>* source_;
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  Quadrotor<double> model;
  systems::Simulator<double> simulator(model);

  model.SetDefaultState(simulator.get_mutable_context());

  simulator.Initialize();
  simulator.StepTo(0.5);
  return 0;
}

}
}
}
}

int main(int argc, char* argv[]) {
  return drake::examples::quadrotor::do_main(argc, argv);
}