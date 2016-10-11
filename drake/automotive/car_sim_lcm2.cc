// #include <iostream>
// #include <memory>

// #include <gtest/gtest.h>

#include "drake/automotive/automotive_common.h"
#include "drake/automotive/car_simulation.h"
#include "drake/common/drake_path.h"
// #include "drake/math/roll_pitch_yaw.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
// #include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
// #include "drake/systems/framework/primitives/adder.h"
// #include "drake/systems/framework/primitives/constant_vector_source.h"
// #include "drake/systems/framework/primitives/demultiplexer.h"
// #include "drake/systems/framework/primitives/gain.h"
// #include "drake/systems/framework/primitives/pid_controller.h"
// #include "drake/systems/framework/primitives/time_varying_polynomial_source.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_tree_lcm_publisher.h"

// TODO(liang.fok) Temporary! Remove once actual input can be provided to
// RigidBodyPlant!
#include "drake/systems/framework/primitives/constant_vector_source.h"
// Includes for the planner.
// #include "drake/systems/plants/IKoptions.h"
// #include "drake/systems/plants/RigidBodyIK.h"

// using Eigen::Vector2d;
// using Eigen::Vector3d;
// using Eigen::VectorXd;
// using Eigen::VectorXi;
// using Eigen::MatrixXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using parsers::ModelInstanceIdTable;
using parsers::sdf::AddModelInstancesFromSdfFile;

using lcm::DrakeLcm;
// using systems::Adder;
using systems::ConstantVectorSource;
using systems::Context;
// using systems::Demultiplexer;
using systems::Diagram;
using systems::DiagramBuilder;
// using systems::Gain;
// using systems::GravityCompensator;
// using systems::PidController;
using systems::RigidBodyPlant;
using systems::RigidBodyTreeLcmPublisher;
using systems::Simulator;
// using systems::TimeVaryingPolynomialSource;
using systems::plants::joints::kQuaternion;

namespace automotive {
namespace {

// A demo of a single vehicle on a flat terrain.
template<typename T>
class CarSimLcm2Demo : public systems::Diagram<T> {
 public:
  CarSimLcm2Demo() {
    this->set_name("CarSimLcmDemo");

    // Instantiates an Multibody Dynamics (MBD) model of the world.
    auto tree = make_unique<RigidBodyTree>();
    ModelInstanceIdTable vehicle_instance_id_table =
        drake::parsers::sdf::AddModelInstancesFromSdfFile(
            drake::GetDrakePath() +
            "/automotive/models/prius/prius_with_lidar.sdf",
            kQuaternion, nullptr /* weld to frame */, tree.get());

    AddFlatTerrainToWorld(tree.get());

    // // Adds the ground.
    // const double kBoxWidth = 3;
    // const double kBoxDepth = 0.2;
    // DrakeShapes::Box geom(Eigen::Vector3d(kBoxWidth, kBoxWidth, kBoxDepth));
    // Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    // // The top of the box is at z = 0.
    // T_element_to_link.translation() << 0, 0, -kBoxDepth / 2.0;

    // RigidBody& world = tree->world();
    // Eigen::Vector4d color;
    // // Ground color in RGBA format.
    // color << 0.9297, 0.7930, 0.6758, 1;
    // world.AddVisualElement(
    //     DrakeShapes::VisualElement(geom, T_element_to_link, color));
    // tree->addCollisionElement(
    //     DrakeCollision::Element(geom, T_element_to_link, &world), world,
    //     "terrain");
    // tree->updateStaticCollisionElements();

    DiagramBuilder<T> builder;

    // Instantiates a RigidBodyPlant from the MBD model of the world.
    plant_ = builder.template AddSystem<RigidBodyPlant<T>>(move(tree));
    plant_->set_contact_parameters(5000.0 /*penetration_stiffness*/,
      500 /* penetration_damping */, 10 /* friction_coefficient */);

    // TODO(liang.fok) Temporary placeholder. Remove when actual inputs can be
    // wired to RigidBodyPlant.
    VectorX<T> constant_value(plant_->get_input_size());
    constant_value.setZero();
    const_source_ = builder.template AddSystem<ConstantVectorSource<T>>(
        constant_value);

    // state_minus_target_ = builder.template AddSystem<Adder<T>>
    //     (2 /*number of inputs*/, plant_->get_num_states() /* size */);

    // // Create and add PID controller.
    // // Constants are chosen by trial and error to qualitatively match an
    // // experimental run with the same initial conditions and planner.
    // // Quantitative comparisons would require torque control and a more careful
    // // estimation of the model constants such as friction in the joints.
    // const double kp = 2.0;  // proportional constant.
    // const double ki = 0.0;  // integral constant.
    // const double kd = 1.0;  // derivative constant.
    // controller_ = builder.template AddSystem<PidController<T>>(
    //     kp, ki, kd, plant_->get_num_positions());

    // gravity_compensator_ = builder.template AddSystem<GravityCompensator<T>>(
    //     plant_->get_rigid_body_tree());
    // gcomp_minus_pid_ = builder.template AddSystem<Adder<T>>(
    //     2 /*number of inputs*/, plant_->get_num_actuators() /* size */);

    // // Split the input state into two signals one with the positions and one
    // // with the velocities.
    // // For Kuka:
    // // -  get_num_states() = 14
    // // -  get_num_positions() = 7
    // error_demux_ = builder.template AddSystem<Demultiplexer<T>>(
    //     plant_->get_num_states(), plant_->get_num_positions());
    // rbp_state_demux_ = builder.template AddSystem<Demultiplexer<T>>(
    //     plant_->get_num_states(), plant_->get_num_positions());

    // controller_inverter_ = builder.template AddSystem<Gain<T>>(
    //     -1.0, plant_->get_num_actuators());
    // error_inverter_ = builder.template AddSystem<Gain<T>>(
    //     -1.0, plant_->get_num_states());

    // // Creates a plan and wraps it into a source system.
    // poly_trajectory_ = MakePlan();
    // desired_plan_ = builder.template AddSystem<TimeVaryingPolynomialSource<T>>(
    //     *poly_trajectory_);

    // Creates and adds LCM publisher for visualization.
    viz_publisher_ = builder.template AddSystem<RigidBodyTreeLcmPublisher>(
        plant_->get_rigid_body_tree(), &lcm_);

    // // Generates an error signal for the PID controller by subtracting the
    // // desired plan state from the RigidBodyPlant's (iiwa arm) state.
    // builder.Connect(desired_plan_->get_output_port(0),
    //                 error_inverter_->get_input_port());
    // builder.Connect(error_inverter_->get_output_port(),
    //                 state_minus_target_->get_input_port(0));
    // builder.Connect(plant_->get_output_port(0),
    //                 state_minus_target_->get_input_port(1));

    // // Splits the error signal into positions and velocities components.
    // builder.Connect(state_minus_target_->get_output_port(),
    //                 error_demux_->get_input_port(0));

    // // Splits the RBP output into positions (q) and velocities (v).
    // builder.Connect(plant_->get_output_port(0),
    //                 rbp_state_demux_->get_input_port(0));

    // // Connects PID controller.
    // builder.Connect(error_demux_->get_output_port(0),
    //                 controller_->get_error_port());
    // builder.Connect(error_demux_->get_output_port(1),
    //                 controller_->get_error_derivative_port());

    // // Connects the gravity compensator to the output generalized positions.
    // builder.Connect(rbp_state_demux_->get_output_port(0),
    //                 gravity_compensator_->get_input_port(0));
    // builder.Connect(gravity_compensator_->get_output_port(0),
    //                 gcomp_minus_pid_->get_input_port(0));

    // // Adds feedback.
    // builder.Connect(controller_->get_output_port(0),
    //                 controller_inverter_->get_input_port());
    // builder.Connect(controller_inverter_->get_output_port(),
    //                 gcomp_minus_pid_->get_input_port(1));

    // builder.Connect(gcomp_minus_pid_->get_output_port(),
    //                 plant_->get_input_port(0));

    // TODO(liang.fok) Remove this once actual inputs an be provided to
    // RigidBodyPlant.
    builder.Connect(const_source_->get_output_port(),
                    plant_->get_input_port(0));

    // Connects to publisher for visualization.
    builder.Connect(plant_->get_output_port(0),
                    viz_publisher_->get_input_port(0));

    builder.ExportOutput(plant_->get_output_port(0));
    builder.BuildInto(this);
  }

  // const RigidBodyPlant<T>& get_kuka_plant() const { return *plant_; }

  void SetDefaultState(Context<T>* context) const {
    // Context<T>* controller_context =
    //     this->GetMutableSubsystemContext(context, controller_);
    // controller_->set_integral_value(controller_context, VectorX<T>::Zero(7));

    Context<T>* plant_context =
        this->GetMutableSubsystemContext(context, plant_);
    plant_->SetZeroConfiguration(plant_context);
  }

 private:
  RigidBodyPlant<T>* plant_;
  // PidController<T>* controller_;
  // Demultiplexer<T>* error_demux_;
  // Demultiplexer<T>* rbp_state_demux_;
  // Gain<T>* controller_inverter_;
  // Gain<T>* error_inverter_;
  // GravityCompensator<T>* gravity_compensator_;
  // Adder<T>* state_minus_target_;
  // Adder<T>* gcomp_minus_pid_;
  // TimeVaryingPolynomialSource<T>* desired_plan_;
  // std::unique_ptr<PiecewisePolynomial<T>> poly_trajectory_;
  RigidBodyTreeLcmPublisher* viz_publisher_;
  DrakeLcm lcm_;

  ConstantVectorSource<T>* const_source_;
};

int main() {
  CarSimLcm2Demo<double> model;
  Simulator<double> simulator(model);

  // Zeroes the state and initializes controller state.
  model.SetDefaultState(simulator.get_mutable_context());

  // VectorX<double> desired_state = VectorX<double>::Zero(14);
  // model.get_kuka_plant().set_state_vector(
  //     simulator.get_mutable_context(), desired_state);

  simulator.Initialize();

  // Simulate for 20 seconds.
  simulator.StepTo(20.0);

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::automotive::main();
}
