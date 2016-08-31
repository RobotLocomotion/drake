#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/demultiplexer.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/gravity_compensator.h"
#include "drake/systems/framework/primitives/pid_controller2.h"
#include "drake/systems/framework/primitives/time_varying_polynomial_source.h"
#include "drake/systems/bot_visualizer_system.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_system/rigid_body_plant.h"

// Includes for the planner part. Move somewher else
#include "drake/common/polynomial.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_system {
namespace test {
namespace {

const double deg_to_rad = M_PI / 180.0;

template<typename T>
class KukaDemo : public Diagram<T> {
 public:
  // Pass through to SpringMassSystem, except add sample rate in samples/s.
  KukaDemo() {
    this->set_name("KukaDemo");

    // Instantiates an MBD model of the world.
    auto mbd_world = make_unique<RigidBodyTree>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14_collision_free.urdf",
        DrakeJoint::FIXED, nullptr /* weld to frame */, mbd_world.get());

    // Adds the ground.
    double kBoxWidth = 3;
    double kBoxDepth = 0.2;
    DrakeShapes::Box geom(Eigen::Vector3d(kBoxWidth, kBoxWidth, kBoxDepth));
    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    // top of the box is at z = 0.
    T_element_to_link.translation() << 0, 0, -kBoxDepth / 2.0;

    RigidBody& world = mbd_world->world();
    Eigen::Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;
    world.AddVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    mbd_world->addCollisionElement(
        RigidBodyCollisionElement(geom, T_element_to_link, &world), world,
        "terrain");
    mbd_world->updateStaticCollisionElements();

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    plant_ = make_unique<RigidBodyPlant<T>>(move(mbd_world));

    VectorX<double> desired_state = VectorX<double>::Zero(14);
    desired_state[0] =   0.0 * deg_to_rad;  // base.
    desired_state[1] =  45.0 * deg_to_rad;  // first elbow.
    desired_state[2] =   0.0 * deg_to_rad;  // axial rotation.
    desired_state[3] = -90.0 * deg_to_rad;  // second elbow.
    desired_state[4] =  90.0 * deg_to_rad;  // axial rotation.
    desired_state[5] =   0.0 * deg_to_rad;  // final wrist
    desired_state[6] =   0.0 * deg_to_rad;  // end effector rotation.

    target_state_ = make_unique<ConstantVectorSource<T>>(desired_state);
    state_minus_target_ = make_unique<Adder<T>>(2 /*number of inputs*/,
        plant_->get_num_states() /* size */);

    const double numerical_stiffness = 3000;
    plant_->penetration_stiffness_ = 10*numerical_stiffness;

    // Rough estimation of controller constants.
    //const double arm_mass = plant_->get_multibody_world().getMass() / 7.0;
    // Closed loop frequency.
    //const double wol = sqrt(numerical_stiffness / arm_mass);

    // Naveen's constants: Kp=10, Ki=0, Kd=0.3
    const double kp = 2.0; //numerical_stiffness/100.0;
    const double ki = 0.0;
    // Closed loop system frequency.
    //const double wcl = sqrt(wol*wol + kp / arm_mass);
    // Damping ratio.
    //const double zeta = 0.001;
    //const double kd = 2.0 * arm_mass * wcl * zeta;
    const double kd = 1.0;

    controller_ = make_unique<PidController<T>>(kp, ki, kd,
        plant_->get_num_positions());
    gravity_compensator_ = make_unique<GravityCompensator<T>>(
        plant_->get_multibody_world());
    gcomp_minus_pid_ = make_unique<Adder<T>>(
        2 /*number of inputs*/, plant_->get_num_actuators() /* size */);

    // Split the input state into two signals one with the positions and one
    // with the velocities.
    // For Kuka:
    // -  get_num_states() = 14
    // -  get_num_positions() = 7
    demux_ = make_unique<Demultiplexer<T>>(plant_->get_num_states(),
        plant_->get_num_positions());

    inverter_ = make_unique<Gain<T>>(-1.0, plant_->get_num_actuators());
    error_inverter_ = make_unique<Gain<T>>(-1.0, plant_->get_num_states());

    // Creates a plan.
    //poly_trajectory_ = MakePlan();
    // GET THIS TRAJECTORY FROM plan_listener

    target_plan_ = make_unique<TimeVaryingPolynomialSource<T>>(
        *poly_trajectory_);

    viz_publisher_ = make_unique<BotVisualizerSystem>(
        plant_->get_multibody_world(), &lcm_);

    DiagramBuilder<T> builder;
    // Target.
    // TODO(amcastro-tri): connect planner output.
    //builder.Connect(target_state_->get_output_port(0),
    //                error_inverter_->get_input_port(0));
    builder.Connect(target_plan_->get_output_port(0),
                    error_inverter_->get_input_port(0));
    builder.Connect(error_inverter_->get_output_port(0),
                    state_minus_target_->get_input_port(0));
    builder.Connect(plant_->get_output_port(0),
                    state_minus_target_->get_input_port(1));

    // Splits plant output port into a positions and velocities ports.
    builder.Connect(state_minus_target_->get_output_port(0),
                    demux_->get_input_port(0));

    builder.Connect(demux_->get_output_port(0),
                    controller_->get_error_signal_port());
    builder.Connect(demux_->get_output_port(1),
                    controller_->get_error_signal_rate_port());

    // Connects the gravity compensator.
    builder.Connect(plant_->get_output_port(0),
                    gravity_compensator_->get_input_port(0));
    builder.Connect(gravity_compensator_->get_output_port(0),
                    gcomp_minus_pid_->get_input_port(0));

    // Adds feedback.
    builder.Connect(controller_->get_output_port(0),
                    inverter_->get_input_port(0));
    builder.Connect(inverter_->get_output_port(0),
                    gcomp_minus_pid_->get_input_port(1));

    builder.Connect(gcomp_minus_pid_->get_output_port(0),
                    plant_->get_input_port(0));

    // Connects to visualizer.
    builder.Connect(plant_->get_output_port(0),
                    viz_publisher_->get_input_port(0));
    builder.ExportOutput(plant_->get_output_port(0));
    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_kuka_plant() const { return *plant_; }

  void SetDefaultState(ContextBase<T>* context) const {
    ContextBase<T>* controller_context =
        Diagram<T>::GetMutableSubSystemContext(context, controller_.get());
    controller_->set_integral_value(controller_context, VectorX<T>::Zero(7));

    ContextBase<T>* plant_context =
        Diagram<T>::GetMutableSubSystemContext(context, plant_.get());

    plant_->ObtainZeroConfiguration(plant_context);
  }

 private:
  std::unique_ptr<RigidBodyPlant<T>> plant_;
  std::unique_ptr<PidController<T>> controller_;
  std::unique_ptr<Demultiplexer<T>> demux_;
  std::unique_ptr<Gain<T>> inverter_;
  std::unique_ptr<Gain<T>> error_inverter_;
  std::unique_ptr<GravityCompensator<T>> gravity_compensator_;
  std::unique_ptr<Adder<T>> state_minus_target_;
  std::unique_ptr<Adder<T>> gcomp_minus_pid_;
  std::unique_ptr<ConstantVectorSource<T>> target_state_;
  std::unique_ptr<TimeVaryingPolynomialSource<T>> target_plan_;
  std::unique_ptr<PiecewisePolynomial<T>> poly_trajectory_;
  std::unique_ptr<BotVisualizerSystem> viz_publisher_;
  ::lcm::LCM lcm_;
};

int DoMain() {
  KukaDemo<double> model;
  Simulator<double> simulator(model);  // Use default Context.

  // Zeroes the state and initializes controller state.
  model.SetDefaultState(simulator.get_mutable_context());

  VectorX<double> desired_state = VectorX<double>::Zero(14);
#if 0
  desired_state[0] =   0.0 * deg_to_rad;  // base.
  desired_state[1] =  45.0 * deg_to_rad;  // first elbow.
  desired_state[2] =   0.0 * deg_to_rad;  // axial rotation.
  desired_state[3] = -45.0 * deg_to_rad;  // second elbow.
  desired_state[4] =  90.0 * deg_to_rad;  // axial rotation.
  desired_state[5] =   0.0 * deg_to_rad;  // final wrist
  desired_state[6] =   0.0 * deg_to_rad;  // end effector rotation.
#endif
  model.get_kuka_plant().set_state_vector(
      simulator.get_mutable_context(), desired_state);

  simulator.request_initial_step_size_attempt(0.002);

  // Take all the defaults.
  simulator.Initialize();

  EXPECT_TRUE(simulator.get_integrator_type_in_use() ==
      IntegratorType::RungeKutta2);

  // Simulate for 1 seconds.
  simulator.StepTo(20.0);

  const auto& context = simulator.get_context();
  EXPECT_EQ(context.get_time(), 1.);  // Should be exact.

  EXPECT_EQ(simulator.get_num_steps_taken(), 500);
  EXPECT_EQ(simulator.get_num_samples_taken(), 0);
  EXPECT_LE(simulator.get_smallest_step_size_taken(),
            simulator.get_largest_step_size_taken());
  return 0;
}


}  // namespace
}  // namespace test
}  // namespace rigid_body_system
}  // namespace plants
}  // namespace systems
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::systems::plants::rigid_body_system::test::DoMain();
}