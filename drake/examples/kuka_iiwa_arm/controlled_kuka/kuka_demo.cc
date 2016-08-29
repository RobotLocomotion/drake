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

unique_ptr<PiecewisePolynomial<double>> MakePlan() {
  RigidBodyTree tree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED);

  // Create a basic pointwise IK trajectory for moving the iiwa arm.
  // We start in the zero configuration (straight up).

  // TODO(sam.creasey) We should start planning with the robot's
  // current position rather than assuming vertical.
  VectorXd zero_conf = tree.getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  PostureConstraint pc1(&tree, Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Define an end effector constraint and make it active for the
  // timespan from 1 to 3 seconds.
  Vector3d pos_end;
  pos_end << 0.6, 0, 0.325;
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
  WorldPositionConstraint wpc(&tree, tree.FindBodyIndex("iiwa_link_ee"),
                              Vector3d::Zero(), pos_lb, pos_ub, Vector2d(1, 3));

  // After the end effector constraint is released, apply the straight
  // up configuration again.
  PostureConstraint pc2(&tree, Vector2d(4, 5.9));
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Bring back the end effector constraint through second 9 of the
  // demo.
  WorldPositionConstraint wpc2(&tree, tree.FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(6, 9));

  // For part of the remaining time, constrain the second joint while
  // preserving the end effector constraint.
  //
  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) = tree.FindChildBodyOfJoint("iiwa_joint_2")->
      get_position_start_index();
  PostureConstraint pc3(&tree, Vector2d(6, 8));
  pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));


  const int kNumTimesteps = 5;
  double t[kNumTimesteps] = { 0.0, 2.0, 5.0, 7.0, 9.0 };
  MatrixXd q0(tree.number_of_positions(), kNumTimesteps);
  for (int i = 0; i < kNumTimesteps; i++) {
    q0.col(i) = zero_conf;
  }

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);
  IKoptions ikoptions(&tree);
  int info[kNumTimesteps];
  MatrixXd q_sol(tree.number_of_positions(), kNumTimesteps);
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(&tree, kNumTimesteps, t, q0, q0, constraint_array.size(),
                      constraint_array.data(), ikoptions, &q_sol, info,
                      &infeasible_constraint);
  bool info_good = true;
  for (int i = 0; i < kNumTimesteps; i++) {
    printf("INFO[%d] = %d ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }
  printf("\n");

  if (!info_good) {
    throw std::runtime_error("Solution failed, not sending.");
  }

  // This comes from TrajectoryRunner in kuka_id_demo
  typedef PiecewisePolynomial<double> PPType;
  typedef PPType::PolynomialType PPPoly;
  typedef PPType::PolynomialMatrix PPMatrix;
  std::vector<PPMatrix> polys;
  std::vector<double> times;
  const int nT_ = kNumTimesteps;

  MatrixXd traj_(tree.number_of_positions() + tree.number_of_velocities(),
                 kNumTimesteps);
  traj_.setZero();
  traj_.block(0, 0, tree.number_of_positions(), kNumTimesteps) = q_sol;
  auto& t_ = t;

  PRINT_VAR(traj_);

  // For each timestep, create a PolynomialMatrix for each joint
  // position.  Each column of traj_ represents a particular time,
  // and the rows of that column contain values for each joint
  // coordinate.
  for (int i = 0; i < nT_; i++) {
    PPMatrix poly_matrix(traj_.rows(), 1);
    const auto traj_now = traj_.col(i);

    // Produce interpolating polynomials for each joint coordinate.
    if (i != nT_ - 1) {
      for (int row = 0; row < traj_.rows(); row++) {
        Eigen::Vector2d coeffs(0, 0);
        coeffs[0] = traj_now(row);
        // Set the coefficient such that it will reach the value of
        // the next timestep at the time when we advance to the next
        // piece.  In the event that we're at the end of the
        // trajectory, this will be left 0.
        coeffs[1] = (traj_(row, i + 1) - coeffs[0]) / (t_[i + 1] - t_[i]);
        poly_matrix(row) = PPPoly(coeffs);
      }
      polys.push_back(poly_matrix);
    }
    times.push_back(t_[i]);
  }
  auto pp_traj = make_unique<PPType>(polys, times);
  return pp_traj;
}

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
        drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
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
    poly_trajectory_ = MakePlan();
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

//GTEST_TEST(KukaDemo, Testing) {
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

  simulator.request_initial_step_size_attempt(0.001);

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