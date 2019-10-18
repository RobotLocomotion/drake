#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/dev/quasistatic_system.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace manipulation {
namespace quasistatic_kuka_iiwa_arm {
namespace {
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using manipulation::util::ModelInstanceInfo;
using manipulation::util::SimDiagramBuilder;
using manipulation::util::WorldSimTreeBuilder;
using math::RollPitchYaw;
using trajectories::PiecewisePolynomial;

const Eigen::Vector3d kTableBase(0.82, 0, 0);
const Eigen::Vector3d kDumbbellBase(0.5 + 0.0725 + 0.035, 0, 0.815);

const char kUrdfPath[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "/iiwa14_no_collision.urdf";

const char kUrdfPathDumbbell[] =
    "drake/manipulation/dev/double_dumbbell_for_pick_up.sdf";

std::unique_ptr<RigidBodyTreed> BuildSimWorld() {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
  tree_builder->StoreDrakeModel("iiwa", kUrdfPath);
  tree_builder->StoreDrakeModel(
      "wsg", "drake/manipulation/dev/simplified_schunk_gripper.sdf");
  tree_builder->StoreDrakeModel("table",
                                "drake/manipulation/dev/box_as_table"
                                ".sdf");
  tree_builder->StoreDrakeModel("dumbbell", kUrdfPathDumbbell);

  tree_builder->AddFloatingModelInstance("dumbbell", Vector3d::Zero());
  tree_builder->AddFixedModelInstance("table", kTableBase);
  int id_iiwa =
      tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(0, 0, 0));
  tree_builder->AddModelInstanceToFrame(
      "wsg", tree_builder->tree().findFrame("iiwa_frame_ee", id_iiwa),
      drake::multibody::joints::kFixed);

  return tree_builder->Build();
}

const std::vector<double> kTimes{0.0, 2.0, 4.0, 6.0, 12.0};

// Creates a basic pointwise IK trajectory for moving the iiwa arm.
// It starts in the zero configuration (straight up).
PiecewisePolynomial<double> MakePlan(
    const Vector3d& pos_end_final, const RollPitchYaw<double>& rpy_end_final) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(kUrdfPath), multibody::joints::kFixed, tree.get());

  VectorXd zero_conf = tree->getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.005);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.005);

  PostureConstraint pc1(tree.get(), Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  Vector3d pos_end(0.48, 0, 0.804);
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.002);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.002);
  WorldPositionConstraint wpc1(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(1.5, 4.0));
  const RollPitchYaw<double> rpy_end1(0, 0, 0);
  Eigen::Quaterniond qE = rpy_end1.ToQuaternion();
  Eigen::Vector4d quat_end(qE.w(), qE.x(), qE.y(), qE.z());
  WorldQuatConstraint wqc1(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                           quat_end, 0.002, Vector2d(1.5, 5.0));

  pos_end << 0.4, 0, 0.9;
  pos_lb = pos_end - Vector3d::Constant(0.002);
  pos_ub = pos_end + Vector3d::Constant(0.002);
  WorldPositionConstraint wpc2(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(5.5, 6.0));
  const RollPitchYaw<double> rpy_end2(0, -M_PI / 2, 0);
  qE = rpy_end2.ToQuaternion();
  quat_end << qE.w(), qE.x(), qE.y(), qE.z();
  WorldQuatConstraint wqc2(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                           quat_end, 0.002, Vector2d(5.5, 6.0));

  pos_end = pos_end_final;
  pos_lb = pos_end - Vector3d::Constant(0.002);
  pos_ub = pos_end + Vector3d::Constant(0.002);
  WorldPositionConstraint wpc4(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(11.5, 12.0));

  qE = rpy_end_final.ToQuaternion();
  quat_end << qE.w(), qE.x(), qE.y(), qE.z();
  WorldQuatConstraint wqc3(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                           quat_end, 0.002, Vector2d(11.5, 12.0));

  MatrixXd q0(tree->get_num_positions(), kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    q0.col(i) = zero_conf;
  }

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc1);
  constraint_array.push_back(&wqc1);
  constraint_array.push_back(&wpc2);
  constraint_array.push_back(&wqc2);
  constraint_array.push_back(&wpc4);
  constraint_array.push_back(&wqc3);

  IKoptions ikoptions(tree.get());
  std::vector<int> info(kTimes.size(), 0);
  MatrixXd q_sol(tree->get_num_positions(), kTimes.size());

  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(tree.get(), kTimes.size(), kTimes.data(), q0, q0,
                      constraint_array.size(), constraint_array.data(),
                      ikoptions, &q_sol, info.data(), &infeasible_constraint);
  bool info_good = true;
  for (size_t i = 0; i < kTimes.size(); ++i) {
    drake::log()->info("INFO[{}] = {} ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }

  if (!info_good) {
    throw std::runtime_error(
        "inverseKinPointwise failed to compute a feasible solution.");
  }

  std::vector<MatrixXd> knots(kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    knots[i] = q_sol.col(i);
  }

  return PiecewisePolynomial<double>::FirstOrderHold(kTimes, knots);
}

class IiwaPickUpDumbbell : public manipulation::QuasistaticSystem<double> {
 public:
  IiwaPickUpDumbbell();

 private:
  static const int idx_base;
  static const std::vector<int> idx_unactuated_bodies;
  static const std::vector<int> fixed_base_positions;
  static const std::vector<int> fixed_base_velocities;
  static QuasistaticSystemOptions InitializeOptions();
};

const int IiwaPickUpDumbbell::idx_base = 1;
const std::vector<int> IiwaPickUpDumbbell::idx_unactuated_bodies = {1};
const std::vector<int> IiwaPickUpDumbbell::fixed_base_positions{};
const std::vector<int> IiwaPickUpDumbbell::fixed_base_velocities{};

QuasistaticSystemOptions IiwaPickUpDumbbell::InitializeOptions() {
  QuasistaticSystemOptions options;
  options.period_sec = 0.005;
  options.mu = 0.5;
  options.is_using_kinetic_energy_minimizing_QP = false;
  return options;
}

IiwaPickUpDumbbell::IiwaPickUpDumbbell()
    : QuasistaticSystem(idx_base, idx_unactuated_bodies, fixed_base_positions,
                        fixed_base_velocities, InitializeOptions()) {
  tree_ = BuildSimWorld();
  Initialize();
}

int do_main() {
  // Loads model into a RigidBodyTree.
  auto tree = BuildSimWorld();

  // Builds simulation diagram.
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  auto iiwa_pickup = builder.AddSystem<IiwaPickUpDumbbell>();
  const double h = iiwa_pickup->get_period_sec();  // time step

  const int n1 = iiwa_pickup->state_output().size();
  const int n_states = tree->get_num_positions() + tree->get_num_velocities();
  MatrixXd D1(n_states, n1);
  D1.setIdentity();
  auto gain1 = builder.AddSystem<systems::MatrixGain>(D1);
  builder.Connect(iiwa_pickup->state_output(), gain1->get_input_port());

  auto publisher =
      builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm, true);
  builder.Connect(gain1->get_output_port(), publisher->get_input_port(0));

  // generates trajectory source for iiwa and gripper
  const Vector3d pos_EE_final(0.2, 0.2, 0.8);
  const RollPitchYaw<double> rpy_EE_final(M_PI / 4, 0, 0);
  PiecewisePolynomial<double> iiwa_traj = MakePlan(pos_EE_final, rpy_EE_final);
  auto iiwa_traj_src =
      builder.template AddSystem<systems::TrajectorySource<double>>(iiwa_traj,
                                                                    1);

  std::vector<MatrixXd> knots(kTimes.size(), VectorXd::Zero(2));
  knots[1] << 0.05, -0.05;
  knots[2] << 0.05, -0.05;
  knots[3] << 0.05, -0.05;
  knots[4] << 0.05, -0.05;
  PiecewisePolynomial<double> gripper_traj =
      PiecewisePolynomial<double>::ZeroOrderHold(kTimes, knots);

  auto gripper_qdot_src =
      builder.template AddSystem<systems::TrajectorySource<double>>(
          gripper_traj, 0);

  // picks the qdot from iiwa_traj_src output using matrix gain
  const int n_qa_dot = iiwa_traj_src->num_total_outputs() / 2;
  MatrixXd D2(n_qa_dot, n_qa_dot * 2);
  D2.setZero();
  D2.block(0, n_qa_dot, n_qa_dot, n_qa_dot).setIdentity();
  auto gain2 = builder.template AddSystem<systems::MatrixGain>(D2);
  builder.Connect(iiwa_traj_src->get_output_port(), gain2->get_input_port());

  // mux iiwa velocity and gripper velocity
  std::vector<int> qdot_sizes = {7, 2};
  auto mux = builder.template AddSystem<systems::Multiplexer>(qdot_sizes);
  builder.Connect(gain2->get_output_port(), mux->get_input_port(0));
  builder.Connect(gripper_qdot_src->get_output_port(), mux->get_input_port(1));

  builder.Connect(mux->get_output_port(0), iiwa_pickup->get_input_port(0));

  // set up signal loggers
  auto log_state = builder.AddSystem<drake::systems::SignalLogger<double>>(n1);
  const int n2 = iiwa_pickup->decision_variables_output().size();
  auto log_decision_variables =
      builder.AddSystem<drake::systems::SignalLogger<double>>(n2);

  builder.Connect(iiwa_pickup->state_output(), log_state->get_input_port());
  builder.Connect(iiwa_pickup->decision_variables_output(),
                  log_decision_variables->get_input_port());

  // publisher->set_publish_period(0.05);
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  // Set the initial conditions x(0).
  drake::systems::BasicVector<double>& x0 = simulator.get_mutable_context()
                                                .get_mutable_discrete_state()
                                                .get_mutable_vector();

  x0.SetZero();
  x0.SetAtIndex(0, kDumbbellBase(0));  // x
  x0.SetAtIndex(1, kDumbbellBase(1));  // y
  x0.SetAtIndex(2, kDumbbellBase(2));  // z
  x0.SetAtIndex(3, 1);                 // qw1
  x0.SetAtIndex(14, -0.055);           // left gripper
  x0.SetAtIndex(15, 0.055);            // right gripper

  simulator.set_target_realtime_rate(0);
  simulator.get_mutable_integrator().set_maximum_step_size(h);
  simulator.Initialize();
  simulator.AdvanceTo(kTimes.back());

  // Comparing the final position and orientation of the object in world frame
  // to expected values.
  const int kDumbbellIndex = 1;
  const int kIiwaEndEffectorIndex = 13;

  // compute offset in the x-direction (in the EE-frame) from the origin of
  // the EE to the origin of the object (dumbbell). The x-axes of the end
  // effector and the object are aligned with the x-axis of the world frame
  // at t = 3.0, the time at which x_offset is calculated.
  const Eigen::VectorXd q_grasping =
      log_state->data().col(static_cast<int>(3.0 / h));
  VectorXd v(tree->get_num_velocities());
  v.setZero();
  KinematicsCache<double> cache = tree->CreateKinematicsCache();
  cache.initialize(q_grasping, v);
  tree->doKinematics(cache);
  auto transform = tree->CalcBodyPoseInWorldFrame(
      cache, tree->get_body(kIiwaEndEffectorIndex));
  const double x_offset = q_grasping[0] - (transform.translation())[0];

  // compute expected_object_CG_final_position =
  // expected_end_effector_final_position + [x_offset, 0, 0]
  const Eigen::Vector3d pos_offset(x_offset, 0, 0);
  const Eigen::Vector3d position_final_expected = pos_EE_final + pos_offset;

  // compute expected_object_orientation as a rotation matrix.
  const math::RotationMatrix<double> R_dumbbell_expected(
      rpy_EE_final.ToQuaternion());

  // compute object final position and orientation.
  const Eigen::VectorXd q_final = log_state->data().rightCols(1);
  cache.initialize(q_final, v);
  tree->doKinematics(cache);
  transform =
      tree->CalcBodyPoseInWorldFrame(cache, tree->get_body(kDumbbellIndex));

  EXPECT_TRUE(CompareMatrices(position_final_expected, q_final.head<3>(), 1e-2,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(R_dumbbell_expected.matrix(), transform.linear(),
                              2e-2, MatrixCompareType::absolute));

  return 0;
}

}  // namespace
}  // namespace quasistatic_kuka_iiwa_arm
}  // namespace manipulation
}  // namespace drake

int main() { return drake::manipulation::quasistatic_kuka_iiwa_arm::do_main(); }
