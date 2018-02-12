#include <fstream>
#include "drake/manipulation/dev/quasistatic_system.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/dev/quasistatic_system.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace manipulation {
namespace quasistatic_kuka_iiwa_arm {
namespace {
using manipulation::util::ModelInstanceInfo;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::util::SimDiagramBuilder;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using std::cout;
using std::endl;
using math::rpy2quat;

const Eigen::Vector3d kTableBase(0.82, 0, 0);
const Eigen::Vector3d kDumbbellBase(0.5 + 0.0725 + 0.035, 0, 0.815);

const char kUrdfPath[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "/iiwa14_polytope_no_collision.urdf";

const char kUrdfPathDumbbell[] =
    "drake/manipulation/dev/double_dumbbell_for_pick_up"
    ".sdf";

template <typename T>
void print_stl_vector(const std::vector<T>& v) {
  for (const T& i : v) {
    cout << i << " ";
  }
  cout << endl;
}

std::unique_ptr<RigidBodyTreed> BuildSimWorld() {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
  tree_builder->StoreDrakeModel("iiwa", kUrdfPath);
  tree_builder->StoreDrakeModel(
      "wsg", "drake/manipulation/dev/simplified_schunk_gripper.sdf");
  tree_builder->StoreDrakeModel("table", "drake/manipulation/dev/box_as_table"
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

//////////////////////////////////////////////////////////////////////////
const std::vector<double> kTimes{0.0, 2.0, 4.0, 6.0, 12.0};

std::unique_ptr<PiecewisePolynomialTrajectory> MakePlan() {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(kUrdfPath), multibody::joints::kFixed, tree.get());

  // Creates a basic pointwise IK trajectory for moving the iiwa arm.
  // It starts in the zero configuration (straight up).
  VectorXd zero_conf = tree->getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.005);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.005);

  PostureConstraint pc1(tree.get(), Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Defines an end effector constraint and makes it active
  // from 1.5s to 4.0s.
  Vector3d pos_end(0.48, 0, 0.804);
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.002);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.002);
  WorldPositionConstraint wpc1(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(1.5, 4.0));
  Eigen::Vector3d rpy_end(0, 0, 0);
  Eigen::Vector4d quat_end(math::rpy2quat(rpy_end));
  WorldQuatConstraint wqc1(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                           quat_end, 0.002, Vector2d(1.5, 5.0));

  // After the end effector constraint is released, applies the straight
  // up configuration again from time 5.5 to 6.
  pos_end << 0.4, 0, 0.9;
  pos_lb = pos_end - Vector3d::Constant(0.002);
  pos_ub = pos_end + Vector3d::Constant(0.002);
  WorldPositionConstraint wpc2(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(5.5, 6.0));
  rpy_end << 0, -M_PI / 2, 0;
  quat_end = math::rpy2quat(rpy_end);
  WorldQuatConstraint wqc2(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                           quat_end, 0.002, Vector2d(5.5, 6.0));

  pos_end << 0.2, 0.2, 0.8;
  pos_lb = pos_end - Vector3d::Constant(0.002);
  pos_ub = pos_end + Vector3d::Constant(0.002);
  WorldPositionConstraint wpc4(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(11.5, 12.0));
  rpy_end << M_PI / 4, 0, 0;
  quat_end = math::rpy2quat(rpy_end);
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
  printf("\n");

  if (!info_good) {
    throw std::runtime_error(
        "inverseKinPointwise failed to compute a valid solution.");
  }

  std::vector<MatrixXd> knots(kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    // We only use column 0 of the matrix in knots (for joint positions),
    // so we write a vector.
    knots[i] = q_sol.col(i);
  }

  return std::make_unique<PiecewisePolynomialTrajectory>(
      PiecewisePolynomial<double>::FirstOrderHold(kTimes, knots));
}
///////////////////////////////////////////////////////////////////////////////

class IiwaPickUpDumbbell : public manipulation::QuasistaticSystem {
 public:
  IiwaPickUpDumbbell(
      const double period_sec, const double mu = 0.5, const double kBigM = 1);

 private:
  static const std::vector<int> idx_unactuated_bodies;
  static const int idx_base;
  static const std::vector<bool> is_contact_2d;
};

const std::vector<int> IiwaPickUpDumbbell::idx_unactuated_bodies = {1};
const int IiwaPickUpDumbbell::idx_base = 1;
const std::vector<bool> IiwaPickUpDumbbell::is_contact_2d = {
    false, false, false, false, false, false};

IiwaPickUpDumbbell::IiwaPickUpDumbbell(const double period_sec, const double mu,
                                       const double kBigM)
    : QuasistaticSystem(period_sec, idx_unactuated_bodies, idx_base, {}, {},
                        is_contact_2d, mu, kBigM) {
  tree_ = BuildSimWorld();
  Initialize();
}

int do_main(int argc, char* argv[]) {
  // command line arguments
  double t_final = 4.0;
  double BigM = 1;
  double time_step = 0.02;
  if (argc > 1) {
    t_final = std::stod(argv[1]);
  }
  if (argc > 2) {
    BigM = std::stod(argv[2]);
  }
  if (argc > 3) {
    time_step = std::stod(argv[3]);
  }

  auto tree = BuildSimWorld();

  // understanding the model
  VectorXd q_iiwa(7);
  q_iiwa << 0, 0, 0, -M_PI / 2, 0, 0, 0;
  VectorXd q_gripper(2);
  q_gripper << -0.055, 0.055;
  VectorXd q_dumbbell(7);
  q_dumbbell << kDumbbellBase, rpy2quat(Eigen::Vector3d::Zero());
  VectorXd q(tree->get_num_positions());
  q << q_dumbbell, q_iiwa, q_gripper;

  cout << "model instances: " << tree->get_num_model_instances() << endl;
  for (int i = 0; i < tree->get_num_positions(); i++) {
    cout << i << ": " << tree->get_position_name(i) << " " << q(i) << endl;
  }
  for (int i = 0; i < tree->get_num_bodies(); i++) {
    cout << "Body no. " << i << " " << tree->get_body(i).get_name() << endl;
    auto body0_collision = tree->get_body(i).get_collision_element_ids();
    cout << "No. of collision elements: " << body0_collision.size() << endl;
  }

  cout << "Num of bodies:" << tree->get_num_bodies() << endl;
  cout << "Num of positions:" << tree->get_num_positions() << endl;
  cout << "Num of velocities:" << tree->get_num_velocities() << endl;

  VectorXd v(tree->get_num_velocities());
  auto cache = tree->doKinematics(q, v);
  VectorXd phi;
  Eigen::Matrix3Xd normals, xA, xB;
  std::vector<int> bodyA_idx;
  // bodyA_idx.push_back(0);
  std::vector<int> bodyB_idx;
  // bodyA_idx.push_back(1);
  cout << "collision? ";
  cout << tree->collisionDetect(cache, phi, normals, xA, xB, bodyA_idx,
                                bodyB_idx)
       << endl;

  cout << "phi\n" << phi << endl;
  cout << "xA\n" << xA << endl;
  cout << "xB\n" << xB << endl;
  cout << "bodyA_idx.size(): " << bodyA_idx.size() << endl;
  cout << "bodyA_idx: ";
  print_stl_vector(bodyA_idx);
  cout << "bodyB_idx.size(): " << bodyB_idx.size() << endl;
  cout << "bodyB_idx: ";
  print_stl_vector(bodyB_idx);

  std::cin.get();

  // simulation
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  const double h = time_step;  // time step
  auto iiwa_pickup = builder.AddSystem<IiwaPickUpDumbbell>(h, 0.5, BigM);

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
  std::unique_ptr<PiecewisePolynomialTrajectory> iiwa_traj = MakePlan();
  auto iiwa_traj_src =
      builder.template AddSystem<systems::TrajectorySource<double>>(*iiwa_traj,
                                                                    1);

  std::vector<MatrixXd> knots(kTimes.size(), VectorXd::Zero(2));
  knots[1] << 0.05, -0.05;
  knots[2] << 0.05, -0.05;
  knots[3] << 0.05, -0.05;
  knots[4] << 0.05, -0.05;
  PiecewisePolynomial<double> poly =
      PiecewisePolynomial<double>::ZeroOrderHold(kTimes, knots);

  auto gripper_qdot_src =
      builder.template AddSystem<systems::TrajectorySource<double>>(
          PiecewisePolynomialTrajectory(poly), 0);

  // picks the qdot segment of iiwa_traj_src output using matrix gain
  const int n_qa_dot = iiwa_traj_src->get_num_total_outputs() / 2;
  MatrixXd D2(n_qa_dot, n_qa_dot * 2);
  D2.setZero();
  D2.block(0, n_qa_dot, n_qa_dot, n_qa_dot).setIdentity();
  cout << "D2\n" << D2 << endl;
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
  cout << "discrete state vector size: " << x0.size() << endl;
  x0.SetAtIndex(0, kDumbbellBase(0));  // x
  x0.SetAtIndex(1, kDumbbellBase(1));  // y
  x0.SetAtIndex(2, kDumbbellBase(2));  // z
  x0.SetAtIndex(3, 1);                 // qw1
  x0.SetAtIndex(14, -0.055);           // left gripper
  x0.SetAtIndex(15, 0.055);            // right gripper

  simulator.set_target_realtime_rate(0);
  simulator.get_mutable_integrator()->set_maximum_step_size(h);
  simulator.Initialize();

  simulator.StepTo(t_final);

  // save logs to a file
  std::ofstream s1, s2;
  s1.open("state_log.txt");
  s1 << log_state->data() << endl;
  s1.close();

  s2.open("decision_variables_log.txt");
  s2 << log_decision_variables->data() << endl;
  s2.close();

  // cout << "TIME" << endl;
  // cout << log_state->sample_times() << std::flush << endl;
  // cout << log_decision_variables->sample_times() << std::flush << endl;

//  while (true) {
//    publisher->ReplayCachedSimulation();
//  }

  return 0;
}

}  // namespace
}  // namespace quasistatic_kuka_iiwa_arm
}  // namespace manipulation
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::manipulation::quasistatic_kuka_iiwa_arm::do_main(argc, argv);
}
