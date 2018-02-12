#include <fstream>
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/dev/quasistatic_system.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/signal_logger.h"

using std::cout;
using std::endl;
using std::sin;
using std::cos;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::systems::BasicVector;

namespace drake {
namespace manipulation {
namespace rod2d {

const char* kUrdfPath = "drake/manipulation/dev/rod2d.urdf";

class Rod2DTimeStepping : public manipulation::QuasistaticSystem {
 public:
  Rod2DTimeStepping(const double period_sec, const bool is_analytic,
                    const double mu = 0.6, const double kBigM = 1);

 private:
  void DoCalcWnWfJnJfPhiAnalytic(const KinematicsCache<double>& cache,
                                 MatrixXd& Wn, MatrixXd& Wf, MatrixXd& Jn,
                                 MatrixXd& Jf, VectorXd& phi) const override;

  const double r_ = 0.01;
  const double l_ = 0.5;

  static const std::vector<int> fixed_base_positions;
  static const std::vector<int> fixed_base_velocities;
  static const std::vector<int> idx_unactuated_bodies;
  static const std::vector<bool> is_contact_2d;
  static const int idx_base;
};

// definition of static varaibles.
const std::vector<int> Rod2DTimeStepping::fixed_base_positions = {3};   // roll
const std::vector<int> Rod2DTimeStepping::fixed_base_velocities = {3};  // roll
const std::vector<int> Rod2DTimeStepping::idx_unactuated_bodies = {3};  // rod
const int Rod2DTimeStepping::idx_base = 3;
const std::vector<bool> Rod2DTimeStepping::is_contact_2d = {false, false, true,
                                                            true};

Rod2DTimeStepping::Rod2DTimeStepping(const double period_sec,
                                     const bool is_analytic, const double mu,
                                     const double kBigM)
    : QuasistaticSystem(period_sec, idx_unactuated_bodies, idx_base,
                        fixed_base_positions, fixed_base_velocities,
                        is_contact_2d, mu, kBigM, is_analytic) {
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(kUrdfPath), multibody::joints::kFixed, tree_.get());
  Initialize();
}

void Rod2DTimeStepping::DoCalcWnWfJnJfPhiAnalytic(
    const KinematicsCache<double>& cache, MatrixXd& Wn, MatrixXd& Wf,
    MatrixXd& Jn, MatrixXd& Jf, VectorXd& phi) const {
  const int nq_tree = tree_->get_num_positions();
  auto q = getQuasistaticSystemStatesFromRigidBodyTreePositions(cache);
  const double qa_l = q(0);
  const double yc = q(2);
  const double zc = q(3);
  const double yaw = q(5);
  const double s = l_ / 2 * sin(yaw);
  const double c = l_ / 2 * cos(yaw);

  phi.resize(nc_);
  Jn.resize(nc_, nq_tree);
  Jf.resize(nd_, nq_tree);
  Jf.setZero();

  phi(3) = yc - s - r_ - qa_l;
  phi(2) = yc + s - r_ - qa_l;
  phi(1) = zc - r_;
  phi(0) = zc - r_;

  Jn << 0, 0, 0, 1, 0, -1, 0,  //
      0, 0, 0, 1, 0, 1, 0,     //
      -1, 0, 1, 0, 0, 0, c,    //
      -1, 0, 1, 0, 0, 0, -c;   //

  Jf.block(0, 1, nd_, nu_ + 1) << 1, 0, 0, 0, 0, -s,  //
      0, 1, 0, 0, 0, c,                               //
      -1, 0, 0, 0, 0, s,                              //
      0, -1, 0, 0, 0, -c,                             //

      1, 0, 0, 0, 0, s,    //
      0, 1, 0, 0, 0, -c,   //
      -1, 0, 0, 0, 0, -s,  //
      0, -1, 0, 0, 0, c,   //

      1, 0, 0, 0, 0, -s,  //
      -1, 0, 0, 0, 0, s,  //

      1, 0, 0, 0, 0, s,    //
      -1, 0, 0, 0, 0, -s;  //

  Wn = Jn.transpose().block(1, 0, 6, nc_);
  Wf = Jf.transpose().block(1, 0, 6, nd_);
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  // Initialize Rod2D quasistatic system.
  const double h = 0.05;  // time step
  const bool is_analytic = false;
  auto rod2d = builder.AddSystem<Rod2DTimeStepping>(h, is_analytic);
  // Initrialize RigidBodyTree from rod2d.urdf.
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(kUrdfPath), multibody::joints::kFixed, tree.get());

  // Figure out the index of each joint.
  cout << "number of states: " << tree->get_num_positions() << endl;
  for (int i = 0; i < tree->get_num_positions(); i++) {
    cout << "position name " << i << ": " << tree->get_position_name(i) << endl;
  }
  cout << "number of velocities: " << tree->get_num_velocities() << endl;
  for (int i = 0; i < tree->get_num_velocities(); i++) {
    cout << i << " " << tree->get_velocity_name(i) << endl;
  }

  // Use matrix gain to connect the states to the correct output.
  const int n1 = rod2d->state_output().size();
  const int n_states = tree->get_num_positions() + tree->get_num_velocities();
  MatrixXd D(n_states, n1);
  D.setIdentity();
  // cout << "D\n" << D << endl;
  // Last four rows correspond to velocity and are set to zero because
  // drake_visualizer ignores them anyway.
  auto gain = builder.AddSystem<systems::MatrixGain>(D);
  builder.Connect(rod2d->state_output(), gain->get_input_port());

  // Initialize drake_visualizer
  lcm::DrakeLcm lcm;
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(gain->get_output_port(), publisher->get_input_port(0));

  // Initialize SignalLogger that logs output to drake_visualizer.
  auto log_state =
      builder.AddSystem<drake::systems::SignalLogger<double>>(n_states);

  // Initialize SingalLogger that logs decision variables.
  const int n = rod2d->decision_variables_output().size();  // number of
  // decision varialbes
  auto log_decision_variables =
      builder.AddSystem<drake::systems::SignalLogger<double>>(n);

  // Initialize constant input to rod2d.
  Matrix<double, 1, 1> input_vector(0.1);  // qa: input to the system
  auto const_source =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
          input_vector);

  // Connect system ports.
  builder.Connect(gain->get_output_port(), log_state->get_input_port());
  builder.Connect(rod2d->decision_variables_output(),
                  log_decision_variables->get_input_port());
  builder.Connect(const_source->get_output_port(), rod2d->get_input_port(0));

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  // Set the initial conditions x(0).
  drake::systems::BasicVector<double>& x0 = simulator.get_mutable_context()
                                                .get_mutable_discrete_state()
                                                .get_mutable_vector();

  x0.SetZero();
  cout << "discrete state vector size: " << x0.size() << endl;
  x0.SetAtIndex(0, 0.25);      // qa
  x0.SetAtIndex(1, 0.0);       // x
  x0.SetAtIndex(2, 0.5);       // y
  x0.SetAtIndex(3, 0.01);      // z = r
  x0.SetAtIndex(5, M_PI / 6);  // yaw

  simulator.set_target_realtime_rate(1);
  simulator.get_mutable_integrator()->set_maximum_step_size(h);
  simulator.Initialize();
  simulator.StepTo(h * 150);
  cout << "done stepping" << endl;

  // test if the final system state is reasonable.
  VectorXd q_final_expected(7);  // [x,y,qa,theta, zeros(4,1)]
  q_final_expected << 1, 0.0117, 1.01, 0.01, 0, 0, 0;

  const double error =
      ((log_state->data().rightCols(1)).block(0, 0, n1, 1) - q_final_expected)
          .matrix()
          .norm();
  cout << "final_state:\n" << log_state->data().rightCols(1) << endl;
  cout << "error: " << error << endl;
  DRAKE_DEMAND(error < 1e-3);

  // write log to a file

  // std::ofstream s;
  // s.open("log.txt");
  // cout << "DATA" << endl;
  // cout << log_state->data() << endl;

  // cout << "TIME" << endl;
  // cout << log_state->sample_times() << endl;
  // s.close();

  return 0;
}

}  // namespace rod2d
}  // namespace manipulation
}  // namespace drake

int main() {
  return drake::manipulation::rod2d::do_main();
}
