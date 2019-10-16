#include <gflags/gflags.h>

#include "drake/examples/box/box_geometry.h"
#include "drake/examples/box/two_boxes_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/solvers/solve.h"
#include "drake/systems/primitives/signal_logger.h"

#include <fstream>

using drake::solvers::SolutionResult;
using drake::systems::System;

namespace drake {
namespace examples {
namespace box {

using trajectories::PiecewisePolynomial;

namespace {
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(input_cost, 10.0,
              "Input cost ");
DEFINE_double(box1_force_limit, 100.0,
              "Box1 force limit (N) ");

DEFINE_double(penalty_k, 300.0,
              "k stiffness constant for contact penalty force (N/m)");

DEFINE_double(penalty_d, 10.0,
              "d damping constant for contact penalty force (s/m)");

DEFINE_double(box1_init_x, -0.1,
              "initial position for box 1 (m)");
DEFINE_double(box1_init_v, 0.2,
              "initial velocity for box 1 (m/s)");

DEFINE_double(box2_init_x, 0.1,
              "initial position for box 2 (m)");
DEFINE_double(box2_init_v, 0.,
              "initial velocity for box 2 (m/s)");
DEFINE_double(box_m, 0.33,
              "box mass (kg)");

DEFINE_double(box_d, 0.2,
              "box damping (s^-1)");

DEFINE_double(box_l, 0.12,
              "box length (m)");


DEFINE_double(box1_target_x, 0.045,
              "box1 target position (m)");

DEFINE_double(box1_target_v, 0.000116806,
              "box1 target velocity (m/s)");

DEFINE_double(box2_target_x, 0.284,
              "box2 target position (m)");

DEFINE_double(box2_target_v, 0.000349791,
              "box2 target velocity (m/s)");

/* this works with the following lines:

 extreme:  bazel run --config snopt //examples/box:trajectory_optimization_simulation -- --box1_target_x -0.2 --box2_target_x 0.5 --input_cost 1 --box1_force_limit 1000 --penalty_k 20 --box1_init_v 0.
 well:     bazel run --config snopt //examples/box:trajectory_optimization_simulation -- --box1_target_x 0.2 --box2_target_x 0.4 --input_cost 3. --box1_force_limit 1000 --penalty_k 30

 */

void StoreTwoBoxesEigenCSV(const std::string& filename, const VectorX<double>& times, const MatrixX<double>& data)
{
  /* csv format from  https://stackoverflow.com/questions/18400596/how-can-a-eigen-matrix-be-written-to-file-in-csv-format */
  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                  Eigen::DontAlignCols, ", ", "\n");
  std::ofstream file(filename);
  file << "t, box1x, box1v, box2x, box2v, box2u" << std::endl;
  /* horizontally concatenate times and data */
  MatrixX<double> OutMatrix(times.rows(), times.cols() + data.rows());
  OutMatrix << times, data.transpose(); /* can also do this with blocks */
  file << OutMatrix.format(CSVFormat);
  file.close();
}

void StoreDirColCSV(const std::string& filename, const VectorX<double>& times, const MatrixX<double>& inputs, const MatrixX<double>& states)
{
  /* csv format from  https://stackoverflow.com/questions/18400596/how-can-a-eigen-matrix-be-written-to-file-in-csv-format */
  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                  Eigen::DontAlignCols, ", ", "\n");
  std::ofstream file(filename);
  file << "t, box1xd, box1vd, box2xd, box2vd, box1ud" << std::endl;
  /* horizontally concatenate times and data */
  MatrixX<double> OutMatrix(times.rows(), times.cols() + states.rows() + inputs.rows());
  OutMatrix << times, states.transpose(), inputs.transpose(); /* can also do this with blocks */
  file << OutMatrix.format(CSVFormat);
  file.close();
}

#if 0
void StorePolyCSV(const std::string& filename, double t0, double tend, double dt)
{
  /* csv format from  https://stackoverflow.com/questions/18400596/how-can-a-eigen-matrix-be-written-to-file-in-csv-format */
  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                  Eigen::DontAlignCols, ", ", "\n");
  std::ofstream file(filename);
  file << "t, box1x, box1v, box2x, box2v, box2u" << std::endl;
  /* horizontally concatenate times and data */
  MatrixX<double> OutMatrix(times.rows(), times.cols() + data.rows());
  OutMatrix << times, data.transpose(); /* can also do this with blocks */
  file << OutMatrix.format(CSVFormat);
  file.close();
}
#endif

int DoMain() {
  auto two_boxes = std::make_unique<TwoBoxesPlant<double>>(FLAGS_box_m,
      FLAGS_box_d, FLAGS_box_l, FLAGS_penalty_k, FLAGS_penalty_d);
  two_boxes->set_name("twoboxes");
  auto context = two_boxes->CreateDefaultContext();

  const int kNumTimeSamples = 75;
  const double kMinimumTimeStep = 0.01;
  const double kMaximumTimeStep = 0.5;
  systems::trajectory_optimization::DirectCollocation dircol(two_boxes.get(),
      *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep,
      systems::InputPortSelection::kUseFirstInputIfItExists,
      true/* non cont states fixed */);
  
  dircol.AddEqualTimeIntervalsConstraints();

  const double kForceLimit = FLAGS_box1_force_limit;
  const solvers::VectorXDecisionVariable& u = dircol.input();
  const solvers::VectorXDecisionVariable& x = dircol.state();
  // for now, input is a 1D force vector on box 1
  dircol.AddConstraintToAllKnotPoints(-kForceLimit <= u(0));
  dircol.AddConstraintToAllKnotPoints(u(0) <= kForceLimit);
  // box 1 should be to the left of box 2
  dircol.AddConstraintToAllKnotPoints(x(0)<= x(2));
  drake::VectorX<double> initState1(2);
  initState1 << FLAGS_box1_init_x /* position */, FLAGS_box1_init_v /* velocity */;
  drake::VectorX<double> initState2(2);
  initState2 << FLAGS_box2_init_x /* position */, FLAGS_box2_init_v /* velocity */;
  two_boxes->SetBox1State(context.get(), initState1);
  two_boxes->SetBox2State(context.get(), initState2);

  drake::VectorX<double> finalState(4);
  finalState << FLAGS_box1_target_x /* b1 position */, FLAGS_box1_target_v /* b1 velocity */,
                FLAGS_box2_target_x /* b2 position */, FLAGS_box2_target_v /* b2 vel */ ; 


  drake::VectorX<double> initState = context->get_continuous_state_vector().CopyToVector();
  DRAKE_DEMAND(context->num_continuous_states() == 4);
  std::cout << initState << std::endl;
  dircol.AddLinearConstraint(dircol.initial_state() ==
                             initState);
  
  //dircol.AddLinearConstraint(dircol.final_state()  ==
  //                           finalState);      
  dircol.AddLinearConstraint(dircol.final_state()(0) == finalState(0));
  dircol.AddLinearConstraint(dircol.final_state()(1) == finalState(1));
  dircol.AddLinearConstraint(dircol.final_state()(2)  == finalState(2));
  dircol.AddLinearConstraint(dircol.final_state()(3)  == finalState(3));
  const double R = FLAGS_input_cost;  // Cost on input "effort"
  drake::VectorX<double> initU(1);
  initU << 0.;
  auto xd = x.block(0,0,4,1) - finalState.block(0,0,4,1);
  //unused(xd); //+ xd.cast<symbolic::Expression>().dot(xd)
  dircol.AddRunningCost(((R * u) * u)(0) + xd.cast<symbolic::Expression>().dot(xd));
  /* set these based on two boxes sim */
  double knot1t = 0.46184;
  double knot2t = 0.522649;
  double knot3t = 2.55;
  double knot4t = 6.;
  double endTime = 10.;
  drake::VectorX<double> knot1x(4);
  drake::VectorX<double> knot2x(4);
  drake::VectorX<double> knot3x(4);
  drake::VectorX<double> knot4x(4);
  knot1x << -0.0194361,	0.148709,	0.100003,	0.0024622;
  knot2x << -0.0147425,	0.0380937,	0.104335,	0.107608;
  knot3x << 0.0278335,	0.0106751,	0.231806,	0.0319679;
  knot4x << 0.0432708,	0.00131917,	0.278034,	0.00395042;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, knot1t, knot2t, knot3t, knot4t, endTime}, {initState, knot1x, knot2x, knot3x, knot4x, finalState});
  auto traj_init_u = PiecewisePolynomial<double>::FirstOrderHold(
      {0, endTime}, {initU, -initU});
  dircol.SetInitialTrajectory(traj_init_u, traj_init_x);
  const auto result = solvers::Solve(dircol);
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization for the trajectory"
              << std::endl;
    return 1;
  }

  systems::DiagramBuilder<double> finalBuilder;
  auto* boxes = finalBuilder.AddSystem(std::move(two_boxes));
  auto target1 = finalBuilder.AddSystem<BoxPlant>(0.0 /* mass */, FLAGS_box_l /* length */, 0.0 /* damping */);
  auto target2 = finalBuilder.AddSystem<BoxPlant>(0.0 /* mass */, FLAGS_box_l /* length */, 0.0 /* damping */);

  auto source1 = finalBuilder.AddSystem<systems::Sine>(4 * M_PI * M_PI * 1.0 * 0. /* amplitude */, 
                                                  2 * M_PI /* omega */, M_PI / 2.0 /* phase */, 1 /* vector size */);
  source1->set_name("source1");
  auto logger = LogOutput(boxes->get_log_output(), &finalBuilder);
  const PiecewisePolynomial<double> pp_traj =
      dircol.ReconstructInputTrajectory(result);
  const PiecewisePolynomial<double> px_traj =
      dircol.ReconstructStateTrajectory(result);
  Eigen::VectorXd times = dircol.GetSampleTimes(result);
  Eigen::MatrixXd input = dircol.GetInputSamples(result);
  Eigen::MatrixXd state = dircol.GetStateSamples(result);
  StoreDirColCSV("trajdesired.csv", times, input, state);
  
  auto input_trajectory = finalBuilder.AddSystem<systems::TrajectorySource>(pp_traj);
  input_trajectory->set_name("input trajectory");
  finalBuilder.Connect(input_trajectory->get_output_port(), boxes->get_input_port(0));
  // connect empty source to targets
  finalBuilder.Connect(source1->get_output_port(0), target1->get_input_port());
  finalBuilder.Connect(source1->get_output_port(0), target2->get_input_port());
  auto scene_graph = finalBuilder.AddSystem<geometry::SceneGraph>();
  // add targets last
  AddGeometryToBuilder(&finalBuilder, *boxes, scene_graph);
  BoxGeometry::AddToBuilder(&finalBuilder, *target1, scene_graph,"3");
  BoxGeometry::AddToBuilder(&finalBuilder, *target2, scene_graph,"4");
  ConnectDrakeVisualizer(&finalBuilder, *scene_graph);
  auto finalDiagram = finalBuilder.Build();

  //const double initial_energy = box1->CalcTotalEnergy(box1_context)+box2->CalcTotalEnergy(box2_context);
  systems::Simulator<double> simulator(*finalDiagram);
  systems::Context<double>& simContext = simulator.get_mutable_context();
  systems::Context<double>& boxesContext = finalDiagram->GetMutableSubsystemContext(*boxes, &simContext);
  systems::Context<double>& t1context = finalDiagram->GetMutableSubsystemContext(*target1,
                                          &simContext);
  systems::Context<double>& t2context = finalDiagram->GetMutableSubsystemContext(*target2,
                                          &simContext);
  
  boxes->SetBox1State(&boxesContext, initState1);
  boxes->SetBox2State(&boxesContext, initState2);
  drake::VectorX<double> targetState1(2);
  targetState1 << FLAGS_box1_target_x - FLAGS_box1_target_v * endTime, FLAGS_box1_target_v;
  drake::VectorX<double> targetState2(2);
  targetState2 << FLAGS_box2_target_x- FLAGS_box2_target_v * endTime, FLAGS_box2_target_v;
  target1->set_initial_state(&t1context, targetState1);
  target2->set_initial_state(&t2context, targetState2);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(pp_traj.end_time());
  StoreTwoBoxesEigenCSV("trajsimout.csv", logger->sample_times(), logger->data());


  return 0;
}

}  // namespace
}  // namespace box
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::box::DoMain();
}
