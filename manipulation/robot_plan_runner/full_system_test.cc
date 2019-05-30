
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"
#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace {

using manipulation::robot_plan_runner::robot_plans::PlanData;
using manipulation::robot_plan_runner::robot_plans::PlanType;
using math::RollPitchYawd;
using std::cout;
using std::endl;
using std::vector;

int test_joint_space_plan() {
  // create plan
  PlanData plan1;

  Eigen::VectorXd t_knots(3);
  t_knots << 0, 2, 4;

  Eigen::MatrixXd q_knots(7, 3);
  q_knots.col(0) << 0, 0, 0, -1.75, 0, 1.0, 0;
  q_knots.col(1) << 0.5, 0, 0, -1.75, 0, 1.0, 0;
  q_knots.col(2) << 1, 0, 0, -1.75, 0, 1.0, 0;

  auto qtraj = trajectories::PiecewisePolynomial<double>::Cubic(
      t_knots, q_knots, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));

  plan1.plan_type = PlanType::kJointSpacePlan;
  plan1.joint_traj = qtraj;
  std::vector<PlanData> plan_list{plan1};

  // Construct plan runner hardware interface.
  auto plan_runner =
      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list);

  // save diagram graphviz string.
  plan_runner.SaveGraphvizStringToFile();

  // Run simulation.
  plan_runner.Run();

  return 0;
};

/*
 * This is not as much a test as it is an example of using
 * PlanRunnerHardwareInterface. To see what it does to robots, it should be run
 * together with //examples/manipulation_station:mock_station_simulation
 */
//int test_task_space_plan() {
//  PlanData plan1;
//  plan1.plan_type = PlanType::kTaskSpacePlan;
//
//  PlanData::EeData ee_data;
//  ee_data.p_ToQ_T.setZero();
//
//  Eigen::VectorXd t_knots(3);
//  t_knots << 0, 2, 4;
//
//  Eigen::MatrixXd xyz_knots(3, 3);
//  xyz_knots.col(0) << 0, 0, 0;
//  xyz_knots.col(1) << 0, -0.1, 0;
//  xyz_knots.col(2) << 0, -0.2, 0;
//
//  ee_data.ee_xyz_traj  = trajectories::PiecewisePolynomial<double>::Cubic(
//      t_knots, xyz_knots, Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3));
//
//  auto Q_WT = RollPitchYawd(0, 0.6 + 1.75 + 1, 0).ToQuaternion();
//
//  vector<double> t_knots_v{0, 2, 4};
//  vector<Eigen::Quaterniond> quaternions{Q_WT, Q_WT, Q_WT};
//
//  ee_data.ee_quat_traj =
//      trajectories::PiecewiseQuaternionSlerp<double>(t_knots_v, quaternions);
//
//  plan1.ee_data = ee_data;
//  vector<PlanData> plan_list{plan1};
//
//  // Construct plan runner hardware interface.
//  auto plan_runner =
//      manipulation::robot_plan_runner::PlanRunnerHardwareInterface(plan_list);
//
//  // save diagram graphviz string.
//  plan_runner.SaveGraphvizStringToFile();
//
//  // Run simulation.
//  plan_runner.Run();
//
//  return 0;
//
//};

}  // namespace
}  // namespace drake

int main() { return drake::test_joint_space_plan(); };