#include "drake/examples/manipulation_station/pdc_ik.h"

#include "drake/solvers/solve.h"

namespace drake {
namespace examples {
namespace manipulation_station {

std::vector<Eigen::VectorXd> SovleGaze(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::Frame<double>& camera_frame,
    const std::vector<Eigen::Vector3d>& gaze_points_W, double max_cone_deg,
    double min_dist, double max_dist) {
  const auto iiwa = plant.GetModelInstanceByName("iiwa");

  std::vector<Eigen::VectorXd> results;
  std::vector<solvers::Binding<solvers::Constraint>> constraints;

  for (const Eigen::Vector3d& pt_W : gaze_points_W) {
    multibody::InverseKinematics ik(plant);

    solvers::Binding<solvers::Constraint> gaze_con = ik.AddGazeTargetConstraint(
        camera_frame, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
        plant.world_frame(), pt_W, max_cone_deg * M_PI / 180.);
    constraints.push_back(gaze_con);

    solvers::Binding<solvers::Constraint> pos_con = ik.AddPositionConstraint(
        plant.world_frame(), pt_W, camera_frame,
        Eigen::Vector3d(-std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity(), min_dist),
        Eigen::Vector3d(std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity(), max_dist));
    constraints.push_back(pos_con);

    const auto result = Solve(ik.prog());
    drake::log()->info("Solve: {}", result.is_success());
    if (result.is_success()) {
      const auto q_full_sol = result.GetSolution(ik.q());
      const auto q_iiwa = plant.GetPositionsFromArray(iiwa, q_full_sol);
      results.push_back(q_iiwa);
    }
  }

  return results;
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
