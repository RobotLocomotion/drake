#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

namespace drake {
namespace examples {
namespace manipulation_station {

std::vector<Eigen::VectorXd> SovleGaze(
    const multibody::MultibodyPlant<double>& plant,
    const multibody::Frame<double>& camera_frame,
    const std::vector<Eigen::Vector3d>& gaze_points_W, double max_cone_deg,
    double min_dist, double max_dist);

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
