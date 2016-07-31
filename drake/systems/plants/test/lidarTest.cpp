#include <cmath>

#include "drake/common/drake_path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/testUtil.h"
#include "lcmtypes/bot_core/planar_lidar_t.hpp"

using Eigen::VectorXd;
using std::make_shared;

namespace drake {
namespace {

int do_main(int argc, char* argv[]) {
  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(
      GetDrakePath() + "/systems/plants/test/lidarTest.sdf", DrakeJoint::FIXED);

  // Verifies that the RigidBodyDepthSensor accessors return the correct values.
  auto sensors = rigid_body_sys->GetSensors();
  if (sensors.size() != 1) {
    std::stringstream error_msg;
    error_msg << "ERROR: Unexpected number of sensors! Got " << sensors.size()
              << " expected 1.";
    throw std::runtime_error(error_msg.str());
  }

  auto lidar_sensor =
      dynamic_cast<const drake::RigidBodyDepthSensor*>(sensors[0]);
  if (lidar_sensor == nullptr) {
    throw std::runtime_error(
        "ERROR: Unable to obtain pointer to the LIDAR sensor.");
  }

  if (lidar_sensor->get_model_name() != "room_w_lidar") {
    throw std::runtime_error(
      "ERROR: LIDAR scanner reported that it is part of the wrong model!");
  }

  if (!lidar_sensor->is_horizontal_scanner()) {
    throw std::runtime_error(
      "ERROR: LIDAR scanner says it is not horizontal but it is!");
  }

  if (lidar_sensor->is_vertical_scanner()) {
    throw std::runtime_error(
      "ERROR: LIDAR scanner says it is vertical but is not!");
  }

  valuecheck(static_cast<size_t>(1), lidar_sensor->num_pixel_rows(),
             std::string("Problems with num_pixel_rows(). "));
  valuecheck(static_cast<size_t>(640), lidar_sensor->num_pixel_cols(),
             std::string("Problems with num_pixel_cols(). "));

  valuecheck(0, lidar_sensor->min_pitch(),
             std::numeric_limits<double>::epsilon());

  valuecheck(0, lidar_sensor->max_pitch(),
             std::numeric_limits<double>::epsilon());

  valuecheck(-1.7, lidar_sensor->min_yaw(),
             std::numeric_limits<double>::epsilon());

  valuecheck(1.7, lidar_sensor->max_yaw(),
             std::numeric_limits<double>::epsilon());

  valuecheck(0.08, lidar_sensor->min_range(),
             std::numeric_limits<double>::epsilon());

  valuecheck(25, lidar_sensor->max_range(),
             std::numeric_limits<double>::epsilon());

  double t = 0;
  VectorXd x = VectorXd::Zero(rigid_body_sys->getNumStates());
  VectorXd u = VectorXd::Zero(rigid_body_sys->getNumInputs());

  //  rigid_body_sys->getRigidBodyTree()->drawKinematicTree("/tmp/lidar.dot");

  auto distances = rigid_body_sys->output(t, x, u);

  valuecheck(640, static_cast<int>(distances.size()),
             std::string("Problem with number of distances. "));

  const double min_yaw = lidar_sensor->min_yaw();
  const double max_yaw = lidar_sensor->max_yaw();
  const double max_range = lidar_sensor->max_range();
  const double tol = 1.0e-6;

  for (int i = 0; i < distances.size(); i++) {
    double theta = min_yaw + (max_yaw - min_yaw) * i / (distances.size() - 1);

    // We've implicitly hard-coded the box geometry from the SDF for the
    // computation of the analytical distances.
    if (std::abs(theta) >=
        M_PI / 2.0 + tol) {  // Should not be hitting any wall.
      valuecheck(max_range, distances(i));
    } else if (theta <= -M_PI / 4) {  // hitting the right wall
      valuecheck(-1.0 / std::sin(theta), distances(i), tol);
    } else if (theta >= M_PI / 4) {  // hitting the left wall
      valuecheck(1.0 / std::sin(theta), distances(i), tol);
    } else {  // hitting the front wall
      valuecheck(1 / std::cos(theta), distances(i), tol);
    }
  }

  // Tells Drake Visualizer to render the environment for debugging purposes.
  // This needs to be done before publishing LIDAR data so the visualizer knows
  // the sensor's position.
  auto lcm = make_shared<lcm::LCM>();
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(
      lcm, rigid_body_sys->getRigidBodyTree());
  visualizer->output(t, x, u);

  // Publishes the LIDAR message.
  bot_core::planar_lidar_t msg;
  msg.utime = -1;
  msg.nintensities = 0;
  msg.rad0 = min_yaw;
  msg.radstep = (max_yaw - min_yaw) / (distances.size() - 1);

  msg.nranges = distances.size();
  for (int i = 0; i < distances.size(); i++) msg.ranges.push_back(distances(i));

  lcm->publish("DRAKE_PLANAR_LIDAR_0_rear_wall", &msg);

  return 0;
}

}  // namespace
}  // namespace drake

/**
 * This unit test sets up a LIDAR in a box room and verifies the returns.
 */
int main(int argc, char* argv[]) {
  return drake::do_main(argc, argv);
}
