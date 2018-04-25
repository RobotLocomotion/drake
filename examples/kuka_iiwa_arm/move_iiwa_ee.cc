/// @file
///
/// Demo of moving the iiwa's end effector in cartesian space.  This
/// program creates a plan to move the end effector from the current
/// position to the location specified on the command line.  The
/// current calculated position of the end effector is printed before,
/// during, and after the commanded motion.

#include <gflags/gflags.h>
#include "lcm/lcm-cpp.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to send robot_plan_t messages.");
DEFINE_double(x, 0., "x coordinate to move to");
DEFINE_double(y, 0., "y coordinate to move to");
DEFINE_double(z, 0., "z coordinate to move to");
DEFINE_double(roll, 0., "target roll about world x axis for end effector");
DEFINE_double(pitch, 0., "target pitch about world y axis for end effector");
DEFINE_double(yaw, 0., "target yaw about world z axis for end effector");
DEFINE_string(ee_name, "iiwa_link_ee", "Name of the end effector link");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using manipulation::planner::ConstraintRelaxingIk;

const char kIiwaUrdf[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

class MoveDemoRunner {
 public:
  MoveDemoRunner() {
    urdf_ =
        (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf));
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf_, multibody::joints::kFixed, &tree_);

    lcm_.subscribe(FLAGS_lcm_status_channel,
                   &MoveDemoRunner::HandleStatus, this);
  }

  void Run() {
    while (lcm_.handle() >= 0) { }
  }

 private:
  // Handle the incoming status message from the iiwa.  This is a
  // fairly high rate operation (200Hz is typical).  The plan is
  // calculated on the first status message received, after that
  // periodically display the current position of the end effector.
  void HandleStatus(const lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    status_count_++;
    Eigen::VectorXd iiwa_q(status->num_joints);
    Eigen::VectorXd iiwa_v(status->num_joints);
    for (int i = 0; i < status->num_joints; i++) {
      iiwa_v[i] = status->joint_velocity_estimated[i];
      iiwa_q[i] = status->joint_position_measured[i];
    }

    // Only print the position every 100 messages (this results in an
    // 0.5s period in a typical configuration).
    if (status_count_ % 100 == 1) {
      // Estimate the end effector position through forward kinematics.
      KinematicsCache<double> cache = tree_.doKinematics(iiwa_q, iiwa_v, true);
      const RigidBody<double>* end_effector = tree_.FindBody(FLAGS_ee_name);

      Isometry3<double> ee_pose =
          tree_.CalcBodyPoseInWorldFrame(cache, *end_effector);
      drake::log()->info("End effector at: {} {}",
                         ee_pose.translation().transpose(),
                         math::rotmat2rpy(ee_pose.rotation()).transpose());
    }

    // If this is the first status we've received, calculate a plan
    // and send it (if it succeeds).
    if (status_count_ == 1) {
      ConstraintRelaxingIk ik(
          urdf_, FLAGS_ee_name, Isometry3<double>::Identity());

      // Create a single waypoint for our plan (the destination).
      // This results in a trajectory with two knot points (the
      // current pose (read from the status message currently being
      // processes and passed directly to PlanSequentialTrajectory as
      // iiwa_q) and the calculated final pose).
      ConstraintRelaxingIk::IkCartesianWaypoint wp;
      wp.pose.translation() = Eigen::Vector3d(FLAGS_x, FLAGS_y, FLAGS_z);
      Eigen::Vector3d rpy(FLAGS_roll, FLAGS_pitch, FLAGS_yaw);
      wp.pose.linear() = math::rpy2rotmat(rpy);
      wp.constrain_orientation = true;
      std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
      waypoints.push_back(wp);
      IKResults ik_res;
      ik.PlanSequentialTrajectory(waypoints, iiwa_q, &ik_res);
      drake::log()->info("IK result: {}", ik_res.info[0]);

      if (ik_res.info[0] == 1) {
        drake::log()->info("IK sol size {}", ik_res.q_sol.size());

        // Run the resulting plan over 2 seconds (which is a fairly
        // arbitrary choice).  This may be slowed down if executing
        // the plan in that time would exceed any joint velocity
        // limits.
        std::vector<double> times{0, 2};
        DRAKE_DEMAND(ik_res.q_sol.size() == times.size());

        // Convert the resulting waypoints into the format expected by
        // ApplyJointVelocityLimits and EncodeKeyFrames.
        MatrixX<double> q_mat(ik_res.q_sol.front().size(), ik_res.q_sol.size());
        for (size_t i = 0; i < ik_res.q_sol.size(); ++i) {
          q_mat.col(i) = ik_res.q_sol[i];
        }

        // Ensure that the planned motion would not exceed the joint
        // velocity limits.  This function will scale the supplied
        // times if needed.
        ApplyJointVelocityLimits(q_mat, &times);
        robotlocomotion::robot_plan_t plan =
            EncodeKeyFrames(tree_, times, ik_res.info, q_mat);
        lcm_.publish(FLAGS_lcm_plan_channel, &plan);
      }
    }
  }

  lcm::LCM lcm_;
  std::string urdf_;
  RigidBodyTree<double> tree_;
  int status_count_{0};
};

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  drake::examples::kuka_iiwa_arm::MoveDemoRunner runner;
  runner.Run();
}
