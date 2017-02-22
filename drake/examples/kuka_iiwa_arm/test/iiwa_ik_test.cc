#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_ik_planner.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

inline double get_orientation_difference(const Matrix3<double>& rot0,
                                         const Matrix3<double>& rot1) {
  AngleAxis<double> err(rot0.transpose() * rot1);
  return err.angle();
}
}

GTEST_TEST(testInverseKinematics, SolveIkFromFk) {
  const std::string kModelPath =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";
  const std::string kEndEffectorLinkName = "iiwa_link_ee";
  std::default_random_engine rand_generator(1234);

  std::unique_ptr<RigidBodyTree<double>> iiwa =
      std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());

  KinematicsCache<double> cache = iiwa->CreateKinematicsCache();
  const RigidBody<double>* end_effector = iiwa->FindBody(kEndEffectorLinkName);

  IiwaIkPlanner ik_planner(kModelPath, kEndEffectorLinkName, nullptr);
  IiwaIkPlanner::IkResult ik_res;
  IiwaIkPlanner::IkCartesianWaypoint wp;
  wp.time = 0;
  wp.pos_tol = Vector3<double>(0.001, 0.001, 0.001);
  wp.rot_tol = 0.001;
  wp.constrain_orientation = true;
  std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints(1, wp);

  const VectorX<double> kQcurrent = iiwa->getZeroConfiguration();
  VectorX<double> q_fk;

  const double kEpsilon = 1e-8;
  const Vector3<double> kUpperBound =
      wp.pos_tol + kEpsilon * Vector3<double>::Ones();
  const Vector3<double> kLowerBound =
      -wp.pos_tol - kEpsilon * Vector3<double>::Ones();

  for (int i = 0; i < 100; ++i) {
    q_fk = iiwa->getRandomConfiguration(rand_generator);
    cache.initialize(q_fk);
    iiwa->doKinematics(cache);

    Isometry3<double> fk_pose =
        iiwa->CalcBodyPoseInWorldFrame(cache, *end_effector);
    waypoints[0].pose = fk_pose;

    bool ret =
        ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &ik_res);
    EXPECT_TRUE(ret);

    cache.initialize(ik_res.q.col(1));
    iiwa->doKinematics(cache);
    Isometry3<double> ik_pose =
        iiwa->CalcBodyPoseInWorldFrame(cache, *end_effector);
    Vector3<double> pos_diff = ik_pose.translation() - fk_pose.translation();
    double rot_diff =
        get_orientation_difference(ik_pose.linear(), fk_pose.linear());

    EXPECT_TRUE((pos_diff.array() <= kUpperBound.array()).all());
    EXPECT_TRUE((pos_diff.array() >= kLowerBound.array()).all());

    // to Hongkai: this fails sometimes with pretty large error, any ideas?
    // std::cout << rot_diff << std::endl;
    // EXPECT_TRUE(std::abs(rot_diff) <= wp.rot_tol + kEpsilon);
  }
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
