#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/joints/floating_base_types.h"

namespace drake {
namespace systems {
namespace plants {
namespace {

using std::make_shared;
using std::shared_ptr;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

Vector3d getGyroscopeOutput(shared_ptr<RigidBodySystem> const& sys,
                            Vector3d const& ang_vel) {
  VectorXd x0 = VectorXd::Zero(sys->getNumStates());
  auto const& tree = sys->getRigidBodyTree();
  x0.head(tree->get_num_positions()) = tree->getZeroConfiguration();
  x0.segment<3>(7) = ang_vel;
  auto const& output = sys->output(0, x0, Vector4d::Zero());
  return output.segment<3>(13);
}

GTEST_TEST(testGyroscope, AllTests) {
  drake::systems::plants::joints::FloatingBaseType floating_base_type =
      drake::systems::plants::joints::kQuaternion;
  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->AddModelInstanceFromFile(
      GetDrakePath() + "/examples/Quadrotor/quadrotor.urdf",
      floating_base_type);
  auto const& tree = rigid_body_sys->getRigidBodyTree();
  auto sensor_frame = tree->findFrame("body");
  auto gyroscope = make_shared<RigidBodyGyroscope>(*rigid_body_sys, "gyroscope",
                                                   sensor_frame);
  rigid_body_sys->addSensor(gyroscope);
  const double tol = 1e-6;

  for (size_t i = 0; i < 100; i++) {
    Vector3d ang_vel = Vector3d::Random();

    EXPECT_TRUE(CompareMatrices(getGyroscopeOutput(rigid_body_sys, ang_vel),
                                ang_vel, tol, MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace plants
}  // namespace systems
}  // namespace drake
