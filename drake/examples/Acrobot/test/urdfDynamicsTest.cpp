#include "drake/common/drake_path.h"
#include "drake/examples/Acrobot/Acrobot.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/eigen_matrix_compare.h"
#include "gtest/gtest.h"

using drake::RigidBodySystem;
using drake::getRandomVector;
using drake::GetDrakePath;
using drake::util::MatrixCompareType;

namespace drake {
namespace examples {
namespace acrobot {
namespace {

// Tests whether the dynamics of Acrobot are the same regardless of whether
// it is loaded via direct Acrobot object instantiation, URDF, or SDF. This
// is done by loading random state and input values into the models and
// verifying
// that their dynamics are identical.
GTEST_TEST(AcrobotDynamicsTest, ValueAssignment) {
  // Create three Acrobot models
  auto r = Acrobot();

  auto r_urdf = RigidBodySystem();
  r_urdf.addRobotFromFile(GetDrakePath() + "/examples/Acrobot/Acrobot.urdf",
                          DrakeJoint::FIXED);

  auto r_sdf = RigidBodySystem();
  r_sdf.addRobotFromFile(GetDrakePath() + "/examples/Acrobot/Acrobot.sdf",
                         DrakeJoint::FIXED);

  // for debugging:
  /*
  r_urdf.getRigidBodyTree()->drawKinematicTree("/tmp/urdf.dot");
  r_sdf.getRigidBodyTree()->drawKinematicTree("/tmp/sdf.dot");
  */
  // I ran this at the console to see the output:
  // dot -Tpng -O /tmp/urdf.dot; dot -Tpng -O /tmp/sdf.dot; open /tmp/*.dot.png

  // Iterate 1000 times each time sending in random state and input variables
  // and verifying that the resulting dynamics are the same.
  for (int ii = 0; ii < 1000; ii++) {
    auto x0 = getRandomVector<AcrobotState>();
    auto u0 = getRandomVector<AcrobotInput>();

    RigidBodySystem::StateVector<double> x0_rb = toEigen(x0);
    RigidBodySystem::InputVector<double> u0_rb = toEigen(u0);

    /*
    auto kinsol_urdf = r_urdf.getRigidBodyTree()->doKinematics(x0_rb.topRows(2),
    x0_rb.bottomRows(2));
    cout << "H_urdf = " << r_urdf.getRigidBodyTree()->massMatrix(kinsol_urdf) <<
    endl;
    auto kinsol_sdf = r_sdf.getRigidBodyTree()->doKinematics(x0_rb.topRows(2),
    x0_rb.bottomRows(2));
    cout << "H_sdf = " << r_sdf.getRigidBodyTree()->massMatrix(kinsol_sdf) <<
    endl;
    eigen_aligned_unordered_map<const RigidBody *, Matrix<double, 6, 1> > f_ext;
    cout << "C_urdf = " <<
    r_urdf.getRigidBodyTree()->dynamicsBiasTerm(kinsol_urdf, f_ext) << endl;
    cout << "C_sdf = " <<
    r_sdf.getRigidBodyTree()->dynamicsBiasTerm(kinsol_sdf, f_ext) << endl;
    */

    auto xdot = toEigen(r.dynamics(0.0, x0, u0));

    auto xdot_urdf = r_urdf.dynamics(0.0, x0_rb, u0_rb);
    EXPECT_TRUE(
        CompareMatrices(xdot_urdf, xdot, 1e-8, MatrixCompareType::absolute));

    auto xdot_sdf = r_sdf.dynamics(0.0, x0_rb, u0_rb);
    EXPECT_TRUE(
        CompareMatrices(xdot_sdf, xdot, 1e-8, MatrixCompareType::absolute));
  }
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake
