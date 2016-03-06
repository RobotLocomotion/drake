
#include "../Acrobot.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  auto r = Acrobot();
  auto r_urdf = RigidBodySystem(
      getDrakePath() + "/examples/Acrobot/Acrobot.urdf", DrakeJoint::FIXED);
  auto r_sdf = RigidBodySystem(getDrakePath() + "/examples/Acrobot/Acrobot.sdf",
                               DrakeJoint::FIXED);

  // for debugging:
  /*
  r_urdf.getRigidBodyTree()->drawKinematicTree("/tmp/urdf.dot");
  r_sdf.getRigidBodyTree()->drawKinematicTree("/tmp/sdf.dot");
  */
  // I ran this at the console to see the output:
  // dot -Tpng -O /tmp/urdf.dot; dot -Tpng -O /tmp/sdf.dot; open /tmp/*.dot.png

  for (int i = 0; i < 1000; i++) {
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
    r_urdf.getRigidBodyTree()->dynamicsBiasTerm(kinsol_urdf,f_ext) << endl;
    cout << "C_sdf = " <<
    r_sdf.getRigidBodyTree()->dynamicsBiasTerm(kinsol_sdf,f_ext) << endl;
    */

    auto xdot = toEigen(r.dynamics(0.0, x0, u0));
    auto xdot_urdf = r_urdf.dynamics(0.0, x0_rb, u0_rb);
    //    cout << "xdot = " << xdot.transpose() << endl;
    //    cout << "xdot_rb = " << xdot_rb.transpose() << endl;
    valuecheckMatrix(xdot_urdf, xdot, 1e-8);

    auto xdot_sdf = r_sdf.dynamics(0.0, x0_rb, u0_rb);
    valuecheckMatrix(xdot_sdf, xdot, 1e-8);
  }
}
