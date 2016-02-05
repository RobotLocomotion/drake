
#include "../Acrobot.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[])
{
  auto r = Acrobot();
  auto r_urdf = RigidBodySystem(getDrakePath() + "/examples/Acrobot/Acrobot.urdf", DrakeJoint::FIXED);
  auto r_sdf = RigidBodySystem(getDrakePath() + "/examples/Acrobot/Acrobot.sdf", DrakeJoint::FIXED);

  for (int i=0; i<1000; i++) {
    auto x0 = getRandomVector<AcrobotState>();
    auto u0 = getRandomVector<AcrobotInput>();

    RigidBodySystem::StateVector<double> x0_rb = toEigen(x0);
    RigidBodySystem::InputVector<double> u0_rb = toEigen(u0);

//    auto kinsol = tree->doKinematics(x0_rb.topRows(2),x0_rb.bottomRows(2));
//    cout << "H_rb = " << tree->massMatrix(kinsol) << endl;
//    eigen_aligned_unordered_map<const RigidBody *, Matrix<double, 6, 1> > f_ext;
//    cout << "C_rb = " << tree->dynamicsBiasTerm(kinsol,f_ext) << endl;

    auto xdot = toEigen(r.dynamics(0.0,x0,u0));
    auto xdot_urdf = r_urdf.dynamics(0.0, x0_rb, u0_rb);
//    cout << "xdot = " << xdot.transpose() << endl;
//    cout << "xdot_rb = " << xdot_rb.transpose() << endl;
    valuecheckMatrix(xdot_urdf,xdot,1e-8);

//    auto xdot_sdf = r_sdf.dynamics(0.0, x0_rb, u0_rb);
//    valuecheckMatrix(xdot_sdf,xdot,1e-8);
  }
}
