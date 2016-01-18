
#include "../Pendulum.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[])
{
  auto tree = make_shared<RigidBodyTree>(getDrakePath()+"/examples/Pendulum/Pendulum.urdf",DrakeJoint::FIXED);
  auto rbsys = RigidBodySystem(tree);
  auto p = Pendulum();

  for (int i=0; i<1000; i++) {
    auto x0 = getRandomVector<PendulumState>();
    auto u0 = getRandomVector<PendulumInput>();

    RigidBodySystem::StateVector<double> x0_rb = toEigen(x0);
    RigidBodySystem::InputVector<double> u0_rb = toEigen(u0);

    auto xdot = toEigen(p.dynamics(0.0,x0,u0));
    auto xdot_rb = rbsys.dynamics(0.0,x0_rb,u0_rb);
//    cout << "xdot = " << xdot.transpose() << endl;
//    cout << "xdot_rb = " << xdot_rb.transpose() << endl;
    valuecheckMatrix(xdot_rb,xdot,1e-8);
  }
}
