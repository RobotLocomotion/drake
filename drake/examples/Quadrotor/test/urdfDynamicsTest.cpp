
#include "../Quadrotor.h"
#include "RigidBodySystem.h"
#include "testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[])
{
  auto rbsys = RigidBodySystem(getDrakePath()+"/examples/Quadrotor/quadrotor.urdf",DrakeJoint::ROLLPITCHYAW);
  auto p = Quadrotor();

  for (int i=0; i<1000; i++) {
    auto x0 = getRandomVector<QuadrotorState>();
    auto u0 = QuadrotorInput<double>(Vector4d::Zero()); //getRandomVector<QuadrotorInput>();

    RigidBodySystem::StateVector<double> x0_rb = toEigen(x0);
    RigidBodySystem::InputVector<double> u0_rb = toEigen(u0);

    auto xdot = toEigen(p.dynamics(0.0,x0,u0));
    auto xdot_rb = rbsys.dynamics(0.0,x0_rb,u0_rb);
//    cout << "xdot = " << xdot.transpose() << endl;
//    cout << "xdot_rb = " << xdot_rb.transpose() << endl;
    valuecheckMatrix(xdot_rb,xdot,1e-8);
  }
}
