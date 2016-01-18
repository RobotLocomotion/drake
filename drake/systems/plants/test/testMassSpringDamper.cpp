
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[])
{
  RigidBodySystem sys(getDrakePath()+"/systems/plants/test/MassSpringDamper.urdf",DrakeJoint::FIXED);

  double mass = 1.0, k = 10.0, b = 1.0;
  Matrix<double,2,1> xdot_desired;

  for (int i=0; i<1000; i++) {
    auto x0 = getInitialState(sys);
    Matrix<double,1,1> u0 = Matrix<double,1,1>::Random();

    auto xdot = toEigen(sys.dynamics(0.0,x0,u0));
    xdot_desired << x0(1), (u0(0) -k*x0(0) - b*x0(1))/mass;

    valuecheckMatrix(xdot_desired,xdot,1e-5);
  }
}
