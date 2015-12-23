
#include "../Pendulum.h"
#include "RigidBodySystem.h"

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

    assert((rbsys.dynamics(0.0,x0_rb,u0_rb) - toEigen(p.dynamics(0.0,x0,u0))).isZero());
  }
}
