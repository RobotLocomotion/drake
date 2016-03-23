#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Drake;
using namespace Eigen;


Vector3d getAccelerometerOutput(shared_ptr<RigidBodySystem> const& sys, Vector3d const& rpy, Vector4d const& u) {
  VectorXd x0 = VectorXd::Zero(sys->getNumStates());
  auto const & tree = sys->getRigidBodyTree();
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  x0.segment(3, 4) = rpy2quat(rpy);
  auto const& system_output = sys->output(0, x0, u);
  return system_output.tail<3>();
}


int main(int argc, char* argv[]) {

  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;
  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(getDrakePath()+"/examples/Quadrotor/quadrotor.urdf", floating_base_type);
  auto const & tree = rigid_body_sys->getRigidBodyTree();
  auto sensor_frame = tree->findFrame("body");
  auto accelerometer = make_shared<RigidBodyAccelerometer>(*rigid_body_sys, "accelerometer", sensor_frame);
  rigid_body_sys->addSensor(accelerometer);
  const double g = 9.81;
  const double tol = 1e-6;
  const Vector4d hoverThrust = 1.226250 * Vector4d::Ones();

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d::Zero(), Vector4d::Zero()),
                   Vector3d(0, 0, -g),
                   tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d(M_PI, 0, 0), Vector4d::Zero()), // inverted
                   Vector3d(0, 0, g),
                   tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d::Zero(), hoverThrust),
                   Vector3d(0, 0, 0),
                   tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d(M_PI, 0, 0), hoverThrust), // inverted with thrust
                   Vector3d(0, 0, 2*g),
                   tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d(M_PI/2, 0, 0), Vector4d::Zero()),
                   Vector3d(0, -g, 0),
                   tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d(-M_PI/2, 0, 0), Vector4d::Zero()),
                   Vector3d(0, g, 0),
                   tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d(0, M_PI/2, 0), Vector4d::Zero()),
                   Vector3d(g, 0, 0),
                   tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d(0, -M_PI/2, 0), Vector4d::Zero()),
                   Vector3d(-g, 0, 0),
                   tol);

  //real accelerometers can't measure gravity during freefall
  //but will measure gravity while hovering
  accelerometer->setGravityCompensation(true);
  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d::Zero(), Vector4d::Zero()),
                 Vector3d(0, 0, 0),
                 tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d(M_PI, 0, 0), Vector4d::Zero()),
                   Vector3d(0, 0, 0),
                   tol);

  valuecheckMatrix(getAccelerometerOutput(rigid_body_sys, Vector3d::Zero(), hoverThrust),
                   Vector3d(0, 0, g),
                   tol);
}
