#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/testUtil.h"

using namespace std;
using namespace Drake;
using namespace Eigen;


Vector3d getMagnetometerOutput(shared_ptr<RigidBodySystem> const& sys, Vector3d const& rpy) {
  VectorXd x0 = VectorXd::Zero(sys->getNumStates());
  auto const & tree = sys->getRigidBodyTree();
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  x0.segment(3, 4) = rpy2quat(rpy);
  auto const& system_output = sys->output(0, x0, Vector4d::Zero());
  return system_output.tail<3>();
}


int main(int argc, char* argv[]) {  

  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;
  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(getDrakePath()+"/examples/Quadrotor/quadrotor.urdf", floating_base_type);
  
  auto const & tree = rigid_body_sys->getRigidBodyTree();
  auto sensor_frame = tree->findFrame("body");
  
  auto magnetometer = make_shared<RigidBodyMagnetometer>(*rigid_body_sys, "magnetometer", sensor_frame, 0.0);
  rigid_body_sys->addSensor(magnetometer);
  
  const double tol = 1e-6;

  valuecheckMatrix(getMagnetometerOutput(rigid_body_sys, Vector3d::Zero()),
                   Vector3d(1, 0, 0),
                   tol);


  valuecheckMatrix(getMagnetometerOutput(rigid_body_sys, Vector3d(0,0,M_PI/2)),
                 Vector3d(0, -1, 0),
                 tol);

  magnetometer->setDeclination(M_PI/4);

  valuecheckMatrix(getMagnetometerOutput(rigid_body_sys, Vector3d::Zero()),
                 Vector3d(std::sqrt(2)/2, std::sqrt(2)/2, 0),
                 tol);

  valuecheckMatrix(getMagnetometerOutput(rigid_body_sys,  Vector3d(0,0,M_PI/2)),
                 Vector3d(std::sqrt(2)/2, -std::sqrt(2)/2, 0),
                 tol);

}