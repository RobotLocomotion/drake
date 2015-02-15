

#include <iostream>
#include <Eigen/Dense>
#include "URDFRigidBodyManipulator.h"

using namespace std;

int main(int argc, char* argv[])
{
	if (argc<2) {
		cerr << "Usage: urdf_kin_test urdf_filename" << endl;
		exit(-1);
	}
  URDFRigidBodyManipulator* model = loadURDFfromFile(argv[1]);

  for (vector<unique_ptr<RigidBody> >::const_iterator it = model->bodies.begin(); it != model->bodies.end(); ++it) {
    RigidBody& body = **it;
    double mass = 1.0;
    Matrix3d moment_of_inertia = Matrix3d::Random();
    moment_of_inertia *= moment_of_inertia;
    Vector3d com = Vector3d::Random();

    body.I.topLeftCorner<3, 3>() = moment_of_inertia;
    body.I.bottomRightCorner<3, 3>() = mass * Matrix3d::Identity();
    body.I.topRightCorner<3, 3>() = vectorToSkewSymmetric((mass * com).eval());
    body.I.bottomLeftCorner<3, 3>() = vectorToSkewSymmetric((-mass * com).eval());
  }


//  URDFRigidBodyManipulator* model = loadURDFfromFile("/Users/russt/drc/software/models/mit_gazebo_models/mit_robot/model.urdf");
  if (!model) {
  	cerr << "ERROR: Failed to load model from " << argv[1] << endl;
  	return -1;
  }

  // run kinematics with second derivatives 100 times
  VectorXd q = VectorXd::Zero(model->num_dof);
	VectorXd v = VectorXd::Zero(model->num_velocities);
	int i;

  if (argc>=2+model->num_dof) {
  	for (i=0; i<model->num_dof; i++)
  		sscanf(argv[2+i],"%lf",&q(i));
  }

	if (argc>=2+model->num_dof+model->num_velocities) {
		for (i=0; i<model->num_velocities; i++)
			sscanf(argv[2+model->num_dof+i],"%lf",&v(i));
  }

  model->doKinematicsNew(q.data(), true, v.data(), true);

  auto H = model->massMatrix<double>();

	cout << H.value() << endl;

  delete model;
  return 0;
}
