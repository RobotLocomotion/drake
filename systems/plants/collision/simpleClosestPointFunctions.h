#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using namespace Eigen;

double pointPointDist(const Vector3d y11, const Vector3d y21);
double pointLineDist(Vector3d y11, Vector3d y21, Vector3d y22);
double lineLineDist(Vector3d y11, Vector3d y12, Vector3d y21, Vector3d y22);
double pointPlaneDist(Vector3d y11, Vector3d y21, Vector3d y22, Vector3d y23);
MatrixXd pointPointDistJac(const Vector3d y11, const Vector3d y21);
MatrixXd pointLineDistJac(Vector3d y11, Vector3d y21, Vector3d y22);
MatrixXd lineLineDistJac(Vector3d y11, Vector3d y12, Vector3d y21, Vector3d y22);
MatrixXd pointPlaneDistJac(Vector3d y11, Vector3d y21, Vector3d y22, Vector3d y23);
