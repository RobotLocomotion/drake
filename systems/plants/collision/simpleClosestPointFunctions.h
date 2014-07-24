#include <Eigen/Dense>

double pointPointDist(const Eigen::Vector3d y11, const Eigen::Vector3d y21);
double pointLineDist(Eigen::Vector3d y11, Eigen::Vector3d y21, Eigen::Vector3d y22);
double lineLineDist(Eigen::Vector3d y11, Eigen::Vector3d y12, Eigen::Vector3d y21, Eigen::Vector3d y22);
double pointPlaneDist(Eigen::Vector3d y11, Eigen::Vector3d y21, Eigen::Vector3d y22, Eigen::Vector3d y23);
Eigen::MatrixXd pointPointDistJac(const Eigen::Vector3d y11, const Eigen::Vector3d y21);
Eigen::MatrixXd pointLineDistJac(Eigen::Vector3d y11, Eigen::Vector3d y21, Eigen::Vector3d y22);
Eigen::MatrixXd lineLineDistJac(Eigen::Vector3d y11, Eigen::Vector3d y12, Eigen::Vector3d y21, Eigen::Vector3d y22);
Eigen::MatrixXd pointPlaneDistJac(Eigen::Vector3d y11, Eigen::Vector3d y21, Eigen::Vector3d y22, Eigen::Vector3d y23);
