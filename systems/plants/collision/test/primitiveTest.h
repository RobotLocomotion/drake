#include "DrakeCollision.h"
#include "../simpleClosestPointFunctions.h"
#include <cmath>

Eigen::MatrixXd numerical_gradient(double (*dist_fun)(Eigen::VectorXd), Eigen::VectorXd x, int order = 2)
{
  // evaluate the numerical gradient of distance function dist_fun at value x. If order = 1, we use first-order approximation, if order = 2, use second order approximation.
  int nx = x.size();
  double y = (*dist_fun)(x);
  int ny = 1;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(ny,nx);
  for(int i = 0;i<nx;i++)
  {
    Eigen::VectorXd ei = Eigen::VectorXd::Zero(nx);
    ei(i) = 1e-10;
    double yi_plus = (*dist_fun)(x+ei);
    if(order == 1)
    {
      J(i) = (yi_plus-y)/ei(i);
    }
    else if(order == 2)
    {
      double yi_minus = (*dist_fun)(x-ei);
      J(i) = (yi_plus-yi_minus)/(2*ei(i)); 
    }
  }
  return J;
};

double pointPointDist_userfun(Eigen::VectorXd x)
{
  Eigen::Vector3d y11 = x.block(0,0,3,1);
  Eigen::Vector3d y21 = x.block(3,0,3,1);
  return pointPointDist(y11,y21);
}

double pointLineDist_userfun(Eigen::VectorXd x)
{
  Eigen::Vector3d y11 = x.block(0,0,3,1);
  Eigen::Vector3d y21 = x.block(3,0,3,1);
  Eigen::Vector3d y22 = x.block(6,0,3,1);
  return pointLineDist(y11,y21,y22);
}

double lineLineDist_userfun(Eigen::VectorXd x)
{
  Eigen::VectorXd y11 = x.block(0,0,3,1);
  Eigen::Vector3d y12 = x.block(3,0,3,1);
  Eigen::Vector3d y21 = x.block(6,0,3,1);
  Eigen::Vector3d y22 = x.block(9,0,3,1);
  return lineLineDist(y11,y12,y21,y22);
}

double pointPlaneDist_userfun(Eigen::VectorXd x)
{
  Eigen::Vector3d y11 = x.block(0,0,3,0);
  Eigen::Vector3d y21 = x.block(3,0,3,0);
  Eigen::Vector3d y22 = x.block(6,0,3,0);
  Eigen::Vector3d y23 = x.block(9,0,3,0);
  return pointPlaneDist(y11,y21,y22,y23);
}
