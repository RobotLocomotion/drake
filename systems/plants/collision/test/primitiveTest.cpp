#define BOOST_TEST_MODULE Primitive test
#include <boost/test/unit_test.hpp>
#include "primitiveTest.h"

using namespace std;
using namespace Eigen;

BOOST_AUTO_TEST_CASE(pointPointTest)
{
  for(int i = 0;i<100;i++)
  {
    Vector3d y11 = Vector3d::Random();
    Vector3d y21 = Vector3d::Random();
    VectorXd x = VectorXd::Zero(6);
    x.block(0,0,3,1) = y11;
    x.block(3,0,3,1) = y21;
    MatrixXd J_num = numerical_gradient(pointPointDist_userfun,x,2); 
    MatrixXd J = pointPointDistJac(y11,y21);
    for(int j = 0;j<6;j++)
    {
      BOOST_CHECK(abs(J_num(j)-J(j))<1e-4);
    }
  }
}

BOOST_AUTO_TEST_CASE(point2lineTest1)
{
  // Test if pointLineDist is correct
  for(int i = 0;i<100;i++)
  {
    Vector3d y21 = Vector3d::Random();
    Vector3d y22 = Vector3d::Random();
    Vector3d y11 = Vector3d::Random();
    double a = (y11-y21).norm();
    double b = (y22-y21).norm();
    double c = (y11-y22).norm();
    double s = (a+b+c)/2;
    double size = sqrt(s*(s-a)*(s-b)*(s-c));
    double d = size*2.0/b;
    VectorXd x = VectorXd::Zero(9);
    x.block(0,0,3,1) = y11;
    x.block(3,0,3,1) = y21;
    x.block(6,0,3,1) = y22;
    double d2 = pointLineDist(y11,y21,y22);
    BOOST_CHECK(abs(d-d2)<1e-5);
  }
}

BOOST_AUTO_TEST_CASE(point2lineTest2)
{
  // Test if pointLineDistJac matches that with numerical gradient
  for(int i = 0;i<100;i++)
  {
    Vector3d y21 = Vector3d::Random();
    Vector3d y22 = Vector3d::Random();
    Vector3d y11 = Vector3d::Random();
    VectorXd x = VectorXd::Zero(9);
    x.block(0,0,3,1) = y11;
    x.block(3,0,3,1) = y21;
    x.block(6,0,3,1) = y22;
    MatrixXd J_num = numerical_gradient(pointLineDist_userfun,x,2); 
    MatrixXd J = pointLineDistJac(y11,y21,y22);
    for(int j = 0;j<9;j++)
    {
      BOOST_CHECK(abs(J_num(j)-J(j))<1e-4);
    }
  }
}

BOOST_AUTO_TEST_CASE(line2lineTest)
{
  // Test if the lineLineDistJac matches that with numerical gradient
  for(int i = 0;i<100;i++)
  {
    Vector3d y11 = Vector3d::Random();
    Vector3d y12 = Vector3d::Random();
    Vector3d y21 = Vector3d::Random();
    Vector3d y22 = Vector3d::Random();
    VectorXd x = VectorXd::Zero(12);
    x.block(0,0,3,1) = y11;
    x.block(3,0,3,1) = y12;
    x.block(6,0,3,1) = y21;
    x.block(9,0,3,1) = y22;
    MatrixXd J_num = numerical_gradient(lineLineDist_userfun,x,2);
    MatrixXd J = lineLineDistJac(y11,y12,y21,y22);
    for(int j = 0;j<12;j++)
    {
      BOOST_CHECK(abs(J_num(j)-J(j))<1e-4);
    }
  }
}

BOOST_AUTO_TEST_CASE(point2planeTest)
{
  for(int i = 0;i<100;i++)
  {
    Vector3d y11 = Vector3d::Random();
    Vector3d y21 = Vector3d::Random();
    Vector3d y22 = Vector3d::Random();
    Vector3d y23 = Vector3d::Random();
    VectorXd x = VectorXd::Zero(12);
    x.block(0,0,3,1) = y11;
    x.block(3,0,3,1) = y21;
    x.block(6,0,3,1) = y22;
    x.block(9,0,3,1) = y23;
    MatrixXd J_num = numerical_gradient(pointPlaneDist_userfun,x,2);
    MatrixXd J = pointPlaneDistJac(y11,y21,y22,y23);
    for(int j = 0;j<12;j++)
    {
      BOOST_CHECK(abs(J_num(j)-J(j))<1e-4);
    }
  }
}
