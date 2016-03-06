#include "expmap2quat.h"


using namespace Eigen;

Vector4d expmap2quatNonDegenerate(const Ref<const Vector3d>& v, double theta)
{
  double t2;
  double t3;
  double t4;
  Eigen::Vector4d q;

  t2 = theta*(1.0/2.0);
  t3 = 1.0/theta;
  t4 = sin(t2);
  q(0,0) = cos(t2);
  q(1,0) = t3*t4*v(0);
  q(2,0) = t3*t4*v(1);
  q(3,0) = t3*t4*v(2);

  return q;
}

Vector4d expmap2quatDegenerate(const Ref<const Vector3d>& v, double theta)
{
  double t2;
  double t3;
  double t4;
  Eigen::Vector4d q;

  t2 = theta*theta;
  t3 = t2*8.0E1;
  t4 = t3-1.92E3;
  q(0,0) = t2*(-1.0/8.0)+1.0;
  q(1,0) = t4*v(0)*(-2.604166666666667E-4);
  q(2,0) = t4*v(1)*(-2.604166666666667E-4);
  q(3,0) = t4*v(2)*(-2.604166666666667E-4);

  return q;
}

Matrix<double, 4, 3> dexpmap2quatNonDegenerate(const Ref<const Vector3d>& v, double theta)
{
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  Eigen::Matrix<double, 4, 3> q;

  t2 = 1.0/theta;
  t3 = theta*(1.0/2.0);
  t4 = sin(t3);
  t5 = v(0)*v(0);
  t6 = 1.0/(theta*theta*theta);
  t7 = cos(t3);
  t8 = t4*2.0;
  t10 = t7*theta;
  t9 = t8-t10;
  t11 = theta*theta;
  t12 = t4*t11*2.0;
  t13 = v(1)*v(1);
  t14 = v(2)*v(2);
  q(0,0) = t2*t4*v(0)*(-1.0/2.0);
  q(0,1) = t2*t4*v(1)*(-1.0/2.0);
  q(0,2) = t2*t4*v(2)*(-1.0/2.0);
  q(1,0) = t6*(t12-t4*t5*2.0+t5*t7*theta)*(1.0/2.0);
  q(1,1) = t6*t9*v(0)*v(1)*(-1.0/2.0);
  q(1,2) = t6*t9*v(0)*v(2)*(-1.0/2.0);
  q(2,0) = t6*t9*v(0)*v(1)*(-1.0/2.0);
  q(2,1) = t6*(t12-t4*t13*2.0+t7*t13*theta)*(1.0/2.0);
  q(2,2) = t6*t9*v(1)*v(2)*(-1.0/2.0);
  q(3,0) = t6*t9*v(0)*v(2)*(-1.0/2.0);
  q(3,1) = t6*t9*v(1)*v(2)*(-1.0/2.0);
  q(3,2) = t6*(t12-t4*t14*2.0+t7*t14*theta)*(1.0/2.0);

  return q;
}

Matrix<double, 4, 3> dexpmap2quatDegenerate(const Ref<const Vector3d>& v, double theta)
{
  double t10;
  double t11;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  Eigen::Matrix<double, 4, 3> q;

  t2 = theta*theta;
  t3 = t2*8.0E1;
  t4 = t3-1.92E3;
  t5 = v(0)*v(0);
  t6 = t2-4.0E1;
  t7 = t6*v(0)*v(1)*(1.0/9.6E2);
  t8 = v(1)*v(1);
  t9 = t6*v(0)*v(2)*(1.0/9.6E2);
  t10 = t6*v(1)*v(2)*(1.0/9.6E2);
  t11 = v(2)*v(2);
  q(0,0) = t4*v(0)*1.302083333333333E-4;
  q(0,1) = t4*v(1)*1.302083333333333E-4;
  q(0,2) = t4*v(2)*1.302083333333333E-4;
  q(1,0) = t2*(-1.0/4.8E1)-t5*(1.0/2.4E1)+t2*t5*(1.0/9.6E2)+1.0/2.0;
  q(1,1) = t7;
  q(1,2) = t9;
  q(2,0) = t7;
  q(2,1) = t2*(-1.0/4.8E1)-t8*(1.0/2.4E1)+t2*t8*(1.0/9.6E2)+1.0/2.0;
  q(2,2) = t10;
  q(3,0) = t9;
  q(3,1) = t10;
  q(3,2) = t2*(-1.0/4.8E1)-t11*(1.0/2.4E1)+t2*t11*(1.0/9.6E2)+1.0/2.0;

  return q;
}

Matrix<double, 12, 3> ddexpmap2quatNonDegenerate(const Ref<const Vector3d>& v, double theta)
{
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t20;
  double t21;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t3;
  double t30;
  double t31;
  double t32;
  double t33;
  double t34;
  double t35;
  double t36;
  double t37;
  double t38;
  double t39;
  double t4;
  double t40;
  double t41;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t50;
  double t51;
  double t52;
  double t53;
  double t54;
  double t55;
  double t6;
  double t7;
  double t8;
  double t9;
  Eigen::Matrix<double, 12, 3> q;

  t2 = theta*(1.0/2.0);
  t3 = sin(t2);
  t4 = v(0)*v(0);
  t5 = 1.0/(theta*theta*theta);
  t6 = cos(t2);
  t7 = t3*2.0;
  t15 = t6*theta;
  t8 = t7-t15;
  t9 = theta*theta;
  t10 = 1.0/(theta*theta*theta*theta*theta);
  t11 = t3*t4*t9;
  t12 = t4*t6*theta*6.0;
  t13 = t3*t9*4.0;
  t35 = t6*t9*theta*2.0;
  t14 = t11+t12+t13-t35-t3*t4*1.2E1;
  t16 = 1.0/theta;
  t17 = 1.0/(theta*theta*theta*theta);
  t18 = t8*t17*v(0)*v(1)*(3.0/2.0);
  t19 = 1.0/(theta*theta);
  t29 = t3*t19*v(0)*v(1)*(1.0/4.0);
  t20 = t18-t29;
  t21 = t6*theta*6.0;
  t22 = t3*t9;
  t34 = t3*1.2E1;
  t23 = t21+t22-t34;
  t24 = t3*t19*v(0)*v(2)*(1.0/4.0);
  t46 = t8*t17*v(0)*v(2)*(3.0/2.0);
  t25 = t24-t46;
  t26 = t5*t8*v(0)*v(1)*(1.0/4.0);
  t27 = t3*t9*2.0;
  t28 = v(1)*v(1);
  t30 = t16*t20*v(0);
  t42 = t5*t8*v(1)*(1.0/2.0);
  t31 = t30-t42;
  t32 = t16*t20*v(1);
  t48 = t5*t8*v(0)*(1.0/2.0);
  t33 = t32-t48;
  t36 = t3*t9*1.2E1;
  t37 = t3*t9*t28;
  t38 = t6*t28*theta*6.0;
  t39 = t13-t35+t37+t38-t3*t28*1.2E1;
  t40 = t3*t19*v(1)*v(2)*(1.0/4.0);
  t49 = t8*t17*v(1)*v(2)*(3.0/2.0);
  t41 = t40-t49;
  t43 = t5*t8*v(0)*v(2)*(1.0/4.0);
  t44 = t5*t8*v(1)*v(2)*(1.0/4.0);
  t45 = v(2)*v(2);
  t50 = t5*t8*v(2)*(1.0/2.0);
  t47 = -t50-t16*t25*v(0);
  t51 = -t42-t16*t41*v(2);
  t52 = t3*t9*t45;
  t53 = t6*t45*theta*6.0;
  t55 = t3*t45*1.2E1;
  t54 = t13-t35+t52+t53-t55;
  q(0,0) = t5*(t27-t3*t4*2.0+t4*t6*theta)*(-1.0/4.0);
  q(0,1) = t26;
  q(0,2) = t43;
  q(1,0) = t10*v(0)*(t11+t12+t36-t3*t4*1.2E1-t6*t9*theta*6.0)*(-1.0/4.0);
  q(1,1) = t10*t14*v(1)*(-1.0/4.0);
  q(1,2) = t10*t14*v(2)*(-1.0/4.0);
  q(2,0) = t31;
  q(2,1) = t33;
  q(2,2) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(3,0) = t47;
  q(3,1) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(3,2) = t5*t8*v(0)*(-1.0/2.0)-t16*t25*v(2);
  q(4,0) = t26;
  q(4,1) = t5*(t27-t3*t28*2.0+t6*t28*theta)*(-1.0/4.0);
  q(4,2) = t44;
  q(5,0) = t31;
  q(5,1) = t33;
  q(5,2) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(6,0) = t10*t39*v(0)*(-1.0/4.0);
  q(6,1) = t10*v(1)*(t36+t37+t38-t3*t28*1.2E1-t6*t9*theta*6.0)*(-1.0/4.0);
  q(6,2) = t10*t39*v(2)*(-1.0/4.0);
  q(7,0) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(7,1) = t5*t8*v(2)*(-1.0/2.0)-t16*t41*v(1);
  q(7,2) = t51;
  q(8,0) = t43;
  q(8,1) = t44;
  q(8,2) = t5*(t27-t3*t45*2.0+t6*t45*theta)*(-1.0/4.0);
  q(9,0) = t47;
  q(9,1) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(9,2) = -t48-t16*t25*v(2);
  q(10,0) = t10*t23*v(0)*v(1)*v(2)*(-1.0/4.0);
  q(10,1) = -t50-t16*t41*v(1);
  q(10,2) = t51;
  q(11,0) = t10*t54*v(0)*(-1.0/4.0);
  q(11,1) = t10*t54*v(1)*(-1.0/4.0);
  q(11,2) = t10*v(2)*(t36+t52+t53-t55-t6*t9*theta*6.0)*(-1.0/4.0);

  return q;
}

Matrix<double, 12, 3> ddexpmap2quatDegenerate(const Ref<const Vector3d>& v, double theta)
{
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t2;
  double t20;
  double t21;
  double t22;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  Eigen::Matrix<double, 12, 3> q;

  t2 = theta*theta;
  t3 = v(0)*v(0);
  t4 = t2-4.0E1;
  t5 = t3*3.2E1;
  t6 = t2*1.6E1;
  t9 = t2*t3;
  t7 = v(1)*(t5+t6-t9-6.4E2)*6.510416666666667E-5;
  t8 = v(1)*v(1);
  t10 = t2-3.2E1;
  t11 = v(2)*v(2);
  t12 = t2*(1.0/9.6E1);
  t13 = t8*3.2E1;
  t15 = t2*t8;
  t14 = v(0)*(t6+t13-t15-6.4E2)*6.510416666666667E-5;
  t16 = t2*4.8E1;
  t17 = v(2)*(t6+t13-t15-6.4E2)*6.510416666666667E-5;
  t18 = t11*3.2E1;
  t19 = v(2)*(t5+t6-t9-6.4E2)*6.510416666666667E-5;
  t21 = t2*t11;
  t20 = v(0)*(t6+t18-t21-6.4E2)*6.510416666666667E-5;
  t22 = v(1)*(t6+t18-t21-6.4E2)*6.510416666666667E-5;
  q(0,0) = t3*(1.0/4.8E1)+t12-t2*t3*5.208333333333333E-4-1.0/4.0;
  q(0,1) = t4*v(0)*v(1)*(-5.208333333333333E-4);
  q(0,2) = t4*v(0)*v(2)*(-5.208333333333333E-4);
  q(1,0) = v(0)*(t5+t16-t2*t3-1.92E3)*6.510416666666667E-5;
  q(1,1) = t7;
  q(1,2) = v(2)*(t5+t6-t2*t3-6.4E2)*6.510416666666667E-5;
  q(2,0) = t7;
  q(2,1) = t14;
  q(2,2) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(3,0) = t19;
  q(3,1) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(3,2) = t20;
  q(4,0) = t4*v(0)*v(1)*(-5.208333333333333E-4);
  q(4,1) = t8*(1.0/4.8E1)+t12-t2*t8*5.208333333333333E-4-1.0/4.0;
  q(4,2) = t4*v(1)*v(2)*(-5.208333333333333E-4);
  q(5,0) = t7;
  q(5,1) = t14;
  q(5,2) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(6,0) = t14;
  q(6,1) = v(1)*(t13-t15+t16-1.92E3)*6.510416666666667E-5;
  q(6,2) = t17;
  q(7,0) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(7,1) = t17;
  q(7,2) = v(1)*(t6+t18-t2*t11-6.4E2)*6.510416666666667E-5;
  q(8,0) = t4*v(0)*v(2)*(-5.208333333333333E-4);
  q(8,1) = t4*v(1)*v(2)*(-5.208333333333333E-4);
  q(8,2) = t11*(1.0/4.8E1)+t12-t2*t11*5.208333333333333E-4-1.0/4.0;
  q(9,0) = t19;
  q(9,1) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(9,2) = t20;
  q(10,0) = t10*v(0)*v(1)*v(2)*(-6.510416666666667E-5);
  q(10,1) = t17;
  q(10,2) = t22;
  q(11,0) = t20;
  q(11,1) = t22;
  q(11,2) = v(2)*(t16+t18-t21-1.92E3)*6.510416666666667E-5;

  return q;
}

