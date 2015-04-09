#include <Eigen/Dense>


template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> expmap2quatNonDegenerate(const Eigen::MatrixBase<Derived>& v, const typename Derived::Scalar& theta)
{
  typename Derived::Scalar t2;
  typename Derived::Scalar t3;
  typename Derived::Scalar t4;
  Eigen::Matrix<typename Derived::Scalar, 4, 1> q;

  t2 = theta*(1.0/2.0);
  t3 = 1.0/theta;
  t4 = sin(t2);
  q(0,0) = cos(t2);
  q(1,0) = t3*t4*v(0);
  q(2,0) = t3*t4*v(1);
  q(3,0) = t3*t4*v(2);

  return q;
};



template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> expmap2quatDegenerate(const Eigen::MatrixBase<Derived>& v, const typename Derived::Scalar& theta)
{
  typename Derived::Scalar t2;
  typename Derived::Scalar t3;
  typename Derived::Scalar t4;
  Eigen::Matrix<typename Derived::Scalar, 4, 1> q;

  t2 = theta*theta;
  t3 = t2*8.0E1;
  t4 = t3-1.92E3;
  q(0,0) = t2*(-1.0/8.0)+1.0;
  q(1,0) = t4*v(0)*(-2.604166666666667E-4);
  q(2,0) = t4*v(1)*(-2.604166666666667E-4);
  q(3,0) = t4*v(2)*(-2.604166666666667E-4);

  return q;
};



template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 3> dexpmap2quatNonDegenerate(const Eigen::MatrixBase<Derived>& v, const typename Derived::Scalar& theta)
{
  typename Derived::Scalar t10;
  typename Derived::Scalar t11;
  typename Derived::Scalar t12;
  typename Derived::Scalar t13;
  typename Derived::Scalar t14;
  typename Derived::Scalar t2;
  typename Derived::Scalar t3;
  typename Derived::Scalar t4;
  typename Derived::Scalar t5;
  typename Derived::Scalar t6;
  typename Derived::Scalar t7;
  typename Derived::Scalar t8;
  typename Derived::Scalar t9;
  Eigen::Matrix<typename Derived::Scalar, 4, 3> q;

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
};



template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 3> dexpmap2quatDegenerate(const Eigen::MatrixBase<Derived>& v, const typename Derived::Scalar& theta)
{
  typename Derived::Scalar t10;
  typename Derived::Scalar t11;
  typename Derived::Scalar t2;
  typename Derived::Scalar t3;
  typename Derived::Scalar t4;
  typename Derived::Scalar t5;
  typename Derived::Scalar t6;
  typename Derived::Scalar t7;
  typename Derived::Scalar t8;
  typename Derived::Scalar t9;
  Eigen::Matrix<typename Derived::Scalar, 4, 3> q;

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
};



template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 12, 3> ddexpmap2quatNonDegenerate(const Eigen::MatrixBase<Derived>& v, const typename Derived::Scalar& theta)
{
  typename Derived::Scalar t10;
  typename Derived::Scalar t11;
  typename Derived::Scalar t12;
  typename Derived::Scalar t13;
  typename Derived::Scalar t14;
  typename Derived::Scalar t15;
  typename Derived::Scalar t16;
  typename Derived::Scalar t17;
  typename Derived::Scalar t18;
  typename Derived::Scalar t19;
  typename Derived::Scalar t2;
  typename Derived::Scalar t20;
  typename Derived::Scalar t21;
  typename Derived::Scalar t22;
  typename Derived::Scalar t23;
  typename Derived::Scalar t24;
  typename Derived::Scalar t25;
  typename Derived::Scalar t26;
  typename Derived::Scalar t27;
  typename Derived::Scalar t28;
  typename Derived::Scalar t29;
  typename Derived::Scalar t3;
  typename Derived::Scalar t30;
  typename Derived::Scalar t31;
  typename Derived::Scalar t32;
  typename Derived::Scalar t33;
  typename Derived::Scalar t34;
  typename Derived::Scalar t35;
  typename Derived::Scalar t36;
  typename Derived::Scalar t37;
  typename Derived::Scalar t38;
  typename Derived::Scalar t39;
  typename Derived::Scalar t4;
  typename Derived::Scalar t40;
  typename Derived::Scalar t41;
  typename Derived::Scalar t42;
  typename Derived::Scalar t43;
  typename Derived::Scalar t44;
  typename Derived::Scalar t45;
  typename Derived::Scalar t46;
  typename Derived::Scalar t47;
  typename Derived::Scalar t48;
  typename Derived::Scalar t49;
  typename Derived::Scalar t5;
  typename Derived::Scalar t50;
  typename Derived::Scalar t51;
  typename Derived::Scalar t52;
  typename Derived::Scalar t53;
  typename Derived::Scalar t54;
  typename Derived::Scalar t55;
  typename Derived::Scalar t6;
  typename Derived::Scalar t7;
  typename Derived::Scalar t8;
  typename Derived::Scalar t9;
  Eigen::Matrix<typename Derived::Scalar, 12, 3> q;

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
};



template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 12, 3> ddexpmap2quatDegenerate(const Eigen::MatrixBase<Derived>& v, const typename Derived::Scalar& theta)
{
  typename Derived::Scalar t10;
  typename Derived::Scalar t11;
  typename Derived::Scalar t12;
  typename Derived::Scalar t13;
  typename Derived::Scalar t14;
  typename Derived::Scalar t15;
  typename Derived::Scalar t16;
  typename Derived::Scalar t17;
  typename Derived::Scalar t18;
  typename Derived::Scalar t19;
  typename Derived::Scalar t2;
  typename Derived::Scalar t20;
  typename Derived::Scalar t21;
  typename Derived::Scalar t22;
  typename Derived::Scalar t3;
  typename Derived::Scalar t4;
  typename Derived::Scalar t5;
  typename Derived::Scalar t6;
  typename Derived::Scalar t7;
  typename Derived::Scalar t8;
  typename Derived::Scalar t9;
  Eigen::Matrix<typename Derived::Scalar, 12, 3> q;

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
};



