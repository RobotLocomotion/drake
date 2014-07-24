#include <cmath>
#include <iostream>

#include "simpleClosestPointFunctions.h"

using namespace std;
using namespace Eigen;

double pointPointDist(const Vector3d y11, const Vector3d y21)
{
  return (y11-y21).norm();
}

double pointLineDist(const Vector3d y11, const Vector3d y21, const Vector3d y22)
{
  Vector3d a = y11-y21;
  Vector3d b = y22-y21;
  Vector3d d = a-(a.dot(b))*b/(b.dot(b));
  double dd = d.norm();
  return dd;
}

double lineLineDist(const Vector3d y11, const Vector3d y12, const Vector3d y21, const Vector3d y22)
{
  Vector3d a = y12-y11;
  Vector3d b = y22-y21;
  Vector3d c = y21-y11;
  Vector3d f = a.cross(b);
  double g = std::abs(c.dot(f));
  double d = g/(f.norm());
  return d;
}

double pointPlaneDist(const Vector3d y11, const Vector3d y21, const Vector3d y22, const Vector3d y23)
{
  Vector3d v_a = y22-y21;
  Vector3d v_b = y23-y21;
  Vector3d normal = v_a.cross(v_b);
  normal = normal/normal.norm();
  double d = abs(normal.dot(y11-y21));
  return d;
}

MatrixXd pointPointDistJac(const Vector3d y11, const Vector3d y21)
{
  MatrixXd J = MatrixXd::Zero(1,6);
  Vector3d a = y11-y21;
  RowVector3d dd_da = a.transpose()/a.norm();
  J.block(0,0,1,3) = dd_da;
  J.block(0,3,1,3) = -dd_da;
  return J;
}
MatrixXd pointLineDistJac(const Vector3d y11, const Vector3d y21, const Vector3d y22)
{
  const Vector3d a(y22 - y21);
  const Vector3d b(y21 - y11);
  const Vector3d c(a.cross(b));
  const double norm_a = a.norm();
  const double norm_a_squared = a.squaredNorm();
  const double norm_c = c.norm();
  const Matrix3d da_dy21(-Matrix3d::Identity());
  const Matrix3d da_dy22(Matrix3d::Identity());
  const Matrix3d db_dy11(-Matrix3d::Identity());
  const Matrix3d db_dy21(Matrix3d::Identity());

  const RowVector3d dd_da(-( c.cross(b)/(norm_a*norm_c) + 
                          norm_c*a/(norm_a*norm_a_squared)));
  const RowVector3d dd_db(c.cross(a)/(norm_a*norm_c));
  MatrixXd Jd(1,9);
  Jd << dd_db*db_dy11, dd_da*da_dy21 + dd_db*db_dy21, dd_da*da_dy22;
  return Jd;
}

MatrixXd lineLineDistJac(Vector3d y11, Vector3d y12, Vector3d y21, Vector3d y22)
{
  Vector3d a(y12 - y11);
  Vector3d b(y22 - y21);
  Vector3d c(y21 - y11);
  Vector3d f(a.cross(b));
  Matrix3d da_dy11(-Matrix3d::Identity());
  Matrix3d da_dy12(Matrix3d::Identity());
  Matrix3d db_dy21(-Matrix3d::Identity());
  Matrix3d db_dy22(Matrix3d::Identity());
  if (f.dot(c) < 0) {
    a = y11 - y12;
    b = y22 - y21;
    f = a.cross(b);
    da_dy11 = Matrix3d::Identity();
    da_dy12 = -Matrix3d::Identity();
    db_dy21 = -Matrix3d::Identity();
    db_dy22 = Matrix3d::Identity();
  }
  const double g = c.dot(f);

  const double norm_f = f.norm();
  const Matrix3d dc_dy11(-Matrix3d::Identity());
  const Matrix3d dc_dy21(Matrix3d::Identity());

  const Vector3d dd_df(-f*abs(g)/pow(norm_f,3));
  const double dd_dg(g/(abs(g)*norm_f));
  const Vector3d& dg_df = c;
  const Vector3d full_dd_df(dd_df + dd_dg*dg_df);

  const RowVector3d dd_da = -full_dd_df.cross(b); 
  const RowVector3d dd_db = full_dd_df.cross(a); 
  const RowVector3d dd_dc = dd_dg*f;
  MatrixXd Jd(1,12);

  Jd << dd_da*da_dy11 + dd_dc*dc_dy11, dd_da*da_dy12, 
        dd_db*db_dy21 + dd_dc*dc_dy21, dd_db*db_dy22;

  return Jd;
}

MatrixXd pointPlaneDistJac(Vector3d y11, Vector3d y21, Vector3d y22, Vector3d y23)
{
  Vector3d a(y22 - y21);
  Vector3d b(y23 - y21);
  Vector3d c(y11 - y21);
  Vector3d f(a.cross(b));
  Matrix3d da_dy21(-Matrix3d::Identity());
  Matrix3d da_dy22(Matrix3d::Identity());
  if(f.dot(c)<0)
  {
    a = y21-y22;
    f = a.cross(b);
    da_dy21 = Matrix3d::Identity();
    da_dy22 = -Matrix3d::Identity();
  }
  const Matrix3d db_dy21(-Matrix3d::Identity());
  const Matrix3d db_dy23(Matrix3d::Identity());
  const Matrix3d dc_dy21(-Matrix3d::Identity());
  const Matrix3d dc_dy11(Matrix3d::Identity());
  const double norm_f = f.norm();

  const Vector3d dd_df((Matrix3d::Identity()/norm_f - f*f.transpose()/pow(norm_f,3))*c);

  const RowVector3d dd_da(-dd_df.cross(b));
  const RowVector3d dd_db(dd_df.cross(a));
  const RowVector3d dd_dc(f/norm_f);

  MatrixXd Jd(1,12);
  Jd << dd_dc*dc_dy11, dd_da*da_dy21 + dd_db*db_dy21 + dd_dc*dc_dy21, 
        dd_da*da_dy22, dd_db*db_dy23;

  //DEBUG
  //cout << "simpleClosestPointFunctions::pointPlaneDistJac: Values:" << endl;
  //cout << "y11 = [" << y11.transpose() << "]';" << endl;
  //cout << "y21 = [" << y21.transpose() << "]';" << endl;
  //cout << "y22 = [" << y22.transpose() << "]';" << endl;
  //cout << "y23 = [" << y23.transpose() << "]';" << endl;
  //cout << "a = [" << a.transpose() << "]';" << endl;
  //cout << "b = [" << b.transpose() << "]';" << endl;
  //cout << "c = [" << c.transpose() << "]';" << endl;
  //cout << "f = [" << f.transpose() << "]';" << endl;
  //cout << "dd_df = [" << dd_df.transpose() << "]';" << endl;
  //cout << "dd_da = [" << dd_da << "]';" << endl;
  //cout << "dd_db = [" << dd_da << "]';" << endl;
  //cout << "dd_dc = [" << dd_da << "]';" << endl;
  //cout << "Jd = [" << Jd << "]';" << endl;
  //cout << "simpleClosestPointFunctions::lineLineDistJac: End Values:" << endl;
  //END_DEBUG
  return Jd;
}
