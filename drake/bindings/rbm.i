%module(package="pydrake") rbm

%include <std_except.i>
%include <std_string.i>
%include <windows.i>

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "RigidBodyManipulator.h"
%}

%include <typemaps.i>
%include <std_vector.i>
%include <eigen.i>

%template(vectorVectorXd) std::vector<Eigen::VectorXd>;
%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorString) std::vector<std::string>;

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)
%eigen_typemaps(Eigen::VectorXi)


#ifdef SWIGPYTHON
%fragment("gradientvar_typemaps", "header")
%{
  #include<iostream>
  template<typename Scalar, int Rows, int Cols>
  Eigen::MatrixXd nthGradientValue(const GradientVar<Scalar, Rows, Cols> &gvar, int n) {
    if (n <= 0) {
      Eigen::MatrixXd value;
      value = gvar.value();
      return value;
    } else {
      return nthGradientValue(gvar.gradient(), n-1);
    }
  }
%}

%define %gradientvar_typemaps(SCALAR, ROWS, COLS)

%typemap(out, fragment="gradientvar_typemaps") GradientVar< SCALAR, ROWS, COLS >
{
  int num_elements = $1.maxOrder() + 1;
  $result = PyTuple_New(num_elements);

  for (int i=0; i < num_elements; i++) {
    MatrixXd gval = nthGradientValue(*(&($1)), i);
    PyObject* pyarray;
    if (!ConvertFromEigenToNumPyMatrix< Eigen::MatrixXd >(&pyarray, &gval))
      SWIG_fail;
    PyTuple_SetItem($result, i, pyarray);
  }
}
%enddef

%gradientvar_typemaps(double, Eigen::Dynamic, Eigen::Dynamic)
%gradientvar_typemaps(double, SPACE_DIMENSION, 1)

#endif

%import "GradientVar.h"
%import <Eigen/Core>
%include "RigidBodyManipulator.h"

%template(doKinematics) RigidBodyManipulator::doKinematics<Eigen::VectorXd, Eigen::VectorXd>;
%template(massMatrix) RigidBodyManipulator::massMatrix<double>;
%template(centerOfMass) RigidBodyManipulator::centerOfMass<double>;
