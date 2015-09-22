#ifdef SWIGPYTHON
%module(package="pydrake") rbm
#endif
#ifdef SWIGMATLAB
%module(package="rbm") rbm
#endif

%include "exception_helper.i"
%include <std_string.i>
%include <windows.i>

%{
#ifdef SWIGPYTHON
  #define SWIG_FILE_WITH_INIT
  #include <Python.h>
#endif
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

%include "gradientvar.i"
%import <Eigen/Core>

%include "KinematicsCache.h"
%template(KinematicsCache_d) KinematicsCache<Eigen::VectorXd::Scalar>;

%include "RigidBodyManipulator.h"
%template(doKinematics) RigidBodyManipulator::doKinematics<Eigen::VectorXd, Eigen::VectorXd>;
%template(centerOfMass) RigidBodyManipulator::centerOfMass<Eigen::VectorXd::Scalar>;
%template(forwardKin) RigidBodyManipulator::forwardKin<Eigen::VectorXd>;
