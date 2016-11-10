%module(package="pydrake.solvers") ik

%rename(InverseKin) inverseKinSimple;
%rename(InverseKinPointwise) inverseKinPointwiseSimple;
%rename(InverseKinTraj) inverseKinTrajSimple;

%include "exception_helper.i"
%include <std_string.i>
%include <windows.i>
#define DRAKE_EXPORT

%{
#ifdef SWIGPYTHON
  #define SWIG_FILE_WITH_INIT
  #include <Python.h>
#endif
#include "drake/multibody/RigidBodyIK.h"
%}

%include <typemaps.i>
%include <std_vector.i>
%include <eigen.i>
%import <rbtree.i>

%template(intVector) std::vector<int>;
%template(vectorConstraintPtr) std::vector<RigidBodyConstraint *>;

%eigen_typemaps(Eigen::Matrix<double, 7, 1>)

%include "drake/multibody/IKoptions.h"
%include "drake/multibody/RigidBodyIK.h"
%include "drake/multibody/constraint/RigidBodyConstraint.h"
