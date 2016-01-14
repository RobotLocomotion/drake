%module(package="pydrake.solvers") ik

%include "exception_helper.i"
%include <std_string.i>
%include <windows.i>
#define DRAKEIK_EXPORT
#define DRAKEIKOPTIONS_EXPORT
#define DRAKERIGIDBODYCONSTRAINT_EXPORT

%{
#ifdef SWIGPYTHON
  #define SWIG_FILE_WITH_INIT
  #include <Python.h>
#endif
#include "RigidBodyIK.h"
%}

%include <typemaps.i>
%include <std_vector.i>
%include <eigen.i>
%import <rbtree.i>

%template(vectorConstraintPtr) std::vector<RigidBodyConstraint *>;

%include "IKoptions.h"
%include "RigidBodyIK.h"
%include "RigidBodyConstraint.h"
