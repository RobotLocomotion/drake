%module(package="drake") systems_wrapper

%include <std_except.i>
%include <std_string.i>
%include <windows.i>

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "DrakeSystem.h"
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

%include <std_shared_ptr.i>
%shared_ptr(CascadeSystem)
%shared_ptr(FeedbackSystem)
%shared_ptr(DrakeSystem)

%import "CoordinateFrame.h"
%include "DrakeSystem.h"
