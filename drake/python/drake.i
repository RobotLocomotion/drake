%module drake_wrapper

%include <std_except.i>
%include <std_string.i>
%include <windows.i>

%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>
#include "CoordinateFrame.h"
#include "DrakeSystem.h"
#include "Pendulum.h"
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
%eigen_typemaps(DrakeSystem::VectorXs)

%include "CoordinateFrame.h"
%include "DrakeSystem.h"
%include "Pendulum.h"
