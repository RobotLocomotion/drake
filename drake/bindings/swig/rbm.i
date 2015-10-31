%module(package="pydrake") rbm

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
%eigen_typemaps(Eigen::Matrix<double, SPACE_DIMENSION, 1>)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)
%eigen_typemaps(Eigen::VectorXi)

// %include "gradientvar.i"
%include "KinematicsCache.h"
%template(KinematicsCache_d) KinematicsCache<double>;

%include "RigidBodyManipulator.h"
%extend RigidBodyManipulator {
  KinematicsCache<double> doKinematics(const Eigen::MatrixBase<Eigen::VectorXd>& q, const Eigen::MatrixBase<Eigen::VectorXd>& v, int gradient_order = 0, bool compute_JdotV = true) {
    return $self->doKinematics<Eigen::VectorXd, Eigen::VectorXd>(q, v, gradient_order, compute_JdotV);
  }

  Eigen::MatrixXd forwardKin(
      const KinematicsCache<double> &cache, const Eigen::MatrixXd &points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type) const
  {
    return $self->forwardKin(cache, points, current_body_or_frame_ind, new_body_or_frame_ind, rotation_type);
  }

  Eigen::Matrix<double, SPACE_DIMENSION, 1> centerOfMass(KinematicsCache<double> &cache, const std::set<int> &robotnum = default_robot_num_set) const {
    return $self->centerOfMass<double>(cache, robotnum);
  }

  // GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> forwardKin(const KinematicsCache<double>& cache, const Eigen::MatrixBase<Eigen::MatrixXd>& points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, int gradient_order) const {
  //   return $self->forwardKin<Eigen::MatrixXd>(cache, points, current_body_or_frame_ind, new_body_or_frame_ind, rotation_type, gradient_order);
  // }
}

