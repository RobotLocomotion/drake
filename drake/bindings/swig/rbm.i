%module(package="pydrake") rbm

%include "exception_helper.i"
%include <std_string.i>
%include <windows.i>

%{
#ifdef SWIGPYTHON
  #define SWIG_FILE_WITH_INIT
  #include <Python.h>
#endif
#include "RigidBodyTree.h"
%}

%include <typemaps.i>
%include <std_vector.i>
%include <eigen.i>
%include <autodiff.i>
%import <autodiff_utils.i>

%template(vectorVectorXd) std::vector<Eigen::VectorXd>;
%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorString) std::vector<std::string>;

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::Matrix<double, SPACE_DIMENSION, 1>)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)
%eigen_typemaps(Eigen::VectorXi)

%include "KinematicsCache.h"
%template(KinematicsCache_d) KinematicsCache<double>;
%template(KinematicsCache_adVectorDynamic) KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> >;

%include "RigidBodyTree.h"
%extend RigidBodyTree {
  KinematicsCache<double> doKinematics(const Eigen::MatrixBase<Eigen::VectorXd>& q, const Eigen::MatrixBase<Eigen::VectorXd>& v) {
    return $self->doKinematics(q, v);
  }

  KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> > doKinematics(const AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, 1>& q, const AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, 1>& v) {
    return $self->doKinematics(q, v);
  }

  Eigen::VectorXd forwardKin(
      const KinematicsCache<double> &cache, const Eigen::Matrix<double, SPACE_DIMENSION, 1> &points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type) const
  {
    return $self->forwardKin(cache, points, current_body_or_frame_ind, new_body_or_frame_ind, rotation_type);
  }

  AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, 1> forwardKin(
      const KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> > &cache, const Eigen::Matrix<double, SPACE_DIMENSION, 1> &points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type) const
  {
    return $self->forwardKin(cache, points, current_body_or_frame_ind, new_body_or_frame_ind, rotation_type);
  }

  Eigen::Matrix<double, SPACE_DIMENSION, 1> centerOfMass(KinematicsCache<double> &cache, const std::set<int> &robotnum = default_robot_num_set) const {
    return $self->centerOfMass(cache, robotnum);
  }
}

