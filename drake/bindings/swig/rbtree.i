%module(package="pydrake") rbtree

%include "exception_helper.i"
%include <std_string.i>
%include <windows.i>
#define DRAKERBM_EXPORT

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
%import <autodiffutils.i>

%template(vectorVectorXd) std::vector<Eigen::VectorXd>;
%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorString) std::vector<std::string>;

%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::Matrix<double, SPACE_DIMENSION, 1>)
%eigen_typemaps(Eigen::Matrix<double, SPACE_DIMENSION, Eigen::Dynamic>)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)
%eigen_typemaps(Eigen::VectorXi)

%include "KinematicsCache.h"
%template(KinematicsCache_d) KinematicsCache<double>;
%template(KinematicsCache_adVectorDynamic) KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> >;
%template(KinematicsCache_adVectorMax73) KinematicsCache<Eigen::AutoDiffScalar<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73> > >;
%template(AutoDiff3XDynamic) AutoDiffWrapper<Eigen::VectorXd, SPACE_DIMENSION, Eigen::Dynamic>;
%template(AutoDiff3XMax73) AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, SPACE_DIMENSION, Eigen::Dynamic>;

%include "RigidBodyTree.h"
%extend RigidBodyTree {
  KinematicsCache<double> doKinematics(const Eigen::MatrixBase<Eigen::VectorXd>& q, const Eigen::MatrixBase<Eigen::VectorXd>& v) {
    return $self->doKinematics(q, v);
  }

  KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> > doKinematics(const AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, 1>& q, const AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, 1>& v) {
    return $self->doKinematics(q, v);
  }

  KinematicsCache<Eigen::AutoDiffScalar<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73> > > doKinematics(const AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, Eigen::Dynamic, 1>& q, const AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, Eigen::Dynamic, 1>& v) {
    return $self->doKinematics(q, v);
  }

  Eigen::Matrix<double, SPACE_DIMENSION, Eigen::Dynamic> transformPoints(
      const KinematicsCache<double> &cache, const Eigen::Matrix<double, SPACE_DIMENSION, Eigen::Dynamic> &points, int current_body_or_frame_ind, int new_body_or_frame_ind) const
  {
    return $self->transformPoints(cache, points, current_body_or_frame_ind, new_body_or_frame_ind);
  }

  AutoDiffWrapper<Eigen::VectorXd, SPACE_DIMENSION, Eigen::Dynamic> transformPoints(
      const KinematicsCache<Eigen::AutoDiffScalar<Eigen::VectorXd> > &cache, const Eigen::Matrix<double, SPACE_DIMENSION, Eigen::Dynamic> &points, int current_body_or_frame_ind, int new_body_or_frame_ind) const
  {
    return $self->transformPoints(cache, points, current_body_or_frame_ind, new_body_or_frame_ind);
  }

  AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, SPACE_DIMENSION, Eigen::Dynamic> transformPoints(
      const KinematicsCache<Eigen::AutoDiffScalar<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73> > > &cache, const Eigen::Matrix<double, SPACE_DIMENSION, Eigen::Dynamic> &points, int current_body_or_frame_ind, int new_body_or_frame_ind) const
  {
    return $self->transformPoints(cache, points, current_body_or_frame_ind, new_body_or_frame_ind);
  }

  Eigen::Matrix<double, SPACE_DIMENSION, 1> centerOfMass(KinematicsCache<double> &cache, const std::set<int> &robotnum = default_robot_num_set) const {
    return $self->centerOfMass(cache, robotnum);
  }
}

