#ifndef __DRAKE_GEOMETRY_UTIL_H__
#define __DRAKE_GEOMETRY_UTIL_H__

#include <Eigen/Dense>
#include <cstring>
#include <cmath>

Eigen::Vector4d quatConjugate(const Eigen::Vector4d &q);
Eigen::Matrix4d dquatConjugate();
Eigen::Vector4d quatProduct(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
Eigen::Matrix<double, 4, 8> dquatProduct(const Eigen::Vector4d &q1,const Eigen::Vector4d &q2);
Eigen::Vector3d quatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v);
Eigen::Matrix<double, 3, 7> dquatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v);
Eigen::Vector4d quatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
Eigen::Matrix<double, 4, 8> dquatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
double quatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u);
Eigen::Matrix<double, 1, 11> dquatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u);


template <typename Derived>
Eigen::Vector4d axis2quat(const Eigen::MatrixBase<Derived>& a);

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> axis2rotmat(const Eigen::MatrixBase<Derived>& a);

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> quat2rpy(const Eigen::MatrixBase<Derived>& q);

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> axis2rpy(const Eigen::MatrixBase<Derived>& a);

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> quat2axis(const Eigen::MatrixBase<Derived>& q);


#endif
