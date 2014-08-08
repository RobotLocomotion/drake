#ifndef __DRAKE_QUAT_UTIL_H__
#define __DRAKE_QUAT_UTIL_H__
#include <Eigen/Dense>
#include <cstring>
#include <cmath>
Eigen::Vector4d quatConjugate(const Eigen::Vector4d &q);
Eigen::Matrix4d dquatConjugate(const Eigen::Vector4d &q);
Eigen::Vector4d quatProduct(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
Eigen::Matrix<double, 4, 8> dquatProduct(const Eigen::Vector4d &q1,const Eigen::Vector4d &q2);
Eigen::Vector3d quatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v);
Eigen::Matrix<double, 3, 7> dquatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v);
Eigen::Vector4d quatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
Eigen::Matrix<double, 4, 8> dquatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2);
double quatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u);
Eigen::Matrix<double, 1, 11> dquatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u);
#endif
