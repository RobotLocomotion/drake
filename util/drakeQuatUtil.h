#ifndef __DRAKE_QUAT_UTIL_H__
#define __DRAKE_QUAT_UTIL_H__
#include <Eigen/Dense>
#include <cstring>
#include <cmath>
void quatConjugate(Eigen::Vector4d q, Eigen::Vector4d &q_conj);
void quatConjugate(Eigen::Vector4d q, Eigen::Vector4d &q_conj, Eigen::Matrix4d dq_conj);
void quatProduct(Eigen::Vector4d q1,Eigen::Vector4d q2, Eigen::Vector4d &r);
void quatProduct(Eigen::Vector4d q1, Eigen::Vector4d q2, Eigen::Vector4d &r, Eigen::Matrix<double,4,8> &dr);
void quatDiff(Eigen::Vector4d q1, Eigen::Vector4d q2, Eigen::Vector4d &r, Eigen::Matrix<double,4,8> &dr);
void quatDiffAxisInvar(Eigen::Vector4d q1, Eigen::Vector4d q2, Eigen::Vector3d u, double &e, Eigen::Matrix<double,1,11> &de);
void quatTransform(Eigen::Vector3d u, Eigen::Vector3d v, Eigen::Vector4d &quat);
void quatTransform(Eigen::Vector3d u, Eigen::Vector3d v, Eigen::Vector4d &quat, Eigen::Matrix<double,4,6> &dquat);
#endif
