#ifndef __DRAKE_QUAT_UTIL_H__
#define __DRAKE_QUAT_UTIL_H__
#include <Eigen/Dense>
#include <cstring>
#include <cmath>
void quatConjugate(const Eigen::Vector4d &q, Eigen::Vector4d &q_conj);
void quatConjugate(const Eigen::Vector4d &q, Eigen::Vector4d &q_conj, Eigen::Matrix4d dq_conj);
void quatProduct(const Eigen::Vector4d &q1,const Eigen::Vector4d &q2, Eigen::Vector4d &r);
void quatProduct(const Eigen::Vector4d &q1,const Eigen::Vector4d &q2, Eigen::Vector4d &r, Eigen::Matrix<double,4,8> &dr);
void quatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, Eigen::Vector4d &r, Eigen::Matrix<double,4,8> &dr);
void quatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u, double &e, Eigen::Matrix<double,1,11> &de);
void quatTransform(const Eigen::Vector3d &u, const Eigen::Vector3d &v, Eigen::Vector4d &quat);
void quatTransform(const Eigen::Vector3d &u, const Eigen::Vector3d &v, Eigen::Vector4d &quat, Eigen::Matrix<double,4,6> &dquat);
#endif
