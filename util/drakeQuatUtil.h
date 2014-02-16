#ifndef __DRAKE_QUAT_UTIL_H__
#define __DRAKE_QUAT_UTIL_H__
#include <Eigen/Dense>
#include <cstring>
#include <cmath>
void quatConjugate(const Eigen::Vector4d &q, Eigen::Vector4d &q_conj);
void quatConjugate(const Eigen::Vector4d &q, Eigen::Vector4d &q_conj, Eigen::Matrix4d &dq_conj);
void quatProduct(const Eigen::Vector4d &q1,const Eigen::Vector4d &q2, Eigen::Vector4d &r);
void quatProduct(const Eigen::Vector4d &q1,const Eigen::Vector4d &q2, Eigen::Vector4d &r, Eigen::Matrix<double,4,8> &dr);
void quatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v, Eigen::Vector3d &r);
void quatRotateVec(const Eigen::Vector4d &q, const Eigen::Vector3d &v, Eigen::Vector3d &r, Eigen::Matrix<double,3,7> &dr);
void quatDiff(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, Eigen::Vector4d &r, Eigen::Matrix<double,4,8> &dr);
void quatDiffAxisInvar(const Eigen::Vector4d &q1, const Eigen::Vector4d &q2, const Eigen::Vector3d &u, double &e, Eigen::Matrix<double,1,11> &de);
#endif
