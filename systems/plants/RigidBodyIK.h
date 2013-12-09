#ifndef __RIGIDBODYIK_H__
#define __RIGIDBODYIK_H__
#include <Eigen/Dense>
class RigidBodyManipulator;
class RigidBodyConstraint;
class IKoptions;

template <typename DerivedA, typename DerivedB, typename DerivedC>
extern void approximateIK(RigidBodyManipulator* model, const Eigen::MatrixBase<DerivedA> &q_seed, const Eigen::MatrixBase<DerivedB> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, Eigen::MatrixBase<DerivedC> &q_sol, int &INFO, const IKoptions &ikoptions); 

#endif
