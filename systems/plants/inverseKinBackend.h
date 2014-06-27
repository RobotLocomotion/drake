#ifndef __INVERSEKINBACKEND_H__
#define __INVERSEKINBACKEND_H__
#include <string>
#include <Eigen/StdVector>
#include <Eigen/Dense>
class RigidBodyManipulator;
class RigidBodyConstraint;
class IKoptions;

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE>
void inverseKinBackend(RigidBodyManipulator* model, const int mode, const int nT, const double* t, const Eigen::MatrixBase<DerivedA> &q_seed, const Eigen::MatrixBase<DerivedB> &q_nom, const int num_constraints, RigidBodyConstraint** const constraint_array, Eigen::MatrixBase<DerivedC> &q_sol, Eigen::MatrixBase<DerivedD> &qdot_sol, Eigen::MatrixBase<DerivedE> &qddot_sol, int* INFO, std::vector<std::string> &infeasible_constraint, const IKoptions &ikoptions);

#endif

