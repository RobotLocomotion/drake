#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_tree.h"

/**
 * inverseKin solves the inverse kinematics problem
 * min_q (q-q_nom)'*Q*(q-q_nom)
 * s.t    lb<=constraint(q)<=ub
 * @param q_seed   an nq x 1 double vector. The initial guess for the
 * optimization problem
 * @param q_nom    an nq x 1 double vector. The nominal posture that the robot
 * wants to stay close to
 * @param num_constraints     The number of RigidBodyConstraints
 * @param constraint_array    Each entry in constraint_array is a
 * RigidBodyConstraint type. Currently support QuasiStaticConstraintType,
 * PostureConstraintType, SingleTimeKinematicConstraintType and
 * SingleTimeLinearPostureConstraintType.
 * @return q_sol    an nq x 1 double vector. The optimized posture
 * @return info     = 1   The optimization is successful
 *                  = 3   The optimization is successful. But optimality is not
 * strictly satisfied
 *                  = 4   The optimization is successful. But feasibility is not
 * strictly satisfied
 *                  = 5   The optimization is successful. But SNOPT runs out of
 * iterations
 *                  = 6   The optimization is successful. But SNOPT runs out of
 * major iterations
 *                  = 12  Fails to find a solution. The linear constraints are
 * not satisfied
 *                  = 13  Fails to find a solution. The nonlinear constraints
 * are not satisfied
 *                  = 31  Fails to find a solution. The iterations limit is
 * reached. Set iterations limit in ikoptions
 *                  = 32  Fails to find a solution. The major iterations limit
 * is reached. Set major iterations limit in ikoptions
 *                  = 41  Fails to find a solution because the numerics of the
 * problem is bad.
 *                  for more information of these info, check SNOPT manual for
 * reference www.stanford.edu/group/SOL/guides/sndoc7.pdf
 * @return infeasible_constraint. When the problem is infeasible,
 * infeasible_constraint contains the name of the infeasible constraints
 * @param ikoptions    The options to set parameters of IK problem.
 */
template <typename DerivedA, typename DerivedB, typename DerivedC>
void inverseKin(
    RigidBodyTree<double>* model,
    const Eigen::MatrixBase<DerivedA>& q_seed,
    const Eigen::MatrixBase<DerivedB>& q_nom,
    const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    Eigen::MatrixBase<DerivedC>* q_sol, int* info,
    std::vector<std::string>* infeasible_constraint);

/**
 * Return type for simplified versions of IK functions.
 */
struct IKResults {
  std::vector<Eigen::VectorXd> q_sol;
  std::vector<int> info;
  std::vector<std::string> infeasible_constraints;
};

/**
 * Simplified (non-template) version of inverseKin.  Useful for
 * generating bindings to non-C++ languages.
 */
IKResults inverseKinSimple(
    RigidBodyTree<double>* model, const Eigen::VectorXd& q_seed,
    const Eigen::VectorXd& q_nom,
    const std::vector<RigidBodyConstraint*>& constraint_array,
    const IKoptions& ikoptions);


/**
 * approximateIK solves the same problem as inverseKin. But for speed reason, it
 * linearizes all constraints around q_seed, and solve a quadratic problem
 * instead of a nonlinear problem.
 * @param q_seed    an nq x 1 double vector. The initial guess. Also where to
 * linearize the constraints
 * @param q_nom     Same as in inverseKin
 * @param num_constraints   Same as in inverseKin
 * @param constraint_array  Same as in inverseKin, but do not accept
 * SingleTimeLinearPostureConstraint for the moment
 * @return q_sol    Same as in inverseKin
 * @return info     = 0 Success
 *                  = 1 Fail
 * @param ikoptions  Same as in inverseKin
 */
template <typename DerivedA, typename DerivedB, typename DerivedC>
void approximateIK(
    RigidBodyTree<double>* model,
    const Eigen::MatrixBase<DerivedA>& q_seed,
    const Eigen::MatrixBase<DerivedB>& q_nom,
    const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    Eigen::MatrixBase<DerivedC>* q_sol,
    int* info);

/**
 * inverseKinPointwise   solves inverse kinematics problem at each t[i]
 * individually
 * @param nT        The length of time samples
 * @param t         t[i] is the i'th time to solve the inverse kinematics
 * problem
 * @param q_seed    An nq x nT double matrix. q_seed.col(i) is the seed for the
 * inverse kinematics problem at t[i]
 * @param q_nom     An nq x nT double matrix. q_nom.col(i) is the nominal
 * posture for the inverse kinematics problem at t[i]
 * @param num_constraints   Same as in inverseKin
 * @param constraint_array  Same as in inverseKin
 * @return q_sol    An nq x nT double matrix. q_sol.col(i) is the solution to
 * inverse kinematics problem at t[i]
 * @return info     info[i] is the info for the inverse kinematics problem at
 * t[i]. The meaning of info[i] is explained in inverseKin
 * @return infeasible_constraint     infeasible_constraint[i] are the names of
 * infeasible constraints at t[i]
 * @param ikoptions   The options to set parameters of IK problem.
 *                    if ikoptions.sequentialSeedFlag = true, then the at time
 * t[i], if t[i-1] is solved successfully, then q_sol.col(i-1) would be used as
 * the seed for t[i]. If the solver fails to find a posture at t[i-1], then
 * q_seed.col(i) would be used as the seed for t[i]
 *                    if ikoptions.sequentialSeedFlag = false, then
 * q_seed.col(i) would always be used as the seed at t[i]
 */
template <typename DerivedA, typename DerivedB, typename DerivedC>
void inverseKinPointwise(
    RigidBodyTree<double>* model, const int nT, const double* t,
    const Eigen::MatrixBase<DerivedA>& q_seed,
    const Eigen::MatrixBase<DerivedB>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    Eigen::MatrixBase<DerivedC>* q_sol, int* info,
    std::vector<std::string>* infeasible_constraint);

/**
 * Simplified (non-template) version of inverseKinPointwise.  Useful
 * for generating bindings to non-C++ languages.
 */
IKResults inverseKinPointwiseSimple(
    RigidBodyTree<double>* model,
    const Eigen::VectorXd& t,
    const Eigen::MatrixXd& q_seed,
    const Eigen::MatrixXd& q_nom,
    const std::vector<RigidBodyConstraint*>& constraint_array,
    const IKoptions& ikoptions);

/**
 * inverseKinTraj  solves the inverse kinematics problem at all time together.
 * Try to generate a smooth trajectory by assuming cubic spline for the posture,
 * and minimize the acceleration of the interpolated trajectory
 * min_(q, qdot, qddot)
 * sum(q(t(i))-q_nom(t(i)))'*Q*(q(t(i))-q_nom(t(i)))+qdot(t(i))'*Qv*qdot(t(i))+qddot(t(i))'*Qa*qddot(t(i))
 * @param nT    The length of time samples
 * @param t     t[i] is the i'th time
 * @param q_seed   An nq x nT double matrix. q_seed.col(i) is the seed for the
 * inverse kinematics problem at t[i]
 * @param q_nom    An nq x nT double matrix. q_nom.col(i) is the seed for the
 * inverse kinematics problem at t[i]
 * @param num_constraints    The number of constraints
 * @param constraint_array   Accept all categories of RigidBodyConstraint
 * @return q_sol      An nq x nT double matrix. q_sol.col(i) is the solution
 * posture at time t[i]
 * @return qdot_sol   An nq x nT double matrix. qdot_sol.col(i) is the solution
 * posture velocity at time t[i]
 * @return qddot_sol  An nq x nT double matrix. qddot_sol.col(i) is the solution
 * posture acceleration at time t[i]
 * @return info       Same as in inverseKin
 * @return infeasible_constraint    Same as in inverseKin
 * @return ikoptions   Set parameters for inverse kinematics problem
 *         ikoptions.fixInitialState = True   The initial posture and velocity
 * at time t[0] will be fixed to the seed posture q_seed.col(0) and seed
 * velocity qdot0_seed
 *         ikoptions.fixInitialState = False   The solver will search for the
 * initial posture and velocity
 */
template <typename DerivedA, typename DerivedB, typename DerivedC,
          typename DerivedD, typename DerivedE, typename DerivedF>
void inverseKinTraj(
    RigidBodyTree<double>* model, const int nT, const double* t,
    const Eigen::MatrixBase<DerivedA>& qdot0_seed,
    const Eigen::MatrixBase<DerivedB>& q_seed,
    const Eigen::MatrixBase<DerivedC>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    Eigen::MatrixBase<DerivedD>* q_sol, Eigen::MatrixBase<DerivedE>* qdot_sol,
    Eigen::MatrixBase<DerivedF>* qddot_sol, int* info,
    std::vector<std::string>* infeasible_constraint);

// TODO(sam.creasey) This version is missing qdot_sol and qddot_sol in
// the results.
/**
 * Simplified (non-template) version of inverseKinTraj.  Useful
 * for generating bindings to non-C++ languages.
 */
IKResults inverseKinTrajSimple(
    RigidBodyTree<double>* model,
    const Eigen::VectorXd& t,
    const Eigen::MatrixXd& q_seed,
    const Eigen::MatrixXd& q_nom,
    const std::vector<RigidBodyConstraint*>& constraint_array,
    const IKoptions& ikoptions);
