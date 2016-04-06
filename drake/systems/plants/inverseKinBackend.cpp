#include "drake/systems/plants/inverseKinBackend.h"

#include <Eigen/Core>

#include "drake/systems/plants/constraint/RigidBodyConstraint.h"

using Eigen::Map;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace Drake {
namespace systems {
namespace plants {

template <typename DerivedA, typename DerivedB, typename DerivedC,
          typename DerivedD, typename DerivedE>
void inverseKinBackend(
    RigidBodyTree* model_input, const int mode, const int nT_input,
    const double* t_input, const MatrixBase<DerivedA>& q_seed,
    const MatrixBase<DerivedB>& q_nom_input, const int num_constraints,
    RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions, MatrixBase<DerivedC>* q_sol,
    MatrixBase<DerivedD>* qdot_sol, MatrixBase<DerivedE>* qddot_sol, int* INFO,
    std::vector<std::string>* infeasible_constraint) {

  // Fall through to SNOPT implementation.
  inverseKinSnoptBackend(model_input, mode, nT_input, t_input, q_seed,
                         q_nom_input, num_constraints, constraint_array,
                         ikoptions, q_sol, qdot_sol, qddot_sol, INFO,
                         infeasible_constraint);
}

template void inverseKinBackend(
    RigidBodyTree* model, const int mode, const int nT, const double* t,
    const MatrixBase<Map<MatrixXd>>& q_seed,
    const MatrixBase<Map<MatrixXd>>& q_nom, const int num_constraints,
    RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<Map<MatrixXd>>* q_sol, MatrixBase<Map<MatrixXd>>* qdot_sol,
    MatrixBase<Map<MatrixXd>>* qddot_sol, int* INFO,
    std::vector<std::string>* infeasible_constraint);
template void inverseKinBackend(
    RigidBodyTree* model, const int mode, const int nT, const double* t,
    const MatrixBase<MatrixXd>& q_seed, const MatrixBase<MatrixXd>& q_nom,
    const int num_constraints, RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<MatrixXd>* q_sol, MatrixBase<MatrixXd>* qdot_sol,
    MatrixBase<MatrixXd>* qddot_sol, int* INFO,
    std::vector<std::string>* infeasible_constraint);
template void inverseKinBackend(
    RigidBodyTree* model, const int mode, const int nT, const double* t,
    const MatrixBase<Map<MatrixXd>>& q_seed,
    const MatrixBase<Map<MatrixXd>>& q_nom, const int num_constraints,
    RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<Map<MatrixXd>>* q_sol, MatrixBase<MatrixXd>* qdot_sol,
    MatrixBase<MatrixXd>* qddot_sol, int* INFO,
    std::vector<std::string>* infeasible_constraint);
template void inverseKinBackend(
    RigidBodyTree* model, const int mode, const int nT, const double* t,
    const MatrixBase<Map<VectorXd>>& q_seed,
    const MatrixBase<Map<VectorXd>>& q_nom, const int num_constraints,
    RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<Map<VectorXd>>* q_sol, MatrixBase<Map<VectorXd>>* qdot_sol,
    MatrixBase<Map<VectorXd>>* qddot_sol, int* INFO,
    std::vector<std::string>* infeasible_constraint);
template void inverseKinBackend(
    RigidBodyTree* model, const int mode, const int nT, const double* t,
    const MatrixBase<VectorXd>& q_seed, const MatrixBase<VectorXd>& q_nom,
    const int num_constraints, RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<VectorXd>* q_sol, MatrixBase<VectorXd>* qdot_sol,
    MatrixBase<VectorXd>* qddot_sol, int* INFO,
    std::vector<std::string>* infeasible_constraint);
template void inverseKinBackend(
    RigidBodyTree* model, const int mode, const int nT, const double* t,
    const MatrixBase<Map<VectorXd>>& q_seed,
    const MatrixBase<Map<VectorXd>>& q_nom, const int num_constraints,
    RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<Map<VectorXd>>* q_sol, MatrixBase<VectorXd>* qdot_sol,
    MatrixBase<VectorXd>* qddot_sol, int* INFO,
    std::vector<std::string>* infeasible_constraint);

}
}
}
