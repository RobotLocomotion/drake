#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "inverseKinBackend.h"

using Eigen::Map;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::systems::plants::inverseKinTrajBackend;

template <typename DerivedA, typename DerivedB, typename DerivedC,
          typename DerivedD, typename DerivedE, typename DerivedF>
DRAKEIK_EXPORT void inverseKinTraj(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<DerivedA>& qdot0_seed, const MatrixBase<DerivedB>& q_seed,
    const MatrixBase<DerivedC>& q_nom, const int num_constraints,
    RigidBodyConstraint** const constraint_array, const IKoptions& ikoptions_in,
    MatrixBase<DerivedD>* q_sol, MatrixBase<DerivedE>* qdot_sol,
    MatrixBase<DerivedF>* qddot_sol, int* info,
    std::vector<std::string>* infeasible_constraint) {
  IKoptions ikoptions = ikoptions_in;
  if (ikoptions.getFixInitialState()) {
    ikoptions.setqd0(qdot0_seed, qdot0_seed);
  }
  inverseKinTrajBackend(model, nT, t, q_seed, q_nom, num_constraints,
                        constraint_array, ikoptions, q_sol, qdot_sol, qddot_sol,
                        info, infeasible_constraint);
}

template DRAKEIK_EXPORT void inverseKinTraj(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<Map<VectorXd>>& qdot0_seed,
    const MatrixBase<Map<MatrixXd>>& q_seed,
    const MatrixBase<Map<MatrixXd>>& q_nom, const int num_constraints,
    RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions,
    MatrixBase<Map<MatrixXd>>* q_sol, MatrixBase<Map<MatrixXd>>* qdot_sol,
    MatrixBase<Map<MatrixXd>>* qddot_sol, int* info,
    std::vector<std::string>* infeasible_constraint);
template DRAKEIK_EXPORT void inverseKinTraj(
    RigidBodyTree* model, const int nT, const double* t,
    const MatrixBase<VectorXd>& qdot0_seed, const MatrixBase<MatrixXd>& q_seed,
    const MatrixBase<MatrixXd>& q_nom, const int num_constraints,
    RigidBodyConstraint** const constraint_array,
    const IKoptions& ikoptions, MatrixBase<MatrixXd>* q_sol,
    MatrixBase<MatrixXd>* qdot_sol, MatrixBase<MatrixXd>* qddot_sol,
    int* info, std::vector<std::string>* infeasible_constraint);
