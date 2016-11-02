#include "drake/systems/plants/inverseKinBackend.h"

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <drake/common/drake_assert.h>
#include <drake/math/autodiff.h>
#include <drake/math/autodiff_gradient.h>
#include <drake/math/gradient.h>
#include <drake/solvers/constraint.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/systems/plants/constraint/RigidBodyConstraint.h>
#include <drake/systems/plants/ConstraintWrappers.h>
#include <drake/systems/plants/IKoptions.h>
#include <drake/systems/plants/RigidBodyTree.h>

#include "drake/systems/plants/ik_trajectory_helper.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;

using drake::solvers::DecisionVariableView;
using drake::solvers::SolutionResult;
using drake::solvers::MathematicalProgram;

namespace drake {
namespace systems {
namespace plants {
namespace {

class IKTrajectoryCost : public drake::solvers::Constraint {
 public:
  /// @p helper is aliased, and must remain valid for the life of
  /// this class.
  template <typename Derived>
  IKTrajectoryCost(const IKTrajectoryHelper& helper,
                   const Eigen::MatrixBase<Derived>& q_nom)
      : Constraint(1),
        helper_(helper),
        q_nom_(q_nom) {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    throw std::runtime_error("Non-gradient version not implemented!");
  }

  void Eval(const Eigen::Ref<const drake::TaylorVecXd>& x,
            drake::TaylorVecXd& y) const override {
    const int nq = helper_.nq();
    const int nT = helper_.nT();

    const VectorXd x_scalar = drake::math::autoDiffToValueMatrix(x);
    MatrixXd q = x_scalar.head(nq * nT);
    q.resize(nq, nT);
    const VectorXd qdot0 = x_scalar.segment(nq * nT, nq);
    const VectorXd qdotf = x_scalar.segment(nq * (nT + 1), nq);
    MatrixXd dJ_vec;
    const double J = helper_.CalculateCost(q, q_nom_, 0, qdot0, qdotf,
                                           false, &dJ_vec);
    Eigen::VectorXd y_scalar(1);
    y_scalar(0) = J;
    drake::math::initializeAutoDiffGivenGradientMatrix(
        y_scalar,
        (dJ_vec * drake::math::autoDiffToGradientMatrix(x)).eval(), y);
  }

 private:
  const IKTrajectoryHelper& helper_;
  const MatrixXd q_nom_;
};

// Create a mega-constraint which handles all of the sub-constraints
// which care about the stuff which uses inbetween samples.
class IKInbetweenConstraint : public drake::solvers::Constraint {
 public:
  /// All pointers/references are aliased, and must remain valid for
  /// the life of this class.
  IKInbetweenConstraint(const RigidBodyTree* model,
                        const IKTrajectoryHelper& helper,
                        int num_constraints,
                        const RigidBodyConstraint* const* constraint_array)
      : Constraint(0),  // Update bounds later in constructor.
        model_(model),
        helper_(helper),
        num_constraints_(num_constraints),
        constraint_array_(constraint_array) {
    const int nT = helper.nT();
    const double* t = helper.t();

    // It's important that the order of evaluation for the constraints
    // be the same in here where the bounds are being determined as
    // when the constraints are being evaluated below.  This
    // meta-constraint does all of the SingleTimeKinematicConstraints
    // first, iterating by time samples in the outer loop and
    // constraints in the inner loop.  This reduces recalculation of
    // the KinematicsCache when calculating the constraint values
    // later.  After all bounds/values are determined for the single
    // time versions, we append all MultipleTimeKinematicConstraints.
    for (int i = 0; i < nT - 1; i++) {
      for (int j = 0; j < helper.t_inbetween()[i].size(); j++) {
        double t_j = helper.t_inbetween()[i](j) + t[i];
        for (int k = 0; k < num_constraints; k++) {
          const RigidBodyConstraint* constraint = constraint_array_[k];
          const int constraint_category = constraint->getCategory();
          if (constraint_category !=
              RigidBodyConstraint::SingleTimeKinematicConstraintCategory) {
            continue;
          }
          const SingleTimeKinematicConstraint* stc =
              static_cast<const SingleTimeKinematicConstraint*>(constraint);
          // Check to see if an inbetween constraint would be active at
          // this timestep.
          const int stc_nc = stc->getNumConstraint(&t_j);
          if (stc_nc > 0) {
            VectorXd lb;
            VectorXd ub;
            stc->bounds(&t_j, lb, ub);
            AppendBounds(lb, ub);
          }
        }
      }
    }

    for (int k = 0; k < num_constraints; k++) {
      const RigidBodyConstraint* constraint = constraint_array_[k];
      const int constraint_category = constraint->getCategory();
      if (constraint_category !=
          RigidBodyConstraint::MultipleTimeKinematicConstraintCategory) {
        continue;
      }
      const MultipleTimeKinematicConstraint* mtkc =
          static_cast<const MultipleTimeKinematicConstraint*>(constraint);
      int mtk_nc = mtkc->getNumConstraint(t, nT);
      if (mtk_nc > 0) {
        VectorXd lb;
        VectorXd ub;
        mtkc->bounds(helper.t_samples().data(), helper.t_samples().size(),
                     lb, ub);
        AppendBounds(lb, ub);
      }
    }
  }

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    throw std::runtime_error("Non-gradient version not implemented!");
  }

  void Eval(const Eigen::Ref<const drake::TaylorVecXd>& x,
            drake::TaylorVecXd& y) const override {
    const int nq = helper_.nq();
    const int nT = helper_.nT();
    const std::vector<Eigen::VectorXd>& t_inbetween =
        helper_.t_inbetween();

    // TODO(sam.creasey): These are a little confusing now that they
    // don't change depending on if the initial state is fixed...
    const int num_qfree = helper_.num_qfree();
    DRAKE_ASSERT(num_qfree == nT);
    const int num_qdotfree = helper_.num_qdotfree();
    DRAKE_ASSERT(x.size() == nq * (nT + num_qdotfree));

    const VectorXd x_scalar = drake::math::autoDiffToValueMatrix(x);

    // Make a local copy of the q portion of the X input values so
    // that we can resize it for convenience later.
    MatrixXd q = x_scalar.head(nq * nT);

    const auto qdot0 = x_scalar.segment(nq * nT, nq);
    const auto qdotf = x_scalar.segment(nq * (nT + 1), nq);

    // Create the return values.  After evaluating our constraints,
    // build the TaylorVec output from the scalar values in y_scalar
    // and dy_scalar.
    VectorXd y_scalar(num_constraints());
    MatrixXd dy_scalar(num_constraints(), x.size());

    // Index into y_scalar/dy_scalar where the next constraint should
    // be stored.
    int y_idx = 0;

    const int num_inbetween_tsamples = helper_.t_samples().size() - nT;
    // q data for only the times inbetween the steps
    MatrixXd q_inbetween = MatrixXd::Zero(nq, num_inbetween_tsamples);
    // q data for all timesteps (original and inbetween)
    MatrixXd q_samples = MatrixXd::Zero(nq, num_inbetween_tsamples + nT);
    int inbetween_idx = 0;

    const double* t = helper_.t();

    // Evaluate all of our single time constraints while building up
    // the q_inbetween and q_samples data for use later.
    for (int i = 0; i < nT - 1; i++) {
      q.resize(nq * nT, 1);
      MatrixXd q_inbetween_block_tmp =
          helper_.dq_inbetween_dqknot()[i] * q +
          helper_.dq_inbetween_dqd0()[i] * qdot0 +
          helper_.dq_inbetween_dqdf()[i] * qdotf;
      q_inbetween_block_tmp.resize(nq, t_inbetween[i].size());
      q_inbetween.block(0, inbetween_idx, nq, t_inbetween[i].size()) =
          q_inbetween_block_tmp;
      q.resize(nq, nT);
      for (int j = 0; j < t_inbetween[i].size(); j++) {
        double t_j = t_inbetween[i](j) + t[i];
        double* qi = q_inbetween.data() + (inbetween_idx + j) * nq;
        Eigen::Map<VectorXd> qvec(qi, nq);
        KinematicsCache<double> cache = model_->doKinematics(qvec);
        for (int k = 0; k < num_constraints_; k++) {
          const RigidBodyConstraint* constraint = constraint_array_[k];
          const int constraint_category = constraint->getCategory();
          if (constraint_category !=
              RigidBodyConstraint::SingleTimeKinematicConstraintCategory) {
            continue;
          }

          const SingleTimeKinematicConstraint* stc =
              static_cast<const SingleTimeKinematicConstraint*>(constraint);
          if (stc->isTimeValid(&t_j)) {
            int nc = stc->getNumConstraint(&t_j);
            VectorXd c_k(nc);
            MatrixXd dc_k(nc, nq);
            stc->eval(&t_j, cache, c_k, dc_k);
            y_scalar.segment(y_idx, nc) = c_k;

            MatrixXd dc_kdx =
                MatrixXd::Zero(nc, nq * (num_qfree + num_qdotfree));
            dc_kdx.block(0, 0, nc, nq * num_qfree) =
                dc_k *
                helper_.dq_inbetween_dqknot()[i].block(
                    nq * j, 0, nq, nq * num_qfree);

            dc_kdx.block(0, nq * num_qfree, nc, nq) =
                dc_k * helper_.dq_inbetween_dqd0()[i].block(nq * j, 0, nq, nq);
            dc_kdx.block(0, nq * num_qfree + nq, nc, nq) =
                dc_k * helper_.dq_inbetween_dqdf()[i].block(nq * j, 0, nq, nq);

            dy_scalar.block(y_idx, 0, nc, x.size()) = dc_kdx;
            y_idx += nc;
          }
        }
      }

      // We've evaluated all single time constraints for this
      // timestep.  Build up more of q_samples.
      q_samples.col(inbetween_idx + i) = q.col(i);
      q_samples.block(0, inbetween_idx + i + 1, nq, t_inbetween[i].size()) =
          q_inbetween.block(0, inbetween_idx, nq, t_inbetween[i].size());
      inbetween_idx += static_cast<int>(t_inbetween[i].size());
    }
    q_samples.col(nT + num_inbetween_tsamples - 1) = q.col(nT - 1);

    // Using the q_samples assembled above which includes the
    // inbetween data, evaluate any multiple time kinematic
    // constraints.
    for (int i = 0; i < num_constraints_; i++) {
      const RigidBodyConstraint* constraint = constraint_array_[i];
      const int constraint_category = constraint->getCategory();
      if (constraint_category !=
          RigidBodyConstraint::MultipleTimeKinematicConstraintCategory) {
        continue;
      }
      const MultipleTimeKinematicConstraint* mtkc =
            static_cast<const MultipleTimeKinematicConstraint*>(constraint);
      const int nc = mtkc->getNumConstraint(helper_.t_samples().data(),
                                            helper_.t_samples().size());
      VectorXd mtkc_c(nc);
      MatrixXd mtkc_dc(nc, nq * (num_qfree + num_inbetween_tsamples));
      DRAKE_ASSERT(static_cast<int>(helper_.t_samples().size()) ==
                   num_qfree + num_inbetween_tsamples);
      mtkc->eval(
          helper_.t_samples().data(), helper_.t_samples().size(),
          q_samples.block(0, 0, nq, num_qfree + num_inbetween_tsamples),
          mtkc_c, mtkc_dc);
      y_scalar.segment(y_idx, nc) = mtkc_c;

      // Calculate our gradient output which should be the size of the
      // input data (timesteps not including the inbetween times).
      MatrixXd mtkc_dc_dx =
          MatrixXd::Zero(nc, nq * (num_qfree + num_qdotfree));
      int mtkc_dc_off = 0;

      // Initialize with the gradient outputs at the original timesteps.
      for (int j = 0; j < nT; j++) {
        mtkc_dc_dx.block(0, j * nq, nc, nq) =
            mtkc_dc.block(0, mtkc_dc_off * nq, nc, nq);
        if (j != nT - 1) {
          mtkc_dc_off += 1 + t_inbetween[j].size();
        }
      }

      // Iterate over each intermediate timestamp again, integrating
      // the gradient results from the intermediate timesteps into our
      // output (and building up the qdot0/qdotf gradients as well).
      mtkc_dc_off = 0;
      for (int j = 0; j < nT - 1; j++) {
        MatrixXd dc_ij = mtkc_dc.block(0, nq * (mtkc_dc_off + 1),
                                       nc, nq * t_inbetween[j].size());
        mtkc_dc_off += 1 + t_inbetween[j].size();

        mtkc_dc_dx.block(0, 0, nc, nq * num_qfree) +=
            dc_ij *
            helper_.dq_inbetween_dqknot()[j].block(
                0, 0, nq * t_inbetween[j].size(), nq * num_qfree);
        mtkc_dc_dx.block(0, nq * num_qfree, nc, nq) +=
            dc_ij * helper_.dq_inbetween_dqd0()[j];
        mtkc_dc_dx.block(0, nq * num_qfree + nq, nc, nq) +=
            dc_ij * helper_.dq_inbetween_dqdf()[j];
      }

      dy_scalar.block(y_idx, 0, nc, x.size()) = mtkc_dc_dx;
      y_idx += nc;
    }

    y.resize(y_idx);
    drake::math::initializeAutoDiffGivenGradientMatrix(
        y_scalar, (dy_scalar * drake::math::autoDiffToGradientMatrix(x)), y);
  }

 private:
  void AppendBounds(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
    DRAKE_ASSERT(lb.size() == ub.size());
    int prev_size = lower_bound().size();
    Eigen::VectorXd new_lb(prev_size + lb.size());
    Eigen::VectorXd new_ub(prev_size + ub.size());
    new_lb.head(prev_size) = lower_bound();
    new_lb.tail(lb.size()) = lb;
    new_ub.head(prev_size) = upper_bound();
    new_ub.tail(ub.size()) = ub;
    set_bounds(new_lb, new_ub);
  }

  const RigidBodyTree* model_;
  const IKTrajectoryHelper& helper_;
  const int num_constraints_;
  const RigidBodyConstraint* const* constraint_array_;
};

}  // anonymous namespace

template <typename DerivedA, typename DerivedB, typename DerivedC,
          typename DerivedD, typename DerivedE>
void inverseKinTrajBackend(
    RigidBodyTree *model, const int nT,
    const double *t,
    const Eigen::MatrixBase<DerivedA>& q_seed,
    const Eigen::MatrixBase<DerivedB>& q_nom,
    int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    Eigen::MatrixBase<DerivedC>* q_sol,
    Eigen::MatrixBase<DerivedD>* qdot_sol,
    Eigen::MatrixBase<DerivedE>* qddot_sol, int *info,
    std::vector<std::string>* infeasible_constraint) {

  DRAKE_ASSERT(q_sol->cols() == nT);
  DRAKE_ASSERT(qdot_sol->cols() == nT);
  DRAKE_ASSERT(qddot_sol->cols() == nT);

  std::vector<double> dt(nT - 1);
  for (int j = 0; j < nT - 1; j++) {
    dt[j] = t[j + 1] - t[j];
  }

  std::vector<double> dt_ratio(nT - 2);
  for (int j = 0; j < nT - 2; j++) {
    dt_ratio[j] = dt[j] / dt[j + 1];
  }

  const int nq = model->get_num_positions();

  IKTrajectoryHelper helper(nq, nT, t, nT, 2,
                            ikoptions, dt.data(), dt_ratio.data());

  MathematicalProgram prog;
  SetIKSolverOptions(ikoptions, &prog);

  // Create our decision variables.  "q" represents all positions of
  // the model at each timestep in nT.  "qdot0" and "qdotf" are qdot
  // at the initial and final timestep.
  DecisionVariableView q = prog.AddContinuousVariables(nT * nq, "q");
  DecisionVariableView qdot0 = prog.AddContinuousVariables(nq, "qdot0");
  DecisionVariableView qdotf = prog.AddContinuousVariables(nq, "qdotf");

  std::shared_ptr<drake::solvers::Constraint> cost =
      std::make_shared<IKTrajectoryCost>(helper, q_nom);
  prog.AddCost(cost, {q, qdot0, qdotf});

  // Bound all of our positions by the model joint limits.
  VectorXd q0_lb(nq);
  VectorXd q0_ub(nq);
  ikoptions.getq0(q0_lb, q0_ub);
  Eigen::VectorXd joint_limit_min(nq * nT);
  Eigen::VectorXd joint_limit_max(nq * nT);
  for (int i = 0; i < nT; i++) {
    joint_limit_min.segment(nq * i, nq) = q0_lb;
    joint_limit_max.segment(nq * i, nq) = q0_ub;
  }

  bool fix_initial_state = ikoptions.getFixInitialState();
  if (fix_initial_state) {
    // If q0 is fixed, set a bounding box around the initial values.
    joint_limit_min.head(nq) = q_seed.col(0);
    joint_limit_max.head(nq) = q_seed.col(0);
  }
  prog.AddBoundingBoxConstraint(joint_limit_min, joint_limit_max, {q});
  Eigen::MatrixXd q_initial_guess = q_seed;
  q_initial_guess.resize(nq * nT, 1);
  prog.SetInitialGuess(q, q_initial_guess);

  // Apply the appropriate bounding box to qdot0.  If the initial
  // state is fixed, set the bounding box to be the same as the
  // initial guess.
  VectorXd qd0_lb(nq);
  VectorXd qd0_ub(nq);
  ikoptions.getqd0(qd0_lb, qd0_ub);
  VectorXd qd0_seed = (qd0_lb + qd0_ub) / 2;
  if (fix_initial_state) {
    prog.AddBoundingBoxConstraint(qd0_seed, qd0_seed, {qdot0});
  } else {
    prog.AddBoundingBoxConstraint(qd0_lb, qd0_ub, {qdot0});
  }
  prog.SetInitialGuess(qdot0, qd0_seed);

  // Bound qdotf and set our initial guess.
  VectorXd qdf_lb(nq);
  VectorXd qdf_ub(nq);
  ikoptions.getqdf(qdf_lb, qdf_ub);
  VectorXd qdf_seed = (qdf_lb + qdf_ub) / 2;
  prog.AddBoundingBoxConstraint(qdf_lb, qdf_ub, {qdotf});
  prog.SetInitialGuess(qdotf, qdf_seed);

  // TODO(sam.creasey) Consider making the kinematics cache helper
  // cache more than one entry (like maybe nT +
  // num_inbetween_tSamples).  It's hard to tell how much we're losing
  // here, but my guess from a quick look at callgrind is maybe a 10%
  // speed boost?  It could be more if there are no inbetween sample
  // times (which effectively cache internally by doing all of the
  // constraints for the same time inbetween sample at once).
  KinematicsCacheHelper<double> kin_helper(model->bodies);

  // Add all of our single time and quasi static constraints.
  int qstart_idx = 0;
  if (fix_initial_state) {
    qstart_idx = 1;
  }

  // Iterate over all of our constraints, and add them to our
  // MathematicalProgram.  For single time constraints, we add
  // multiple constraints for each timestep when the constraint is
  // valid, using the subset of the "q" decision variable representing
  // the state at that time.
  for (int i = 0; i < num_constraints; i++) {
    const RigidBodyConstraint* constraint = constraint_array[i];
    const int constraint_category = constraint->getCategory();
    if (constraint_category ==
        RigidBodyConstraint::SingleTimeKinematicConstraintCategory) {
      const SingleTimeKinematicConstraint* stc =
          static_cast<const SingleTimeKinematicConstraint*>(constraint);
      auto wrapper = std::make_shared<SingleTimeKinematicConstraintWrapper>(
          stc, &kin_helper);
      for (int t_index = qstart_idx; t_index < nT; t_index++) {
        if (!stc->isTimeValid(&t[t_index])) { continue; }
        prog.AddConstraint(wrapper, {q.segment(nq * t_index, nq)});
      }
    } else if (constraint_category ==
               RigidBodyConstraint::PostureConstraintCategory) {
      const PostureConstraint* pc =
          static_cast<const PostureConstraint*>(constraint);
      for (int t_index = qstart_idx; t_index < nT; t_index++) {
        if (!pc->isTimeValid(&t[t_index])) { continue; }
        VectorXd lb;
        VectorXd ub;
        pc->bounds(&t[t_index], lb, ub);
        prog.AddBoundingBoxConstraint(lb, ub, {q.segment(nq * t_index , nq)});
      }
    } else if (
        constraint_category ==
        RigidBodyConstraint::SingleTimeLinearPostureConstraintCategory) {
      for (int t_index = qstart_idx; t_index < nT; t_index++) {
        AddSingleTimeLinearPostureConstraint(
            &t[t_index], constraint, nq, q.segment(nq * t_index , nq),
            &prog);
      }
    } else if (constraint_category ==
               RigidBodyConstraint::QuasiStaticConstraintCategory) {
      for (int t_index = qstart_idx; t_index < nT; t_index++) {
        AddQuasiStaticConstraint(&t[t_index], constraint, &kin_helper,
                                 q.segment(nq * t_index , nq), &prog);
      }
    } else if (
        constraint_category ==
        RigidBodyConstraint::MultipleTimeLinearPostureConstraintCategory) {
      const MultipleTimeLinearPostureConstraint* mt_lpc =
          static_cast<const MultipleTimeLinearPostureConstraint*>(constraint);
      const int num_constraint = mt_lpc->getNumConstraint(t, nT);
      if (num_constraint == 0) { continue; }

      VectorXd lb;
      VectorXd ub;
      mt_lpc->bounds(t, nT, lb, ub);

      VectorXi iAfun;
      VectorXi jAvar;
      VectorXd A;
      mt_lpc->geval(t, nT, iAfun, jAvar, A);
      typedef Eigen::Triplet<double> T;
      std::vector<T> triplet_list;
      for (int j = 0; j < iAfun.size(); j++) {
          triplet_list.push_back(T(iAfun[j], jAvar[j], A[j]));
      }
      Eigen::SparseMatrix<double> A_sparse(num_constraint, nq * nT);
      A_sparse.setFromTriplets(triplet_list.begin(), triplet_list.end());
      prog.AddLinearConstraint(MatrixXd(A_sparse), lb, ub, {q});
    }
  }

  // Build an additional constraint to handle the "inbetween" samples
  // (if present).
  std::shared_ptr<drake::solvers::Constraint> inbetween_constraint =
      std::make_shared<IKInbetweenConstraint>(
          model, helper, num_constraints, constraint_array);
  if (inbetween_constraint->num_constraints() > 0) {
    prog.AddConstraint(inbetween_constraint, {q, qdot0, qdotf});
  }

  const SolutionResult result = prog.Solve();
  *info = GetIKSolverInfo(prog, result);

  // Populate the output arguments.
  const auto q_value = q.value();
  q_sol->resize(nq, nT);
  for (int i = 0; i < nT; i++) {
    q_sol->col(i) = q_value.segment(i * nq, nq);
  }

  qdot_sol->resize(nq, nT);
  qdot_sol->block(0, 0, nq, 1) = qdot0.value();
  qdot_sol->block(0, nT - 1, nq, 1) = qdotf.value();
  MatrixXd q_sol_tmp = *q_sol;
  q_sol_tmp.resize(nq * nT, 1);
  if (nT > 2) {
    MatrixXd qdot_sol_tmp = helper.velocity_mat() * q_sol_tmp;
    qdot_sol_tmp.resize(nq, nT - 2);
    qdot_sol->block(0, 1, nq, nT - 2) = qdot_sol_tmp;
  }
  MatrixXd qddot_sol_tmp(nq * nT, 1);
  qddot_sol_tmp =
      helper.accel_mat() * q_sol_tmp +
      helper.accel_mat_qd0() * qdot0.value() +
      helper.accel_mat_qdf() * qdotf.value();
  qddot_sol_tmp.resize(nq, nT);
  (*qddot_sol) = qddot_sol_tmp;
}

template void inverseKinTrajBackend(
    RigidBodyTree* model, const int nT, const double* t,
    const Eigen::MatrixBase<Eigen::Map<MatrixXd>>& q_seed,
    const Eigen::MatrixBase<Eigen::Map<MatrixXd>>& q_nom,
    const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    Eigen::MatrixBase<Eigen::Map<MatrixXd>>* q_sol,
    Eigen::MatrixBase<Eigen::Map<MatrixXd>>* qdot_sol,
    Eigen::MatrixBase<Eigen::Map<MatrixXd>>* qddot_sol, int* info,
    std::vector<std::string>* infeasible_constraint);
template void inverseKinTrajBackend(
    RigidBodyTree* model, const int nT, const double* t,
    const Eigen::MatrixBase<MatrixXd>& q_seed,
    const Eigen::MatrixBase<MatrixXd>& q_nom,
    const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions,
    Eigen::MatrixBase<MatrixXd>* q_sol,
    Eigen::MatrixBase<MatrixXd>* qdot_sol,
    Eigen::MatrixBase<MatrixXd>* qddot_sol, int* info,
    std::vector<std::string>* infeasible_constraint);

}  // namespace plants
}  // namespace systems
}  // namespace drake
