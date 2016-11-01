#include "drake/systems/plants/inverseKinBackend.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/plants/ConstraintWrappers.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyTree.h"

using Eigen::Map;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;

using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::solvers::Constraint;
using drake::solvers::DecisionVariableView;
using drake::solvers::MathematicalProgram;
using drake::solvers::SolutionResult;

/// NOTE: The contents of this class are for the most part direct ports of
/// drake/systems/plants/@RigidBodyManipulator/inverseKinBackend.m from Matlab
/// to C++; many methods and variables follow Matlab conventions and are
/// documented in that file.

namespace drake {
namespace systems {
namespace plants {

int GetIKSolverInfo(const MathematicalProgram& prog, SolutionResult result) {
  std::string solver_name;
  int solver_result = 0;
  prog.GetSolverResult(&solver_name, &solver_result);

  if (solver_name == "SNOPT") {
    // We can return SNOPT results directly.
    return solver_result;
  }

  // Make a SNOPT-like return code out of the generic result.
  switch (result) {
    case SolutionResult::kSolutionFound: {
      return 1;
    }
    case SolutionResult::kInvalidInput: {
      return 91;
    }
    case SolutionResult::kInfeasibleConstraints: {
      return 13;
    }
    case SolutionResult::kUnknownError: {
      return 100;  // Not a real SNOPT error.
    }
  }

  return -1;
}

void SetIKSolverOptions(const IKoptions& ikoptions,
                        drake::solvers::MathematicalProgram* prog) {
  prog->SetSolverOption("SNOPT", "Derivative option", 1);
  prog->SetSolverOption("SNOPT", "Major optimality tolerance",
                        ikoptions.getMajorOptimalityTolerance());
  prog->SetSolverOption("SNOPT", "Major feasibility tolerance",
                        ikoptions.getMajorFeasibilityTolerance());
  prog->SetSolverOption("SNOPT", "Superbasics limit",
                        ikoptions.getSuperbasicsLimit());
  prog->SetSolverOption("SNOPT", "Major iterations limit",
                        ikoptions.getMajorIterationsLimit());
  prog->SetSolverOption("SNOPT", "Iterations limit",
                        ikoptions.getIterationsLimit());
}

void AddSingleTimeLinearPostureConstraint(
    const double *t, const RigidBodyConstraint* constraint, int nq,
    const drake::solvers::DecisionVariableView& vars,
    MathematicalProgram* prog) {
  DRAKE_ASSERT(
      constraint->getCategory() ==
      RigidBodyConstraint::SingleTimeLinearPostureConstraintCategory);

  const SingleTimeLinearPostureConstraint* st_lpc =
      static_cast<const SingleTimeLinearPostureConstraint*>(constraint);
  if (!st_lpc->isTimeValid(t)) { return; }
  VectorXd lb;
  VectorXd ub;
  st_lpc->bounds(t, lb, ub);

  VectorXi iAfun;
  VectorXi jAvar;
  VectorXd A;
  st_lpc->geval(t, iAfun, jAvar, A);

  DRAKE_ASSERT(iAfun.size() == jAvar.size());
  DRAKE_ASSERT(iAfun.size() == A.size());

  typedef Eigen::Triplet<double> T;
  std::vector<T> triplet_list;
  for (int i = 0; i < iAfun.size(); i++) {
    triplet_list.push_back(T(iAfun[i], jAvar[i], A[i]));
  }

  Eigen::SparseMatrix<double> A_sparse(
      st_lpc->getNumConstraint(t), nq);
  A_sparse.setFromTriplets(triplet_list.begin(), triplet_list.end());
  prog->AddLinearConstraint(MatrixXd(A_sparse), lb, ub, {vars});
}

/// Add a single time linear posture constraint to @p prog at time @p
/// t covering @p vars.  @p nq is the KinematicsCacheHelper for the
/// underlying model.
void AddQuasiStaticConstraint(
    const double *t, const RigidBodyConstraint* constraint,
    KinematicsCacheHelper<double>* kin_helper,
    const drake::solvers::DecisionVariableView& vars,
    drake::solvers::MathematicalProgram* prog) {
  DRAKE_ASSERT(constraint->getCategory() ==
               RigidBodyConstraint::QuasiStaticConstraintCategory);

  const QuasiStaticConstraint* qsc =
      static_cast<const QuasiStaticConstraint*>(constraint);
  if (!qsc->isTimeValid(t)) { return; }
  int num_vars = qsc->getNumWeights();
  DecisionVariableView qsc_vars =
      prog->AddContinuousVariables(num_vars, "qsc");
  auto wrapper = std::make_shared<QuasiStaticConstraintWrapper>(
      qsc, kin_helper);
  prog->AddConstraint(wrapper, {vars, qsc_vars});
  prog->AddBoundingBoxConstraint(VectorXd::Constant(num_vars, 0.),
                                 VectorXd::Constant(num_vars, 1.),
                                 {qsc_vars});
  VectorXd constraint_eq(num_vars);
  constraint_eq.fill(1.);
  prog->AddLinearEqualityConstraint(constraint_eq.transpose(),
                                    Vector1d::Constant(1.), {qsc_vars});
  prog->SetInitialGuess(qsc_vars,
                        VectorXd::Constant(num_vars, 1.0 / num_vars));
}

namespace {

class InverseKinObjective : public Constraint {
 public:
  // All references are aliased for the life of the objective.
  InverseKinObjective(const RigidBodyTree* model, const MatrixXd& Q)
      : Constraint(model->get_num_positions()),
        Q_(Q) {}

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override {
    VectorXd q_err = x - q_nom_i_;
    y(0) = q_err.transpose() * Q_ * q_err;
  }

  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override {
    VectorXd x_val = autoDiffToValueMatrix(x);
    VectorXd q_err = x_val - q_nom_i_;
    VectorXd y_val = q_err.transpose() * Q_ * q_err;
    MatrixXd dy_vec = 2 * q_err.transpose() * Q_;
    auto gradient_mat = autoDiffToGradientMatrix(x);
    math::initializeAutoDiffGivenGradientMatrix(
        y_val, (dy_vec * gradient_mat).eval(), y);
  }


  /// Set the nominal posture.  This should be invoked before any
  /// calls to Eval() (the output of Eval() is undefined if this has
  /// not been set.)
  void set_q_nom(const VectorXd& q_nom_i) {
    q_nom_i_ = q_nom_i;
  }

 private:
  const MatrixXd& Q_;
  VectorXd q_nom_i_;
};

}  // anonymous namespace

template <typename DerivedA, typename DerivedB, typename DerivedC>
void inverseKinBackend(
    RigidBodyTree* model, const int nT,
    const double* t, const MatrixBase<DerivedA>& q_seed,
    const MatrixBase<DerivedB>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<DerivedC>* q_sol,
    int* info, std::vector<std::string>* infeasible_constraint) {

  // Validate some basic parameters of the input.
  if (q_seed.rows() != model->get_num_positions() || q_seed.cols() != nT ||
      q_nom.rows() != model->get_num_positions() || q_nom.cols() != nT) {
    throw std::runtime_error(
        "Drake::inverseKinBackend: q_seed and q_nom must be of size "
        "nq x nT");
  }

  KinematicsCacheHelper<double> kin_helper(model->bodies);

  // TODO(sam.creasey) I really don't like rebuilding the
  // MathematicalProgram for every timestep, but it's not possible to
  // enable/disable (or even remove) a constraint from an
  // MathematicalProgram between calls to Solve() currently, so
  // there's not actually another way.
  for (int t_index = 0; t_index < nT; t_index++) {
    MathematicalProgram prog;
    SetIKSolverOptions(ikoptions, &prog);

    DecisionVariableView vars =
        prog.AddContinuousVariables(model->get_num_positions());

    MatrixXd Q;
    ikoptions.getQ(Q);
    auto objective = std::make_shared<InverseKinObjective>(model, Q);
    prog.AddCost(objective, {vars});

    for (int i = 0; i < num_constraints; i++) {
      const RigidBodyConstraint* constraint = constraint_array[i];
      const int constraint_category = constraint->getCategory();
      if (constraint_category ==
          RigidBodyConstraint::SingleTimeKinematicConstraintCategory) {
        const SingleTimeKinematicConstraint* stc =
            static_cast<const SingleTimeKinematicConstraint*>(constraint);
        if (!stc->isTimeValid(&t[t_index])) { continue; }
        auto wrapper = std::make_shared<SingleTimeKinematicConstraintWrapper>(
            stc, &kin_helper);
        prog.AddConstraint(wrapper, {vars});
      } else if (constraint_category ==
                 RigidBodyConstraint::PostureConstraintCategory) {
        const PostureConstraint* pc =
            static_cast<const PostureConstraint*>(constraint);
        if (!pc->isTimeValid(&t[t_index])) { continue; }
        VectorXd lb;
        VectorXd ub;
        pc->bounds(&t[t_index], lb, ub);
        prog.AddBoundingBoxConstraint(lb, ub, {vars});
      } else if (
          constraint_category ==
          RigidBodyConstraint::SingleTimeLinearPostureConstraintCategory) {
        AddSingleTimeLinearPostureConstraint(
            &t[t_index], constraint, model->get_num_positions(),
            vars, &prog);
      } else if (constraint_category ==
                 RigidBodyConstraint::QuasiStaticConstraintCategory) {
        AddQuasiStaticConstraint(&t[t_index], constraint, &kin_helper,
                                 vars, &prog);
      } else if (constraint_category ==
                 RigidBodyConstraint::MultipleTimeKinematicConstraintCategory) {
        throw std::runtime_error(
            "MultipleTimeKinematicConstraint is not supported"
            " in pointwise mode.");
      } else if (
          constraint_category ==
          RigidBodyConstraint::MultipleTimeLinearPostureConstraintCategory) {
        throw std::runtime_error(
            "MultipleTimeLinearPostureConstraint is not supported"
            " in pointwise mode.");
      }
    }

    // Add a bounding box constraint from the model.
    prog.AddBoundingBoxConstraint(
        model->joint_limit_min, model->joint_limit_max,
        {vars});

    // TODO(sam.creasey) would this be faster if we stored the view
    // instead of copying into a VectorXd?
    objective->set_q_nom(q_nom.col(t_index));
    if (!ikoptions.getSequentialSeedFlag() || (t_index == 0)) {
      prog.SetInitialGuess(vars, q_seed.col(t_index));
    } else {
      prog.SetInitialGuess(vars, q_sol->col(t_index - 1));
    }

    SolutionResult result = prog.Solve();
    q_sol->col(t_index) = vars.value();
    info[t_index] = GetIKSolverInfo(prog, result);
  }
}

template void inverseKinBackend(
    RigidBodyTree* model, const int nT,
    const double* t, const MatrixBase<Map<MatrixXd>>& q_seed,
    const MatrixBase<Map<MatrixXd>>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<Map<MatrixXd>>* q_sol,
    int* info, std::vector<std::string>* infeasible_constraint);
template void inverseKinBackend(
    RigidBodyTree* model, const int nT,
    const double* t, const MatrixBase<MatrixXd>& q_seed,
    const MatrixBase<MatrixXd>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<MatrixXd>* q_sol,
    int* info, std::vector<std::string>* infeasible_constraint);
template void inverseKinBackend(
    RigidBodyTree* model, const int nT,
    const double* t, const MatrixBase<Map<VectorXd>>& q_seed,
    const MatrixBase<Map<VectorXd>>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<Map<VectorXd>>* q_sol,
    int* info, std::vector<std::string>* infeasible_constraint);
template void inverseKinBackend(
    RigidBodyTree* model, const int nT,
    const double* t, const MatrixBase<VectorXd>& q_seed,
    const MatrixBase<VectorXd>& q_nom, const int num_constraints,
    const RigidBodyConstraint* const* constraint_array,
    const IKoptions& ikoptions, MatrixBase<VectorXd>* q_sol,
    int* info, std::vector<std::string>* infeasible_constraint);

}  // namespace plants
}  // namespace systems
}  // namespace drake
