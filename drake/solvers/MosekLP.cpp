// Copyright 2016, Alex Dunyak

#include "drake/solvers/MosekLP.h"

extern "C" {
  #include <mosek/mosek.h>
}

#include <memory>
#include <vector>
#include <iostream>
#include <string>
#include <map>

#include <Eigen/Sparse>
#include <Eigen/Core>

#include "drake/solvers/Constraint.h"


namespace drake {
namespace solvers {

MosekLP::MosekLP(int num_variables, int num_constraints,
    std::vector<double> equationScalars,
    Eigen::MatrixXd cons,
    std::vector<MSKboundkeye> mosek_constraint_bounds,
    std::vector<double> upper_constraint_bounds,
    std::vector<double> lower_constraint_bounds,
    std::vector<MSKboundkeye> mosek_variable_bounds,
    std::vector<double> upper_variable_bounds,
    std::vector<double> lower_variable_bounds) {
  numvar_ = num_variables;
  numcon_ = num_constraints;
  env_ = NULL;
  task_ = NULL;
  solutions_.clear();
  r_ = MSK_makeenv(&env_, NULL);
  if (r_ == MSK_RES_OK) {
    // Creates optimization task
    r_ = MSK_maketask(env_, numcon_, numvar_, &task_);
    // Append numcon_ empty constraints
    if (r_ == MSK_RES_OK)
      r_ = MSK_appendcons(task_, numcon_);

    // Append numvar_ variables, initially fixed at zero
    if (r_ == MSK_RES_OK)
      r_ = MSK_appendvars(task_, numvar_);
  }
  // add the equation to maximize to the environment.
  int j = 0;
  for (j = 0; j < numvar_ && r_ == MSK_RES_OK; j++) {
    if (r_ == MSK_RES_OK)
      r_ = MSK_putcj(task_, j, equationScalars[j]);
  }
  AddVariableBounds(mosek_variable_bounds, upper_variable_bounds,
      lower_variable_bounds);
  AddLinearConstraintMatrix(cons);
  AddLinearConstraintBounds(mosek_constraint_bounds, upper_constraint_bounds,
      lower_constraint_bounds);
}

void MosekLP::AddLinearConstraintMatrix(const Eigen::MatrixXd& cons) {
  Eigen::SparseMatrix<double> sparsecons = cons.sparseView();
  // Send the sparse matrix rep into addLinearConstraintSparseColumnMatrix(),
  // which will handle setting the mosek constraints
  MosekLP::AddLinearConstraintSparseColumnMatrix(sparsecons);
}

void MosekLP::AddLinearConstraintSparseColumnMatrix(
    const Eigen::SparseMatrix<double>& sparsecons) {
  int j = 0;  // iterator
  // Define sparse matrix representation to be the same size as the desired
  // constraints
  std::vector<MSKint32t> aptrb;
  std::vector<MSKint32t> aptre;
  std::vector<MSKint32t> asub;
  std::vector<double> aval;

  for (j = 0; j < sparsecons.cols(); j++)
    aptrb.push_back((MSKint32t) sparsecons.outerIndexPtr()[j]);
  for (j = 0; j < sparsecons.cols(); j++)
    aptre.push_back((MSKint32t) sparsecons.outerIndexPtr()[j+1]);
  for (j = 0; j < sparsecons.nonZeros(); j++)
    asub.push_back((MSKint32t) sparsecons.innerIndexPtr()[j]);
  for (j = 0; j < sparsecons.nonZeros(); j++)
    aval.push_back(sparsecons.valuePtr()[j]);

  // following code adapted from http://docs.mosek.com/7.1/capi/Linear_optimization.html
  // check if still working in valid environment
  for (j = 0; j < numvar_ && r_ == MSK_RES_OK; j++) {
    r_ = MSK_putacol(task_,
                    j,
                    aptre[j] - aptrb[j],  // Number of nonzeros in column i
                    &asub[0] + aptrb[j],  // Pointer to row indexes of column i
                    &aval[0] + aptrb[j]);  // pointer to values of column i
  }
}

void MosekLP::AddLinearConstraintBounds(
    const std::vector<MSKboundkeye>& mosek_bounds_,
    const std::vector<double>& upper_bounds_,
    const std::vector<double>& lower_bounds_) {
  int i = 0;
  for (; i < numcon_ && r_ == MSK_RES_OK; i++) {
    r_ = MSK_putconbound(task_, i, mosek_bounds_[i],
                        lower_bounds_[i], upper_bounds_[i]);
  }
}

void MosekLP::AddVariableBounds(const std::vector<MSKboundkeye>& mosek_bounds_,
                                const std::vector<double>& upper_bounds_,
                                const std::vector<double>& lower_bounds_) {
  int j = 0;
  for (; j < numvar_ && r_ == MSK_RES_OK; j++) {
    r_ = MSK_putvarbound(task_, j, mosek_bounds_[j], lower_bounds_[j],
        upper_bounds_[j]);
  }
}

std::vector<MSKboundkeye> MosekLP::FindMosekBounds(
    const std::vector<double>& upper_bounds_,
    const std::vector<double>& lower_bounds_) const {
  assert(upper_bounds_.size() == lower_bounds_.size());
  std::vector<MSKboundkeye> mosek_bounds_;
  unsigned int i = 0;
  for (i = 0; i != upper_bounds_.size(); i++) {
    if (upper_bounds_[i] == +MSK_INFINITY) {
      if (lower_bounds_[i] == -MSK_INFINITY) {
        mosek_bounds_.push_back(MSK_BK_FR);
      } else {
        mosek_bounds_.push_back(MSK_BK_LO);
      }
    } else {
      if (upper_bounds_[i] == lower_bounds_[i]) {
        mosek_bounds_.push_back(MSK_BK_FX);
      } else if (lower_bounds_[i] == -MSK_INFINITY) {
        mosek_bounds_.push_back(MSK_BK_UP);
      } else {
        mosek_bounds_.push_back(MSK_BK_RA);
      }
    }
  }
  return mosek_bounds_;
}


SolutionResult MosekLP::Solve(OptimizationProblem &prog) const {
  // construct an object that calls all the previous work so I can salvage
  // something at least.
  // assume that the problem type is linear currently.
  // TODO(adunyak): Add support for quadratic objective and constraints
  if (!prog.GetSolverOptionsStr("Mosek").empty()) {
    if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("linear")
        == std::string::npos) {
      return kUnknownError;  // Not a linear optimization
    }
  } else {
      return kUnknownError;
  }
  int totalconnum = 0, i = 0;
  for (auto&& con_ : prog.GetAllLinearConstraints()) {
    // check to see if the constraint references a single variable,
    // which is handles as a variable bound by mosek
    for (i = 0; i < (con_.constraint())->A().rows(); ++i) {
      auto row = (con_.constraint())->A().row(i);
      if (row.nonZeros() != 1)
        totalconnum++;
    }
  }
  Eigen::MatrixXd linear_cons(totalconnum, prog.num_vars());
  std::vector<double> upper_constraint_bounds(totalconnum);
  std::vector<double> lower_constraint_bounds(totalconnum);
  std::vector<MSKboundkeye> mosek_constraint_bounds(totalconnum);
  std::vector<MSKboundkeye> mosek_variable_bounds(prog.num_vars());
  std::vector<double> upper_variable_bounds(numvar_);
  std::vector<double> lower_variable_bounds(numvar_);
  linear_cons.setZero(totalconnum, prog.num_vars());

  // Expect only one boundingbox constraint or no such constraint.
  if (!(prog.bounding_box_constraints().empty())) {
    assert(&(prog.bounding_box_constraints().front()) ==
        &(prog.bounding_box_constraints().back()));
    auto bbox = *(prog.bounding_box_constraints().front().constraint());
    std::vector<double> lower_variable_bounds(bbox.lower_bound().data(),
        bbox.lower_bound().data() +
        bbox.lower_bound().rows() * bbox.lower_bound().cols());
    std::vector<double> upper_variable_bounds(bbox.upper_bound().data(),
        bbox.upper_bound().data() +
        bbox.upper_bound().rows() * bbox.upper_bound().cols());
    for (auto& b : lower_variable_bounds) {
      if (b == -std::numeric_limits<double>::infinity())
        b = -MSK_INFINITY;
    }
    for (auto& b : upper_variable_bounds) {
      if (b == +std::numeric_limits<double>::infinity())
        b = MSK_INFINITY;
    }
    mosek_variable_bounds = FindMosekBounds(upper_variable_bounds,
                                           lower_variable_bounds);
  } else {
    // No bounding box constraint
    for (i = 0; i < numvar_; i++) {
      upper_variable_bounds[i] = +MSK_INFINITY;
      lower_variable_bounds[i] = -MSK_INFINITY;
    }
    mosek_variable_bounds = FindMosekBounds(upper_variable_bounds,
                                            lower_variable_bounds);
  }
  int connum = 0;
  i = 0;
  // TODO(alexdunyak): Allow constraints to affect specific variables
  for (const auto& con : prog.GetAllLinearConstraints()) {
    // con can call functions of  Binding<LinearConstraint>, but the actual
    // type is const std::list<Binding<LinearConstraint>>::const_iterator&
    // Address the constraint matrix directly, translating back to our original
    // variable space
    for (int i = 0; (unsigned int) i < con.constraint()->num_constraints();
        i++) {
      for (int j = 0; j < (con.constraint())->A().cols(); j++) {
        linear_cons(connum, j) = (con.constraint()->A())(i, j);
      }
      // lower bounds first
      lower_constraint_bounds[connum] =
          (con.constraint())->lower_bound()(i);
      if (lower_constraint_bounds[connum] ==
          -std::numeric_limits<double>::infinity())
        lower_constraint_bounds[connum] = -MSK_INFINITY;
      // upper bound
      upper_constraint_bounds[connum] =
          (con.constraint())->upper_bound()(i);
      if (upper_constraint_bounds[connum] ==
          +std::numeric_limits<double>::infinity())
        upper_constraint_bounds[connum] = +MSK_INFINITY;
      connum++;
    }
  }
  mosek_constraint_bounds = FindMosekBounds(upper_constraint_bounds,
                                            lower_constraint_bounds);
  // find the linear objective here
  LinearConstraint *obj_ = static_cast<LinearConstraint *>(
      &(*(prog.generic_costs().front().constraint())));
  std::vector<double> linobj_((*obj_).A().data(),
      (*obj_).A().data() + (*obj_).A().rows() * (*obj_).A().cols());
  MosekLP opt(prog.num_vars(),
              totalconnum,
              linobj_,
              linear_cons,
              mosek_constraint_bounds,
              upper_constraint_bounds,
              lower_constraint_bounds,
              mosek_variable_bounds,
              upper_variable_bounds,
              lower_variable_bounds);
  std::string mom = prog.GetSolverOptionsStr("Mosek").at("maxormin");
  std::string ptype = prog.GetSolverOptionsStr("Mosek").at("problemtype");
  SolutionResult s = opt.OptimizeTask(mom, ptype);
  prog.SetDecisionVariableValues(opt.GetEigenVectorSolutions());
  return s;
}

SolutionResult MosekLP::OptimizeTask(const std::string& maxormin,
                                     const std::string& ptype) {
  solutions_.clear();
  MSKsoltypee problemtype = MSK_SOL_BAS;
  if (ptype.find("quad") != std::string::npos)
    problemtype = MSK_SOL_ITR;
  else if (ptype.find("linear") != std::string::npos)
    problemtype = MSK_SOL_BAS;
  else
    return kUnknownError;
  if (r_ == MSK_RES_OK && maxormin == "max")
    r_ = MSK_putobjsense(task_, MSK_OBJECTIVE_SENSE_MAXIMIZE);
  else if (r_ == MSK_RES_OK && maxormin == "min")
    r_ = MSK_putobjsense(task_, MSK_OBJECTIVE_SENSE_MINIMIZE);
  if (r_ == MSK_RES_OK) {
    MSKrescodee trmcode;

    r_ = MSK_optimizetrm(task_, &trmcode);
    if (r_ == MSK_RES_OK) {
      MSKsolstae solsta;
      if (r_ == MSK_RES_OK) {
        r_ = MSK_getsolsta(task_, problemtype, &solsta);

        switch (solsta) {
          case MSK_SOL_STA_OPTIMAL:
          case MSK_SOL_STA_NEAR_OPTIMAL: {
            std::unique_ptr<double[]> xx(new double[numvar_]);
            if (xx) {
              /* Request the basic solution. */
              MSK_getxx(task_, problemtype, xx.get());
              for (int j = 0; j < numvar_; ++j) {
                solutions_.push_back(xx[j]);
              }
              return kSolutionFound;
            } else {
              r_ = MSK_RES_ERR_SPACE;
              return kUnknownError;
            }
            break;
          }
          case MSK_SOL_STA_DUAL_INFEAS_CER:
          case MSK_SOL_STA_PRIM_INFEAS_CER:
          case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
          case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
            printf("Primal or dual infeasibility certificate found.\n");
            return kInfeasibleConstraints;
            break;
          case MSK_SOL_STA_UNKNOWN: {
            char symname[MSK_MAX_STR_LEN];
            char desc[MSK_MAX_STR_LEN];

            /* If the solutions status is unknown, print the termination code
               indicating why the optimizer terminated prematurely. */
            MSK_getcodedesc(trmcode, symname, desc);
            printf("The solution status is unknown.\n");
            printf("The optimizer terminitated with code: %s\n", symname);
            break;
          }
          default:
            printf("Other solution status.\n");
            break;
        }
      }
    }
  }
  return kUnknownError;
}

Eigen::VectorXd MosekLP::GetEigenVectorSolutions() const {
  Eigen::VectorXd soln(numvar_);
  if (solutions_.empty())
    return soln;
  int i = 0;
  for (i = 0; (unsigned int) i < solutions_.size(); ++i) {
    soln(i) = solutions_[i];
  }
  return soln.transpose();
}

}  // namespace solvers
}  // namespace drake
