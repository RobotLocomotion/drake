// Copyright 2016, Alex Dunyak
#pragma once

#include "mosekLP.h"
extern "C" {
  #include "mosek.h"  // Make sure to figure out how to put in makefile
}
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <string>


namespace drake {
namespace solvers {

static void MSKAPI printstr(void *handle,
                            MSKCONST char str[]) {
  printf("%s", str);
}

mosekLP::mosekLP(int num_variables, int num_constraints,
                 std::vector<double> equationScalars,
                 Eigen::MatrixXd cons_,
                 std::vector<MSKboundkeye> mosek_constraint_bounds_,
                 std::vector<double> upper_constraint_bounds_,
                 std::vector<double> lower_constraint_bounds_,
                 std::vector<MSKboundkeye> mosek_variable_bounds_,
                 std::vector<double> upper_variable_bounds_,
                 std::vector<double> lower_variable_bounds_) {
  numvar = num_variables;
  numcon = num_constraints;
  env = NULL;
  task = NULL;
  aptrb = NULL;  // Where ptrb[j] is the position of the first
                 // value/index in aval / asub for column j.
  aptre = NULL;  // Where ptre[j] is the position of the last
                 // value/index plus one in aval / asub for column j.
  asub = NULL;   // list of row indices
  aval = NULL;   // list of nonzero entries of A ordered by columns
  bkc = NULL;    // mosek notation bounds on constraints
  blc = NULL;
  buc = NULL;    // bounds on constraints
  bkx = NULL;    // mosek notation bounds on variables
  blx = NULL;
  bux = NULL;    // bounds on variables
  // Adapted from http://docs.mosek.com/7.1/capi/Linear_optimization.html
  r = MSK_makeenv(&env, NULL);
  if (r == MSK_RES_OK) {
    // Creates optimization task
    r = MSK_maketask(env, numcon, numvar, &task);
    // Directs log task stream to the 'printstr' function
    if (r == MSK_RES_OK)
      r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
    // Append numcon empty constraints
    if (r == MSK_RES_OK)
      r = MSK_appendcons(task, numcon);

    // Append numvar variables, initially fixed at zero
    if (r == MSK_RES_OK)
      r = MSK_appendvars(task, numvar);
  }
  // add the equation to maximize to the environment.
  int j = 0;
  for (j = 0; j < numvar && r == MSK_RES_OK; j++) {
    if (r == MSK_RES_OK)
      r = MSK_putcj(task, j, equationScalars[j]);
  }
  AddVariableBounds(mosek_variable_bounds_, upper_variable_bounds_,
      lower_variable_bounds_);
  AddLinearConstraintMatrix(cons_);
  AddLinearConstraintBounds(mosek_constraint_bounds_, upper_constraint_bounds_,
      lower_constraint_bounds_);
}

mosekLP::mosekLP(int num_variables, int num_constraints,
                 std::vector<double> equationScalars,
                 Eigen::SparseMatrix<double> sparsecons_,
                 std::vector<MSKboundkeye> mosek_constraint_bounds_,
                 std::vector<double> upper_constraint_bounds_,
                 std::vector<double> lower_constraint_bounds_,
                 std::vector<MSKboundkeye> mosek_variable_bounds_,
                 std::vector<double> upper_variable_bounds_,
                 std::vector<double> lower_variable_bounds_) {
  numvar = num_variables;
  numcon = num_constraints;
  env = NULL;
  task = NULL;
  aptrb = NULL;  // Where ptrb[j] is the position of the first
                 // value/index in aval / asub for column j.
  aptre = NULL;  // Where ptre[j] is the position of the last
                 // value/index plus one in aval / asub for column j.
  asub = NULL;   // list of row indices
  aval = NULL;   // list of nonzero entries of A ordered by columns
  bkc = NULL;    // mosek notation bounds on constraints
  blc = NULL;
  buc = NULL;    // bounds on constraints
  bkx = NULL;    // mosek notation bounds on variables
  blx = NULL;
  bux = NULL;    // bounds on variables
  // Adapted from http://docs.mosek.com/7.1/capi/Linear_optimization.html
  r = MSK_makeenv(&env, NULL);
  if (r == MSK_RES_OK) {
    // Creates optimization task
    r = MSK_maketask(env, numcon, numvar, &task);

    // Directs log task stream to the 'printstr' function
    if (r == MSK_RES_OK)
      r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

    // Append numcon empty constraints
    if (r == MSK_RES_OK)
      r = MSK_appendcons(task, numcon);

    // Append numvar variables, initially fixed at zero
    if (r == MSK_RES_OK)
      r = MSK_appendvars(task, numvar);
  }
  // add the equation to maximize to the environment.
  int j = 0;
  for (j = 0; j < numvar && r == MSK_RES_OK; j++) {
    if (r == MSK_RES_OK)
      r = MSK_putcj(task, j, equationScalars[j]);
  }
  AddVariableBounds(mosek_variable_bounds_,
      upper_variable_bounds_, lower_variable_bounds_);
  AddLinearConstraintBounds(mosek_constraint_bounds_,
      upper_constraint_bounds_, lower_constraint_bounds_);
  AddLinearConstraintSparseColumnMatrix(sparsecons_);
}

bool mosekLP::available() { return true }

void mosekLP::AddLinearConstraintMatrix(Eigen::MatrixXd cons_) {
  // Create sparse matrix vectors by column, then send them to
  // addLinearConstraintSparseColumnMatrix()
  std::vector<MSKint32t> ptrb_;
  std::vector<MSKint32t> ptre_;
  std::vector<MSKint32t> sub_;
  std::vector<double> val_;
  int i = 0, j = 0;  // iterators
  int k = 0;  // counts number of non-zero entries
  bool firstfound = false;
  Eigen::SparseMatrix<double> sparsecons_ = cons_.sparseView();
  // Send the sparse matrix rep into addLinearConstraintSparseColumnMatrix(),
  // which will handle setting the mosek constraints
  mosekLP::AddLinearConstraintSparseColumnMatrix(sparsecons_);
}

void mosekLP::AddLinearConstraintSparseColumnMatrix(
    Eigen::SparseMatrix<double> sparsecons_) {
  int j = 0;  // iterator
  // Define sparse matrix representation to be the same size as the desired
  // constraints
  free(aptrb);
  aptrb = (MSKint32t *) malloc(sizeof(MSKint32t)*sparsecons_.cols());
  free(aptre);
  aptre = (MSKint32t *) malloc(sizeof(MSKint32t)*sparsecons_.cols());
  free(asub);
  asub = (MSKint32t *) malloc(sizeof(MSKint32t)*sparsecons_.nonZeros());
  free(aval);
  aval = (double *) malloc(sizeof(double)*sparsecons_.nonZeros());

  for (j = 0; j < sparsecons_.cols(); j++)
    aptrb[j] = (MSKint32t) sparsecons_.outerIndexPtr()[j];
  for (j = 0; j < sparsecons_.cols(); j++)
    aptre[j] = (MSKint32t) sparsecons_.outerIndexPtr()[j+1];
  for (j = 0; j < sparsecons_.nonZeros(); j++)
    asub[j] = (MSKint32t) sparsecons_.innerIndexPtr()[j];
  for (j = 0; j < sparsecons_.nonZeros(); j++)
    aval[j] = sparsecons_.valuePtr()[j];

  // following code adapted from http://docs.mosek.com/7.1/capi/Linear_optimization.html
  // check if still working in valid environment
  for (j = 0; j < numvar && r == MSK_RES_OK; j++) {
    r = MSK_putacol(task,
                    j,
                    aptre[j] - aptrb[j],  // Number of nonzeros in column i
                    &asub[0] + aptrb[j],  // Pointer to row indexes of column i
                    &aval[0] + aptrb[j]);  // pointer to values of column i
  }
}

void mosekLP::AddLinearConstraintBounds(std::vector<MSKboundkeye> mosek_bounds_,
                                        std::vector<double> upper_bounds_,
                                        std::vector<double> lower_bounds_) {
  int i = 0;
  for (i; i < numcon && r == MSK_RES_OK; i++) {
    r = MSK_putconbound(task, i, mosek_bounds_[i],
                        lower_bounds_[i], upper_bounds_[i]);
  }
}

void mosekLP::AddVariableBounds(std::vector<MSKboundkeye> mosek_bounds_,
                                std::vector<double> upper_bounds_,
                                std::vector<double> lower_bounds_) {
  int j = 0;
  for (j; j < numvar && r == MSK_RES_OK; j++) {
    r = MSK_putvarbound(task, j, mosek_bounds_[j], lower_bounds_[j],
        upper_bounds_[j]);
  }
}

std::vector<MSKboundkeye> FindMosekBounds(std::vector<double> upper_bounds_,
                                          std::vector<double> lower_bounds_) {
  assert(upper_bounds_.size() == lower_bounds_.size());
  std::vector<MSKboundkeye> mosek_bounds_(upper_bounds_.size());
  int i = 0;
  for (i = 0; i < upper_bounds_.size(); i++) {
    if (upper_bounds_[i] == +MSK_INFINITY) {
      if (lower_bounds_[i] == -MSK_INFINITY) {
        mosek_bounds_.push_back(MSK_BK_FR);
      } else {
        mosek_bounds_.push_back(MSK_BK_LO);
      }
    } else {
      if (upper_bounds_[i] == lower_bounds_[i]) {
        mosek_bounds_.push_back(MSK_BK_FX);
      } else if (lower_bounds_ == -MSK_INFINITY) {
        mosek_bounds_.push_back(MSK_BK_UP);
      } else {
        mosek_bounds_.push_back(MSK_BK_RA);
      }
    }
  }
  return mosek_bounds_;
}


SolutionResult mosekLP::Solve(OptimizationProblem &prog) const {
  int j = 0;
  // construct an object that calls all the previous work so I can salvage
  // something at least.
  // assume that the problem type is linear currently.
  // TODO(adunyak): Add support for quadratic objective and constraints
  if (prog.GetSolverOptionsStr("problemtype").find("linear")
      == std::string::npos)
    return kUknownError;  // Not a linear optimization
  int totalconnum = 0, varnum = 0, i = 0;
  for (auto&& con_ : prog.linear_constraints()) {
    // check to see if the constraint references a single variable,
    // which is handles as a variable bound by mosek
    for (auto&& row : con_.constraint()->A().row(i)) {
      if (row.nonZeros() != 1)
        totalconnum++;
    }
    ++i;
  }
  Eigen::MatrixXd linear_cons_(totalconnum, prog.num_vars());
  std::vector<double> upper_constraint_bounds_(totalconnum);
  std::vector<double> lower_constraint_bounds_(totalconnum);
  std::vector<MSKboundkeye> mosek_constraint_bounds_(totalconnum);
  std::vector<MSKboundkeye> mosek_variable_bounds_(prog.num_vars());
  std::vector<double> upper_variable_bounds_(prog.num_vars());
  std::vector<double> lower_variable_bounds_(prog.num_vars());
  linear_cons_.setZero(totalconnum, prog.num_vars());
  int connum = 0;
  i = 0
  for (auto&& con_ : prog.linear_constraints()) {  // type is
                                                   // Binding<LinearConstraint>
    // construct a mapping from a variable in con_ 's VariableList
    // to the column indexes of the variables constraint points to
    std::vector<int> indexmap(con_.variable_list().size());
    i = 0;
    for (auto&& view : con_.variable_list()) {  // view is DecisionVariableView
      indexmap[i] = view.index();
      ++i;
    }
    // Address the constraint matrix directly, translating back to our original
    // variable space
    for (int i = 0; i < con_.constraint()->num_constraints(); i++) {
      if (con_.constraint()->A().row(i).nonZeros() != 1) {
        // if this if statement is entered, the constraint is not a var bound
        // and must be handled differently by mosek
        for (int j = 0; j < con_.constraint()->A().cols(); j++) {
              linear_cons_(connum, indexmap[i]) = con_.constraint()->A()(i, j);
            // lower bounds first
            if (con_.constraint()->lower_bound()(i) !=
                -std::numeric_limits<double>::infinity()) {
              lower_constraint_bounds_[connum] =
                con_.constraint()->lower_bound()(i);
            } else {
              lower_constraint_bounds_[connum] = -MSK_INFINITY;
            }
            // upper bound
            if (con_.constraint()->upper_bound()(i) !=
                +std::numeric_limits<double>::infinity()) {
              upper_constraint_bounds_[connum] =
                con_.constraint()->upper_bound()(i);
            } else {
              upper_constraint_bounds_[connum] = +MSK_INFINITY;
            }
            ++connum;
          }
      } else {
        // This is a var bound
        for (int j = 0; j < con_.constraint()->A().cols(); j++) {
          if (con_.constraint()->A()(i, j) != 0) {  // expect only one nonzero
            // upper bounds
            if (con_.constraint()->upper_bound()(i) !=
                +std::numeric_limits<double>::infinity()) {
              upper_variable_bounds_[indexmap[j]] =
                con_.constraint()->upper_bound()(i);
            } else {
              upper_variable_bounds_[indexmap[j]] = +MSK_INFINITY;
            }
            // lower bounds
            if (con_.constraint()->lower_bound()(i) !=
                -std::numeric_limits<double>::infinity()) {
              lower_variable_bounds_[indexmap[j]] =
                con_.constraint()->lower_bound()(i);
            } else {
              lower_variable_bounds_[indexmap[j]] = -MSK_INFINITY;
            }
            break;  // no need to check the rest
          }
        }
      }
    }
  }
  mosek_constraint_bounds_ = FindMosekBounds(upper_constraint_bounds_,
                                             lower_constraint_bounds_);
  mosek_variable_bounds_ = FindMosekBounds(upper_variable_bounds_,
                                           lower_constraint_bounds_);

  // find the linear objective here
  std::vector<double> linobj_(
      prog.generic_objectives().front().constraint()->A().data(),
      prog.generic_objectives().front().constraint()->A().data() +
        prog.generic_objectives().front().constraint()->A().rows() *
        prog.generic_objectives().front().constraint()->A().cols());
  mosekLP opt(prog.num_vars(),
              totalconnum,
              linobj_,
              linear_cons_,
              mosek_constraint_bounds_,
              upper_constraint_bounds_,
              lower_constraint_bounds_,
              mosek_variable_bounds_,
              upper_variable_bounds_,
              lower_variable_bounds_);
  SolutionResult s = opt.OptimizeTask(
      prog.GetSolverOptionsStr("Mosek")["maxormin"],
      prog.GetSolverOptionsStr("Mosek")["problemtype"]);
  solutions_ = opt.GetSolution();
  return s;
}

SolutionResult mosekLP::OptimizeTask(std::string maxormin, std::string ptype) {
  solutions_.clear();
  if (ptype.find("quad") != std::string::npos)
    auto problemtype = MSK_SOL_ITR;
  else if (ptype.find("linear") != std::string::npos)
    auto problemtype = MSK_SOL_BAS;
  else
    return kUknownError;
  if (r == MSK_RES_OK && maxormin == "max")
    r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE);
  else if (r == MSK_RES_OK && maxormin == "min")
    r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);
  if (r == MSK_RES_OK) {
    MSKrescodee trmcode;

    r = MSK_optimizetrm(task, &trmcode);

    // Uncomment next line to print solution summary
    // MSK_solutionsummary(task, MSK_STREAM_LOG);

    if (r == MSK_RES_OK) {
      MSKsolstae solsta;
      if (r == MSK_RES_OK) {
        r = MSK_getsolsta(task, MSK_SOL_BAS, &solsta);

        switch (solsta) {
          case MSK_SOL_STA_OPTIMAL:
          case MSK_SOL_STA_NEAR_OPTIMAL: {
            double *xx = (double*) calloc(numvar, sizeof(double));
            if (xx) {
              /* Request the basic solution. */
              MSK_getxx(task, MSK_SOL_BAS, xx);
              printf("Optimal primal solution\n");
              std::vector<double> soln;
              for (j = 0; j < numvar; ++j) {
                printf("x[%d]: %e\n", j, xx[j]);
                solutions.push_back(xx[j]);
              }
              free(xx);
              return soln;
            } else {
              r = MSK_RES_ERR_SPACE;
            }
            break;
          }
          case MSK_SOL_STA_DUAL_INFEAS_CER:
          case MSK_SOL_STA_PRIM_INFEAS_CER:
          case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
          case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER:
            printf("Primal or dual infeasibility certificate found.\n");
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
}

}  // namespace solvers
}  // namespace drake
