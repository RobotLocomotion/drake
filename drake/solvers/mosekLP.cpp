// Copyright 2016, Alex Dunyak
#pragma once

#include "drake/solvers/mosekLP.h"
extern "C" {
  #include "build/include/mosek/mosek.h"  // Make sure to figure out how to put in makefile
}
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <string>
#include <map>

#include "drake/solvers/Constraint.h"


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
  aptrb = (MSKint32t *) malloc(sizeof(MSKint32t));
  aptre = (MSKint32t *) malloc(sizeof(MSKint32t));
  asub = (MSKint32t *) malloc(sizeof(MSKint32t));
  aval = (double *) malloc(sizeof(double));
  bkc = (MSKboundkeye *) malloc(sizeof(MSKboundkeye));
  blc = (double *) malloc(sizeof(double));
  buc = (double *) malloc(sizeof(double));
  bkx = (MSKboundkeye *) malloc(sizeof(MSKboundkeye));
  blx = (double *) malloc(sizeof(double));
  bux = (double *) malloc(sizeof(double));
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
  aptrb = (MSKint32t *) malloc(sizeof(MSKint32t));
  aptre = (MSKint32t *) malloc(sizeof(MSKint32t));
  asub = (MSKint32t *) malloc(sizeof(MSKint32t));
  aval = (double *) malloc(sizeof(double));
  bkc = (MSKboundkeye *) malloc(sizeof(MSKboundkeye));
  blc = (double *) malloc(sizeof(double));
  buc = (double *) malloc(sizeof(double));
  bkx = (MSKboundkeye *) malloc(sizeof(MSKboundkeye));
  blx = (double *) malloc(sizeof(double));
  bux = (double *) malloc(sizeof(double));
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

bool mosekLP::available() const { return true; }

void mosekLP::AddLinearConstraintMatrix(Eigen::MatrixXd cons_) {
  // Create sparse matrix vectors by column, then send them to
  // addLinearConstraintSparseColumnMatrix()
  std::vector<MSKint32t> ptrb_;
  std::vector<MSKint32t> ptre_;
  std::vector<MSKint32t> sub_;
  std::vector<double> val_;
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
  for (; i < numcon && r == MSK_RES_OK; i++) {
    r = MSK_putconbound(task, i, mosek_bounds_[i],
                        lower_bounds_[i], upper_bounds_[i]);
  }
}

void mosekLP::AddVariableBounds(std::vector<MSKboundkeye> mosek_bounds_,
                                std::vector<double> upper_bounds_,
                                std::vector<double> lower_bounds_) {
  int j = 0;
  for (; j < numvar && r == MSK_RES_OK; j++) {
    r = MSK_putvarbound(task, j, mosek_bounds_[j], lower_bounds_[j],
        upper_bounds_[j]);
  }
}

std::vector<MSKboundkeye> mosekLP::FindMosekBounds(
    std::vector<double> upper_bounds_,
    std::vector<double> lower_bounds_) const {
  assert(upper_bounds_.size() == lower_bounds_.size());
  std::vector<MSKboundkeye> mosek_bounds_(upper_bounds_.size());
  int i = 0;
  for (i = 0; i < upper_bounds_.size(); i++) {
    if (upper_bounds_[i] == +std::numeric_limits<double>::infinity()) {
      if (lower_bounds_[i] == -std::numeric_limits<double>::infinity()) {
        mosek_bounds_.push_back(MSK_BK_FR);
        std::cout << "MSK_BK_FR\n";
      } else {
        mosek_bounds_.push_back(MSK_BK_LO);
        std::cout << "MSK_BK_LO\n";
      }
    } else {
      if (upper_bounds_[i] == lower_bounds_[i]) {
        mosek_bounds_.push_back(MSK_BK_FX);
        std::cout << "-MSK_BK_FX\n";
      } else if (lower_bounds_[i] == -std::numeric_limits<double>::infinity()) {
        mosek_bounds_.push_back(MSK_BK_UP);
        std::cout << "MSK_BK_UP\n";
      } else {
        mosek_bounds_.push_back(MSK_BK_RA);
        std::cout << "MSK_BK_RA\n";
      }
    }
  }
  return mosek_bounds_;
}


SolutionResult mosekLP::Solve(OptimizationProblem &prog) const {
  // construct an object that calls all the previous work so I can salvage
  // something at least.
  // assume that the problem type is linear currently.
  // TODO(adunyak): Add support for quadratic objective and constraints
  std::cout << "\nEntering Mosek solve\n" << std::flush;
  if (!prog.GetSolverOptionsStr("Mosek").empty()) {
    if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("linear")
        == std::string::npos) {
      return kUnknownError;  // Not a linear optimization
    }
  } else {
      return kUnknownError;
  }
  int totalconnum = 0, i = 0;
  std::cout << "Checking constraints\n"<< std::flush;
  for (auto&& con_ : prog.linear_constraints()) {
    // check to see if the constraint references a single variable,
    // which is handles as a variable bound by mosek
    for (i = 0; i < (con_.constraint())->A().rows(); ++i) {
      auto row = (con_.constraint())->A().row(i);
      if (row.nonZeros() != 1)
        totalconnum++;
    }
  }
  std::cout << "constraints counted, "<< totalconnum << std::endl << std::flush;
  Eigen::MatrixXd linear_cons_(totalconnum, prog.num_vars());
  std::vector<double> upper_constraint_bounds_(totalconnum);
  std::vector<double> lower_constraint_bounds_(totalconnum);
  std::vector<MSKboundkeye> mosek_constraint_bounds_(totalconnum);
  std::vector<MSKboundkeye> mosek_variable_bounds_(prog.num_vars());
  //std::vector<double> upper_variable_bounds_(prog.num_vars());
  //std::vector<double> lower_variable_bounds_(prog.num_vars());
  linear_cons_.setZero(totalconnum, prog.num_vars());
  int connum = 0;
  i = 0;

  // expect only one boundingbox constraint
  std::cout << "boundingbox\n" << std::flush;
  auto bbox = *(prog.bounding_box_constraints().front().constraint());
  std::vector<double> lower_variable_bounds_(bbox.lower_bound().data(),
      bbox.lower_bound().data() +
      bbox.lower_bound().rows() * bbox.lower_bound().cols());
  std::vector<double> upper_variable_bounds_(bbox.upper_bound().data(),
      bbox.upper_bound().data() +
      bbox.upper_bound().rows() * bbox.upper_bound().cols());
    std::cout << "boundingbox found \n";
  for (auto i : lower_variable_bounds_)
    std::cout << i << " ";
  std::cout << std::endl;
  for (auto i : upper_variable_bounds_)
    std::cout << i << " ";
  std::cout << "Mosek bounds:\n";
  mosek_variable_bounds_ = FindMosekBounds(upper_variable_bounds_,
                                         lower_constraint_bounds_);

  std::cout << std::endl << "Now try constraints\n" << std::flush;

  for (const auto& con_ : prog.linear_constraints()) {  // type is
                                                   // Binding<LinearConstraint>
    // std::cout << con_.constraint()->A() << std::flush;
    // Address the constraint matrix directly, translating back to our original
    // variable space
    for (int i = 0; i < con_.constraint()->num_constraints(); i++) {
        // if this if statement is entered, the constraint is not a var bound
        // and must be handled differently by mosek
      for (int j = 0; j < (con_.constraint())->A().cols(); j++) {
        linear_cons_(i, j) = (con_.constraint()->A())(i, j);
      }
      // lower bounds first
      lower_constraint_bounds_[connum] = (con_.constraint())->lower_bound()(i);
      // upper bound
      upper_constraint_bounds_[connum] = (con_.constraint())->upper_bound()(i);
      ++connum;
    }
  }
  std::cout << "\nMade it out???\n" << std::flush;
  mosek_constraint_bounds_ = FindMosekBounds(upper_constraint_bounds_,
                                             lower_constraint_bounds_);
  std::cout << "bounds set\n"<< std::flush;
  // find the linear objective here
  LinearConstraint *obj_ = static_cast<LinearConstraint * >(
      &(*(prog.generic_objectives().front().constraint())));
  std::vector<double> linobj_((*obj_).A().data(),
      (*obj_).A().data() + (*obj_).A().rows() * (*obj_).A().cols());
  std::cout << "sending to opt\n\n " << std::flush;
  std::cout << "Numvars: " << prog.num_vars() << '\n';
  std::cout << "Constraint num: " << totalconnum << '\n';
  std::cout << "linear objective: ";
  for (auto i : linobj_) {
    std::cout << i << " ";
  }
  std::cout << "\nConstraints:\n" << linear_cons_;
  std::cout << "\nupper_constraint_bounds:\n";
  for (auto& i : upper_constraint_bounds_) {
      if (i == std::numeric_limits<double>::infinity())
        i = +MSK_INFINITY;
  }
  for (auto i : upper_constraint_bounds_)
    std::cout << i << '\n';
  std::cout << "lower_constraint_bounds:\n";
  for (auto& i : lower_constraint_bounds_) {
    if (i == -std::numeric_limits<double>::infinity())
        i = -MSK_INFINITY;
  }
  for (auto i : lower_constraint_bounds_)
    std::cout << i << '\n';
  std::cout << "upper_var_bounds:\n";
  for (auto& i : upper_variable_bounds_) {
    if (i == std::numeric_limits<double>::infinity())
        i = +MSK_INFINITY;
  }
  for (auto i : upper_variable_bounds_)
    std::cout << i << '\n';
  std::cout << "lower_var_bounds:\n";
  for (auto& i : lower_variable_bounds_) {
    if (i == -std::numeric_limits<double>::infinity())
        i = -MSK_INFINITY;
  }
  for (auto i : lower_variable_bounds_)
    std::cout << i << '\n';
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
  std::cout << "Sent to opt for solving\n"<< std::flush;
  std::string mom = prog.GetSolverOptionsStr("Mosek").at("maxormin");
  std::string ptype = prog.GetSolverOptionsStr("Mosek").at("problemtype");
  SolutionResult s = opt.OptimizeTask(mom, ptype);
  std::cout << "task solved\n"<< std::flush;
  //*solutions_ = opt.GetSolution();
  prog.SetDecisionVariableValues(opt.GetEigenVectorSolutions());
  return s;
}

SolutionResult mosekLP::OptimizeTask(std::string maxormin, std::string ptype) {
  solutions_.clear();
  std::cout << maxormin << "\n\n" << ptype <<'\n';
  MSKsoltypee problemtype = MSK_SOL_BAS;
  if (ptype.find("quad") != std::string::npos)
    problemtype = MSK_SOL_ITR;
  else if (ptype.find("linear") != std::string::npos)
    problemtype = MSK_SOL_BAS;
  else
    return kUnknownError;
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
        r = MSK_getsolsta(task, problemtype, &solsta);

        switch (solsta) {
          case MSK_SOL_STA_OPTIMAL:
          case MSK_SOL_STA_NEAR_OPTIMAL: {
            double *xx = (double*) calloc(numvar, sizeof(double));
            if (xx) {
              /* Request the basic solution. */
              MSK_getxx(task, problemtype, xx);
              //printf("Optimal primal solution\n");
              for (int j = 0; j < numvar; ++j) {
                printf("x[%d]: %e\n", j, xx[j]);
                solutions_.push_back(xx[j]);
              }
              free(xx);
              return kSolutionFound;
            } else {
              r = MSK_RES_ERR_SPACE;
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

}  // namespace solvers
}  // namespace drake
