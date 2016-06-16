#pragma once

#include "mosekQP.h"

static void MSKAPI printstr(void *handle,
                            MSKCONST char str[]) {
  printf("%s", str);
}

mosekQP::mosekQP(int num_variables, int num_constraints,
                 std::vector<double> linear_equation_scalars_,
                 Eigen::MatrixXd quad_objective_,
                 double constant_eqn_term,
                 Eigen::MatrixXd linear_cons_,
                 Eigen::MatrixXd dense_quad_cons_,
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
    // Add fixed term, optional
    if (r == MSK_RES_OK)
      r = MSK_putcfix(task, constant_eqn_term);
  }
  // add the linear equation to optimize to the environment.
  int j = 0;
  for (j = 0; j < numvar && r == MSK_RES_OK; j++) {
    if (r == MSK_RES_OK)
      r = MSK_putcj(task, j, linear_equation_scalars_[j]);
  }
  // Add quadratic objective
  std::vector<int> qsubi, qsubj;
  std::vector<double> qval;
  int lowtrinonzero = 0, i = 0;
  for (i = 0; i < quad_objective_.rows(); i++) {
    for (j = 0; j <= i; j++) {
      if (quad_objective_(i, j) != 0) {
        qsubi.push_back(i);
        qsubj.push_back(j);
        qval.push_back(quad_objective_(i, j));
        lowtrinonzero++;
      }
    }
  }
  if (r == MSK_RES_OK)
    r = MSK_putqobj(task, lowtrinonzero, &qsubi[0], &qsubj[0], &qval[0]);
  AddVariableBounds(mosek_variable_bounds_, upper_variable_bounds_,
      lower_variable_bounds_);
  AddLinearConstraintMatrix(linear_cons_);
  AddConstraintBounds(mosek_constraint_bounds_, upper_constraint_bounds_,
      lower_constraint_bounds_);
  AddQuadraticConstraintMatrix(dense_quad_cons_);
}

mosekQP::mosekQP(int num_variables, int num_constraints,
                 std::vector<double> linear_equation_scalars_,
                 Eigen::SparseMatrix<double> sparse_quad_objective_,
                 double constant_eqn_term,
                 Eigen::SparseMatrix<double> sparse_linear_cons_,
                 Eigen::SparseMatrix<double> sparse_quad_cons_,
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

    // Add fixed term, optional
    if (r == MSK_RES_OK)
      r = MSK_putcfix(task, constant_eqn_term);
  }
  // add the linear equation to maximize to the environment.
  int j = 0;
  for (j = 0; j < numvar && r == MSK_RES_OK; j++) {
    if (r == MSK_RES_OK)
      r = MSK_putcj(task, j, linear_equation_scalars_[j]);
  }
  // Add quadratic objective
  // int rowindexiter = 0, columniter = 0;
  std::vector<int> qsubi, qsubj;
  std::vector<double> qval;
  // modified from https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  // iterating over nonzero coefficients
  int lowtrinonzero = 0;
  for (int k = 0; k < sparse_quad_objective_.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator
              it(sparse_quad_objective_, k);
              it; ++it) {
      if (it.row() >= it.col()) {  // I.e. it is lower triangular
        qval.push_back(it.value());
        qsubi.push_back(it.row());   // row index
        qsubj.push_back(it.col());   // col index (here it is equal to k)
        lowtrinonzero++;
      }
    }
  }
  if (r == MSK_RES_OK)
    r = MSK_putqobj(task, lowtrinonzero, &qsubi[0], &qsubj[0], &qval[0]);
  AddVariableBounds(mosek_variable_bounds_,
      upper_variable_bounds_, lower_variable_bounds_);
  AddConstraintBounds(mosek_constraint_bounds_,
      upper_constraint_bounds_, lower_constraint_bounds_);
  AddLinearConstraintSparseColumnMatrix(sparse_linear_cons_);
  AddQuadraticConstraintSparseColumnMatrix(sparse_quad_cons_);
}

void mosekQP::AddQuadraticConstraintSparseColumnMatrix(
    Eigen::SparseMatrix<double> sparse_quad_cons_) {
  std::vector<int> qsubi, qsubj;
  std::vector<double> qval;
  // modified from https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  // iterating over nonzero coefficients
  int lowtrinonzero = 0;
  for (int k = 0; k < sparse_quad_cons_.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sparse_quad_cons_, k);
         it; ++it) {
      if (it.row() >= it.col()) {  // I.e. it is lower triangular
        qval.push_back(it.value());
        qsubi.push_back(it.row());   // row index
        qsubj.push_back(it.col());   // col index (here it is equal to k)
        lowtrinonzero++;
      }
    }
  }
  if (r = MSK_RES_OK)
    r = MSK_putqconk(task, 0, lowtrinonzero, &qsubi[0], &qsubj[0], &qval[0]);
}

void mosekQP::AddQuadraticConstraintMatrix(Eigen::MatrixXd dense_quad_cons_) {
  std::vector<int> qsubi, qsubj;
  std::vector<double> qval;
  int lowtrinonzero = 0, i = 0, j = 0;
  for (i = 0; i < dense_quad_cons_.rows(); i++) {
    for (j = 0; j <= i; j++) {
      if (dense_quad_cons_(i, j) != 0) {
        qsubi.push_back(i);
        qsubj.push_back(j);
        qval.push_back(dense_quad_cons_(i, j));
        lowtrinonzero++;
      }
    }
  }
  if (r = MSK_RES_OK)
    r = MSK_putqconk(task, 0, lowtrinonzero, &qsubi[0], &qsubj[0], &qval[0]);
}

void mosekQP::AddLinearConstraintMatrix(Eigen::MatrixXd cons_) {
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
  mosekQP::AddLinearConstraintSparseColumnMatrix(sparsecons_);
}

void mosekQP::AddLinearConstraintSparseColumnMatrix(
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

void mosekQP::AddConstraintBounds(std::vector<MSKboundkeye> mosek_bounds_,
                                  std::vector<double> upper_bounds_,
                                  std::vector<double> lower_bounds_) {
  int i = 0;
  for (i; i < numcon && r == MSK_RES_OK; i++) {
    r = MSK_putconbound(task, i, mosek_bounds_[i], lower_bounds_[i],
      upper_bounds_[i]);
  }
}

void mosekQP::AddVariableBounds(std::vector<MSKboundkeye> mosek_bounds_,
    std::vector<double> upper_bounds_, std::vector<double> lower_bounds_) {
  int j = 0;
  for (j = 0; j < numvar && r == MSK_RES_OK; j++) {
    r = MSK_putvarbound(task, j, mosek_bounds_[j], lower_bounds_[j],
      upper_bounds_[j]);
  }
}

std::vector<double> mosekQP::OptimizeTask(std::string maxormin) {
  int j = 0;
  std::vector<double> soln;
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
        r = MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

        switch (solsta) {
          case MSK_SOL_STA_OPTIMAL:
          case MSK_SOL_STA_NEAR_OPTIMAL: {
            double *xx = (double*) calloc(numvar, sizeof(double));
            if (xx) {
              /* Request the interior point solution. */
              MSK_getxx(task, MSK_SOL_ITR, xx);
              printf("Optimal primal solution\n");
              for (j = 0; j < numvar; ++j) {
                printf("x[%d]: %e\n", j, xx[j]);
                soln.push_back(xx[j]);
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
  return soln;
}
