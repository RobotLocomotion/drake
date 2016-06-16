#pragma once

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include <Eigen/Sparse>
extern "C" {
  #include "mosek.h"  //Make sure to figure out how to make the makefile include
}

/*Definitions and program flow taken from
http://docs.mosek.com/7.1/capi/Linear_optimization.html
and
http://docs.mosek.com/7.1/capi/Quadratic_optimization.html

For further definition of terms, see
http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
*/

class mosekQP {
  /*mosekQP
   *@brief this class allows the creation and solution of a linear programming
   *problem using the mosek solver.
   */
 public:
  /* Create a mosek quadratic programming environment using a constraint matrix
  * Takes the number of variables and constraints, the quadratic eqn to
  * optimize, the constraint matrix, and the constraint and variable bounds
  * @p environment is created and must be told to optimize.
  */
  mosekQP(int num_variables, int num_constraints,
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
          std::vector<double> lower_variable_bounds_);

  /*Create a mosek linear programming environment using a sparse column
  *linear constraint matrix and a sparse column quad constraint matrix
  *Takes the number of variables and constraints, the quadrattic eqn to
  *optimize, the constraint matrix, and the constraint and variable bounds
  *@p environment is created and must be told to optimize.
  */
  mosekQP(int num_variables, int num_constraints,
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
          std::vector<double> lower_variable_bounds_);

  ~mosekQP() {
    MSK_deletetask(&task);
    MSK_deleteenv(&env);
    delete aptrb;
    delete aptre;
    delete asub;
    delete aval;
    delete bkc;
    delete blc;
    delete buc;
    delete bkx;
    delete blx;
    delete bux;
  }

    /*optimizeTask()
   *@brief optimizes variables in given quadratic constraints and objective,
   *works with either
   *of the two previous object declarations. Accepts "max" or "min" to maximize
   *or minimize the task respectively (case sensitive).
   */
  std::vector<double> OptimizeTask(std::string maxormin);

 private:
  /*The following names are consistent with the example programs given by
   *http://docs.mosek.com/7.1/capi/Linear_optimization.html
   *and by
   *http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
   *to ensure ease of translation and understanding.
   */
   /*addLinearConstraintMatrix()
   *@brief adds linear constraints to mosek environment
   */
  void AddLinearConstraintMatrix(Eigen::MatrixXd cons_);

  /*addLinearConstraintSparseColumnMatrix()
  *@brief adds linear constraints in sparse column matrix form
  *just uses Eigen's sparse matrix library to implement expected format
  */
  void AddLinearConstraintSparseColumnMatrix(
      Eigen::SparseMatrix<double> sparsecons_);

  /*AddQuadraticConstraintMatrix()
   *@brief adds quadratic constraints to mosek environment
   */
  void AddQuadraticConstraintMatrix(Eigen::MatrixXd cons_);

  /*AddLinearConstraintSparseColumnMatrix()
  *@brief adds quadratic constraints in sparse column matrix form
  *just uses Eigen's sparse matrix library to implement expected format
  */
  void AddQuadraticConstraintSparseColumnMatrix(
      Eigen::SparseMatrix<double> sparsecons_);

  /*addLinearConstraintBounds()
  *@brief bounds constraints, see http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
  *for details on how to set mosek_bounds_
  */
  void AddConstraintBounds(std::vector<MSKboundkeye> mosek_bounds_,
                           std::vector<double> upper_bounds_,
                           std::vector<double> lower_bounds_);

  /*addVariableBounds()
   *@brief bounds variables, see http://docs.mosek.com/7.1/capi/Conventions_employed_in_the_API.html
   *for details on how to set mosek_bounds_
   */
  void AddVariableBounds(std::vector<MSKboundkeye> mosek_bounds_,
                         std::vector<double> upper_bounds_,
                         std::vector<double> lower_bounds_);


  MSKint32t numvar, numcon;
  MSKint32t* aptrb;   // Where ptrb[j] is the position of the first
                      // value/index in aval / asub for column j.
  MSKint32t*  aptre;  // Where ptre[j] is the position of the last
                      // value/index plus one in aval / asub for column j.
  MSKint32t* asub;    // list of row indices
  double* aval;       // list of nonzero entries of A ordered by columns
  MSKboundkeye* bkc;  // mosek notation bounds on constraints
  double *blc, *buc;  // bounds on constraints
  MSKboundkeye *bkx;  // mosek notation bounds on variables
  double *blx, *bux;  // bounds on variables
  MSKenv_t env;       // Internal environment, used to check if problem formed
                      // correctly
  MSKtask_t task;     // internal definition of task
  MSKrescodee r;      // used for validity checking
};
