#include "drake/solvers/mosek_wrapper.h"

#include <memory>
#include <string>
#include <vector>


#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_assert.h"
#include "drake/solvers/constraint.h"


namespace drake {
namespace solvers {

// Linear programming
MosekWrapper::MosekWrapper(int num_variables, int num_constraints,
    const std::vector<double>& equation_scalars,
    const Eigen::MatrixXd& linear_cons,
    const std::vector<MSKboundkeye>& mosek_constraint_bounds,
    const std::vector<double>& upper_constraint_bounds,
    const std::vector<double>& lower_constraint_bounds,
    const std::vector<MSKboundkeye>& mosek_variable_bounds,
    const std::vector<double>& upper_variable_bounds,
    const std::vector<double>& lower_variable_bounds,
    double constant_eqn_term)
      : numvar_(static_cast<MSKint32t>(num_variables)),
        numcon_(static_cast<MSKint32t>(num_constraints)),
        numbarvar_(static_cast<MSKint32t>(0)),
        env_(NULL), task_(NULL), solutions_(std::vector<double>()) {
  result_ = MSK_makeenv(&env_, NULL);
  if (result_ == MSK_RES_OK) {
    // Creates optimization task
    result_ = MSK_maketask(env_, numcon_, numvar_, &task_);
    // Append numcon_ empty constraints
    if (result_ == MSK_RES_OK)
      result_ = MSK_appendcons(task_, numcon_);
    // Append numvar_ variables, initially fixed at zero
    if (result_ == MSK_RES_OK)
      result_ = MSK_appendvars(task_, numvar_);
    // Add fixed term, optional
    if (result_ == MSK_RES_OK && constant_eqn_term != 0)
      result_ = MSK_putcfix(task_, constant_eqn_term);
  }
  // add the equation to maximize to the environment.
  int j = 0;
  for (j = 0; j < numvar_ && result_ == MSK_RES_OK; j++) {
      result_ = MSK_putcj(task_, j, equation_scalars[j]);
  }
  AddVariableBounds(mosek_variable_bounds, upper_variable_bounds,
      lower_variable_bounds);
  if (!linear_cons.isZero()) {
    AddLinearConstraintMatrix(linear_cons);
  }
  AddLinearConstraintBounds(mosek_constraint_bounds, upper_constraint_bounds,
      lower_constraint_bounds);
}

// Quadratic Programming
MosekWrapper::MosekWrapper(int num_variables, int num_constraints,
    const std::vector<double>& equation_scalars,
    const Eigen::MatrixXd& linear_cons,
    const std::vector<MSKboundkeye>& mosek_constraint_bounds,
    const std::vector<double>& upper_constraint_bounds,
    const std::vector<double>& lower_constraint_bounds,
    const std::vector<MSKboundkeye>& mosek_variable_bounds,
    const std::vector<double>& upper_variable_bounds,
    const std::vector<double>& lower_variable_bounds,
    double constant_eqn_term,
    const Eigen::MatrixXd& quad_objective,
    const Eigen::MatrixXd& quad_cons)
      : numvar_(static_cast<MSKint32t>(num_variables)),
        numcon_(static_cast<MSKint32t>(num_constraints)),
        numbarvar_(static_cast<MSKint32t>(0)),
        env_(NULL), task_(NULL), solutions_(std::vector<double>()) {
  result_ = MSK_makeenv(&env_, NULL);
  if (result_ == MSK_RES_OK) {
    // Creates optimization task
    result_ = MSK_maketask(env_, numcon_, numvar_, &task_);
    // Append numcon_ empty constraints
    if (result_ == MSK_RES_OK)
      result_ = MSK_appendcons(task_, numcon_);
    // Append numvar_ variables, initially fixed at zero
    if (result_ == MSK_RES_OK)
      result_ = MSK_appendvars(task_, numvar_);
    // Add fixed term, optional
    if (result_ == MSK_RES_OK && constant_eqn_term != 0)
      result_ = MSK_putcfix(task_, constant_eqn_term);
  }
  // add the equation to maximize to the environment.
  int j = 0;
  for (j = 0; j < numvar_ && result_ == MSK_RES_OK; j++) {
    result_ = MSK_putcj(task_, j, equation_scalars[j]);
  }
  if (!quad_objective.isZero())
    AddQuadraticObjective(quad_objective);
  if (!quad_cons.isZero())
    AddQuadraticConstraintMatrix(quad_cons);
  AddVariableBounds(mosek_variable_bounds, upper_variable_bounds,
      lower_variable_bounds);
  if (!linear_cons.isZero()) {
    AddLinearConstraintMatrix(linear_cons);
  }
  AddLinearConstraintBounds(mosek_constraint_bounds, upper_constraint_bounds,
      lower_constraint_bounds);
}

// Semidefinite programming
MosekWrapper::MosekWrapper(int num_variables, int num_constraints,
    const std::vector<MSKboundkeye>& mosek_constraint_bounds,
    const std::vector<double>& upper_constraint_bounds,
    const std::vector<double>& lower_constraint_bounds,
    const std::vector<MSKboundkeye>& mosek_variable_bounds,
    const std::vector<double>& upper_variable_bounds,
    const std::vector<double>& lower_variable_bounds,
    double constant_eqn_term,
    const SemidefiniteConstraint& sdp_objective,
    const std::vector<SemidefiniteConstraint>& sdp_constraints,
    const std::vector<int>& lorentz_cone_subscripts,
    int numbarvar)
      : numvar_(static_cast<MSKint32t>(num_variables)),
        numcon_(static_cast<MSKint32t>(num_constraints)),
        numbarvar_(static_cast<MSKint32t>(numbarvar)),
        env_(NULL), task_(NULL), solutions_(std::vector<double>()) {
  result_ = MSK_makeenv(&env_, NULL);
  if (result_ == MSK_RES_OK) {
    // Creates optimization task
    result_ = MSK_maketask(env_, numcon_, numvar_, &task_);
    // Append numcon_ empty constraints
    if (result_ == MSK_RES_OK)
      result_ = MSK_appendcons(task_, numcon_);
    // Append numvar_ variables, initially fixed at zero
    if (result_ == MSK_RES_OK)
      result_ = MSK_appendvars(task_, numvar_);
    if (result_ == MSK_RES_OK && !(sdp_constraints.empty() ||
        sdp_objective.G().isZero())) {
      int dimbarvar = sdp_objective.G().rows();
      result_ = MSK_appendbarvars(task_, 1, &dimbarvar);
    }
    // Add fixed term, optional
    if (result_ == MSK_RES_OK && constant_eqn_term != 0)
      result_ = MSK_putcfix(task_, constant_eqn_term);
  }
  // add the equation to maximize to the environment.
  if (!sdp_objective.G().isZero())
    AddSDPObjective(sdp_objective);
  if (!sdp_constraints.empty())
    AddSDPConstraints(sdp_constraints);
  if (!sdp_constraints.empty())
    AppendLorentzCone(lorentz_cone_subscripts);
  AddVariableBounds(mosek_variable_bounds, upper_variable_bounds,
      lower_variable_bounds);
  AddLinearConstraintBounds(mosek_constraint_bounds, upper_constraint_bounds,
      lower_constraint_bounds);
}

void MosekWrapper::AppendLorentzCone(
      const std::vector<int>& lorentz_cone_subscripts) {
  if (result_ == MSK_RES_OK) {
    // If no subscripts set, assume that x_0 >= sqrt(sum(x_i^2)) for i > 0.
    // Adapted from http://docs.mosek.com/7.1/capi/MSK_appendcone_.html
    // Currently, it only handles Lorentz the Lorentz cone.
    // TODO(alexdunyak): Allow other cones as described here:
    // http://docs.mosek.com/7.1/capi/Conic_quadratic_optimization.html
    if (lorentz_cone_subscripts.empty()) {
      std::vector<int> empty_cone_subscripts(numvar_);
      for (int i = 0; i < numvar_; i++) {
        empty_cone_subscripts[i] = i;
      }
      result_ = MSK_appendcone(task_, MSK_CT_QUAD, 0.0, numvar_,
                          &empty_cone_subscripts[0]);
    } else {
      result_ = MSK_appendcone(task_, MSK_CT_QUAD, 0.0, numvar_,
                          &lorentz_cone_subscripts[0]);
    }
  }
}

void MosekWrapper::AddSDPObjective(
      const SemidefiniteConstraint& sdp_objective) {
  MSKint64t idx;  // idx is assigned to a specific sparse symmetric matrix by
                  // Mosek.
  double falpha = 1.0;  // Used for weighting variables
  // TODO(alexdunyak): Make the weight actually affect something, the docs are
  // very unhelpful.
  // First, handle the linear term in the objective.
  const Eigen::VectorXd& linearterm = sdp_objective.b();
  for (int j = 0; j < linearterm.size(); j++) {
    if (result_== MSK_RES_OK && linearterm(j) != 0)
      result_ = MSK_putcj(task_, j, linearterm(j));
  }
  // Now we handle the cost Trace(C'*x). Mosek expects it in lower triangle
  // triplet form.
  std::vector<MSKint32t> sdp_i, sdp_j;
  std::vector<double>  sdp_values;
  // The kth nonzero value of the constraint matrix is:
  // Matrix(i, j) = sdp_values[k], and sdp_i[k] = i, sdp_j[k] = j
  const Eigen::MatrixXd& matrixterm = sdp_objective.G();
  int numnonzero = 0;
  for (int i = 0; static_cast<unsigned int>(i) < matrixterm.rows(); i++) {
    for (int j = 0; j <= i; j++) {
      if (matrixterm(i, j) != 0) {
        sdp_i.push_back(static_cast<MSKint32t>(i));
        sdp_j.push_back(static_cast<MSKint32t>(j));
        sdp_values.push_back(matrixterm(i, j));
        numnonzero++;
      }
    }
  }
  if (result_ == MSK_RES_OK)
    result_ = MSK_appendsparsesymmat(task_,
                                matrixterm.rows(),
                                numnonzero,
                                &sdp_i[0],
                                &sdp_j[0],
                                &sdp_values[0],
                                &idx);
  if (result_ == MSK_RES_OK)
    result_ = MSK_putbarcj(task_, 0, 1, &idx, &falpha);
}

void MosekWrapper::AddSDPConstraints(
    const std::vector<SemidefiniteConstraint>& sdp_constraints) {
  // The linear terms are constructed by creating a matrix of all linear terms
  // and adding that row by row to Mosek.
  // See: http://docs.mosek.com/7.1/capi/Semidefinite_optimization.html
  Eigen::MatrixXd linear_cons(numcon_, numvar_);
  for (unsigned int i = 0; i < sdp_constraints.size(); i++) {
    linear_cons.row(i) = sdp_constraints[i].b().transpose();
  }
  Eigen::SparseMatrix<double> sparsecons = linear_cons.sparseView();
  // Now convert the linear constraints into sparse column matrix.
  std::vector<MSKint32t> aptrb;  // aptrb[j] is the position of the first index
                                 // in asub for column j.
  std::vector<MSKint32t> aptre;  // aptre[j] is the position of the last index
                                 // plus one in asub for column j.
  std::vector<MSKint32t> asub;  // List of row indices.
  std::vector<MSKrealt> aval;  // List of non-zero entries ordered by columns.
  int j = 0;
  for (j = 0; j < static_cast<int>(sparsecons.cols()); j++) {
    aptrb.push_back((MSKint32t) sparsecons.outerIndexPtr()[j]);
  }
  for (j = 0; j < static_cast<int>(sparsecons.cols()); j++) {
    aptre.push_back((MSKint32t) sparsecons.outerIndexPtr()[j+1]);
  }
  for (j = 0; j < static_cast<int>(sparsecons.nonZeros()); j++) {
    asub.push_back((MSKint32t) sparsecons.innerIndexPtr()[j]);
  }
  for (j = 0; j < static_cast<int>(sparsecons.nonZeros()); j++) {
    aval.push_back(static_cast<MSKrealt>(sparsecons.valuePtr()[j]));
  }
  // Now that the sparse format used by Mosek has been found, add it to Mosek
  // by row.
  for (j = 0; j < static_cast<int>(numvar_); j++) {
    if (result_ == MSK_RES_OK)
      result_ = MSK_putacol(task_,
                       j,
                       aptre[j] - aptrb[j],
                       &asub[0] + aptrb[j],
                       &aval[0] + aptrb[j]);
  }
  int currentnonzero = 0;
  std::vector<int> constraintnonzero;
  std::vector<MSKint32t> bar_i, bar_j;
  std::vector<double> bar_value;
  for (int k = 0; k < static_cast<int>(sdp_constraints.size()); k++) {
    const Eigen::MatrixXd& matrixterm = sdp_constraints[k].G();
    // Expect that each matrix is of the same dimensions and is square.
    DRAKE_ASSERT(matrixterm.rows() == matrixterm.cols());
    if (k >= 1)
      DRAKE_ASSERT(sdp_constraints[k-1].G().rows() == matrixterm.rows());
    for (int i = 0; i < static_cast<int>(matrixterm.rows()); i++) {
      for (j = 0; j <= i; j++) {
        // Only iterate over the lower triangle.
        if (matrixterm(i, j) != 0) {
          bar_i.push_back(static_cast<MSKint32t>(i));
          bar_j.push_back(static_cast<MSKint32t>(j));
          bar_value.push_back(matrixterm(i, j));
          ++currentnonzero;
        }
      }
    }
    constraintnonzero.push_back(currentnonzero);
    currentnonzero = 0;
  }
  int matrixdim = sdp_constraints[0].G().rows();
  MSKint32t previousentries = 0;
  MSKint64t idx;
  double falpha = 1.0;
  // Now that the relevant arrays are in memory, add each constraint to Mosek.
  for (int k = 0; k < static_cast<int>(sdp_constraints.size()); k++) {
    if (result_ == MSK_RES_OK) {
      result_ = MSK_appendsparsesymmat(
          task_,
          static_cast<MSKint32t>(matrixdim),
          static_cast<MSKint32t>(constraintnonzero[k]),
          &bar_i[previousentries],
          &bar_j[previousentries],
          &bar_value[previousentries],
          &idx);
    }
    if (result_ == MSK_RES_OK) {
      result_ = MSK_putbaraij(task_, k, 0, 1, &idx, &falpha);
    }
    previousentries += constraintnonzero[k];
  }
}

void MosekWrapper::AddQuadraticObjective(
    const Eigen::MatrixXd& quad_objective) {
  std::vector<int> qsubi, qsubj;
  std::vector<double> qval;
  int lowtrinonzero = 0, i = 0, j = 0;
  for (i = 0; i < quad_objective.rows(); i++) {
    for (j = 0; j <= i; j++) {
      if (quad_objective(i, j) != 0) {
        qsubi.push_back(i);
        qsubj.push_back(j);
        qval.push_back(quad_objective(i, j));
        lowtrinonzero++;
      }
    }
  }
  if (result_ == MSK_RES_OK)
    result_ = MSK_putqobj(task_, lowtrinonzero, &qsubi[0], &qsubj[0], &qval[0]);
}

void MosekWrapper::AddQuadraticConstraintMatrix(
    const Eigen::MatrixXd& quad_cons) {
  std::vector<int> qsubi, qsubj;
  std::vector<double> qval;
  int lowtrinonzero = 0, i = 0, j = 0;
  for (i = 0; i < quad_cons.rows(); i++) {
    for (j = 0; j <= i; j++) {
      if (quad_cons(i, j) != 0) {
        qsubi.push_back(i);
        qsubj.push_back(j);
        qval.push_back(quad_cons(i, j));
        lowtrinonzero++;
      }
    }
  }
  if (result_ == MSK_RES_OK)
    result_ = MSK_putqconk(task_, 0, lowtrinonzero, &qsubi[0], &qsubj[0],
                           &qval[0]);
}

void MosekWrapper::AddLinearConstraintMatrix(const Eigen::MatrixXd& cons) {
  Eigen::SparseMatrix<double> sparsecons = cons.sparseView();
  // Send the sparse matrix rep into addLinearConstraintSparseColumnMatrix(),
  // which will handle setting the mosek constraints
  MosekWrapper::AddLinearConstraintSparseColumnMatrix(sparsecons);
}

void MosekWrapper::AddLinearConstraintSparseColumnMatrix(
    const Eigen::SparseMatrix<double>& sparsecons) {
  int j = 0;  // iterator
  // Define sparse matrix representation to be the same size as the desired
  // constraints.
  std::vector<MSKint32t> aptrb;  // aptrb[j] is the position of the first index
                                 // in asub for column j.
  std::vector<MSKint32t> aptre;  // aptre[j] is the position of the last index
                                 // plus one in asub for column j.
  std::vector<MSKint32t> asub;  // List of row indices.
  std::vector<MSKrealt> aval;  // List of non-zero entries ordered by columns.

  for (j = 0; j < sparsecons.cols(); j++)
    aptrb.push_back((MSKint32t) sparsecons.outerIndexPtr()[j]);
  for (j = 0; j < sparsecons.cols(); j++)
    aptre.push_back((MSKint32t) sparsecons.outerIndexPtr()[j+1]);
  for (j = 0; j < sparsecons.nonZeros(); j++)
    asub.push_back((MSKint32t) sparsecons.innerIndexPtr()[j]);
  for (j = 0; j < sparsecons.nonZeros(); j++)
    aval.push_back(sparsecons.valuePtr()[j]);

  // The following code is adapted from http://docs.mosek.com/7.1/capi/Linear_optimization.html
  for (j = 0; j < numvar_ && result_ == MSK_RES_OK; j++) {
    result_ = MSK_putacol(task_,
                    j,
                    aptre[j] - aptrb[j],  // Number of nonzeros in column i
                    &asub[0] + aptrb[j],  // Pointer to row indexes of column i
                    &aval[0] + aptrb[j]);  // pointer to values of column i
  }
}

void MosekWrapper::AddLinearConstraintBounds(
    const std::vector<MSKboundkeye>& mosek_bounds,
    const std::vector<double>& upper_bounds,
    const std::vector<double>& lower_bounds) {
  int i = 0;
  for (i = 0; i < numcon_ && result_ == MSK_RES_OK; i++) {
    result_ = MSK_putconbound(task_, i, mosek_bounds[i],
                         lower_bounds[i], upper_bounds[i]);
  }
}

void MosekWrapper::AddVariableBounds(
    const std::vector<MSKboundkeye>& mosek_bounds,
    const std::vector<double>& upper_bounds,
    const std::vector<double>& lower_bounds) {
  assert(mosek_bounds.size() == lower_bounds.size());
  assert(mosek_bounds.size() == upper_bounds.size());
  for (int j = 0; j < numvar_ && result_ == MSK_RES_OK; j++) {
    result_ = MSK_putvarbound(task_, j, mosek_bounds[j], lower_bounds[j],
                         upper_bounds[j]);
  }
}

std::vector<MSKboundkeye> MosekWrapper::FindMosekBounds(
    const std::vector<double>& upper_bounds,
    const std::vector<double>& lower_bounds) {
  assert(upper_bounds.size() == lower_bounds.size());
  std::vector<MSKboundkeye> mosek_bounds;
  int i = 0;
  for (i = 0; i != static_cast<int>(upper_bounds.size()); i++) {
    if (upper_bounds[i] == +MSK_INFINITY) {
      if (lower_bounds[i] == -MSK_INFINITY) {
        mosek_bounds.push_back(MSK_BK_FR);
      } else {
        mosek_bounds.push_back(MSK_BK_LO);
      }
    } else {
      if (upper_bounds[i] == lower_bounds[i]) {
        mosek_bounds.push_back(MSK_BK_FX);
      } else if (lower_bounds[i] == -MSK_INFINITY) {
        mosek_bounds.push_back(MSK_BK_UP);
      } else {
        mosek_bounds.push_back(MSK_BK_RA);
      }
    }
  }
  return mosek_bounds;
}


SolutionResult MosekWrapper::Solve(MathematicalProgram &prog) {
  Eigen::MatrixXd linear_cons;
  Eigen::MatrixXd quad_cons;
  std::vector<double> upper_constraint_bounds;
  std::vector<double> lower_constraint_bounds;
  std::vector<MSKboundkeye> mosek_constraint_bounds;
  std::vector<MSKboundkeye> mosek_variable_bounds;

  // Expect exactly one boundingbox constraint.
  DRAKE_ASSERT(prog.bounding_box_constraints().size() == 1);
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

  int totalconnum = 0, connum = 0, i = 0, j = 0;
  // This block of code handles constraints, depending on if the program is
  // linear or quadratic.
  // TODO(alexdunyak): Allow constraints to affect specific variables
  if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("linear")
      != std::string::npos) {
    // Count all linear constraints.
    for (auto&& con : prog.GetAllLinearConstraints()) {
      // Check to see if the constraint references a single variable,
      // which is handled as a variable bound by mosek.
      for (i = 0; i < (con.constraint())->A().rows(); ++i) {
        auto row = (con.constraint())->A().row(i);
        if (row.nonZeros() != 1)
          totalconnum++;
      }
    }
    linear_cons.resize(totalconnum, prog.num_vars());
    quad_cons.resize(0, 0);
    upper_constraint_bounds.resize(totalconnum);
    lower_constraint_bounds.resize(totalconnum);
    for (const auto& con : prog.GetAllLinearConstraints()) {
      // con can call functions of  Binding<LinearConstraint>, but the actual
      // type is const std::list<Binding<LinearConstraint>>::const_iterator&.
      for (i = 0; (i) <
          static_cast<int>(con.constraint()->num_constraints()); i++) {
        for (j = 0; j < (con.constraint())->A().cols(); j++) {
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
    // Find the linear objective here.
    /* TODO(alexdunyak): Once there is a OptimizationProblem::AddLinearObjective
    * function, then this cast should be changed. Also be sure to assert that
    * only one type of objective is present.
    */
    DRAKE_ASSERT(prog.generic_costs().size() == 1);
    LinearConstraint *obj = dynamic_cast<LinearConstraint *>(
        prog.generic_costs().front().constraint().get());
    DRAKE_ASSERT(obj != nullptr);
    std::vector<double> linobj(obj->A().data(),
        obj->A().data() + obj->A().rows() * obj->A().cols());
    MosekWrapper opt(prog.num_vars(),
            totalconnum,
            linobj,
            linear_cons,
            mosek_constraint_bounds,
            upper_constraint_bounds,
            lower_constraint_bounds,
            mosek_variable_bounds,
            upper_variable_bounds,
            lower_variable_bounds,
            0);

    std::string mom = prog.GetSolverOptionsStr("Mosek").at("maxormin");
    std::string ptype = prog.GetSolverOptionsStr("Mosek").at("problemtype");
    SolutionResult s = opt.OptimizeTask(mom, ptype);
    prog.SetDecisionVariableValues(opt.GetEigenVectorSolutions());
    return s;
  } else if //NOLINT
      (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("quadratic")
        != std::string::npos) {
    /* QUADRATIC STUFF HERE */
    // Because there's no getter for quadratic constraints yet, we have to
    // use generic constraints and dynamically cast them to quadratic
    // constraints.
    // For now assume a single quadratic constraint.
    /* TODO(alexdunyak): Once we've added
    * OptimizationProblem::AddQuadraticConstraint and started to store quadratic
    * constraints independently of generic constraints, this cast should go away
    * and we should get the quadratic constraints from the OptimizationProblem
    * without a cast being necessary. Make sure to assert that other types of
    * constraints are not also present.
    */
    DRAKE_ASSERT(prog.generic_constraints().size() == 1);
    DRAKE_ASSERT(prog.generic_costs().size() == 0);
    QuadraticConstraint *quad_con_ptr = dynamic_cast<QuadraticConstraint *>(
        prog.generic_constraints().front().constraint().get());
    DRAKE_ASSERT(quad_con_ptr != nullptr);
    linear_cons = quad_con_ptr->b().transpose();
    quad_cons = quad_con_ptr->Q();
    upper_constraint_bounds.resize(1);
    lower_constraint_bounds.resize(1);
    mosek_constraint_bounds.resize(1);
    upper_constraint_bounds[0] = quad_con_ptr->upper_bound()(0);
    if (upper_constraint_bounds[0] == +std::numeric_limits<double>::infinity())
      upper_constraint_bounds[0] = +MSK_INFINITY;
    lower_constraint_bounds[0] = quad_con_ptr->lower_bound()(0);
    if (lower_constraint_bounds[0] == -std::numeric_limits<double>::infinity())
      lower_constraint_bounds[0] = -MSK_INFINITY;
    mosek_constraint_bounds = FindMosekBounds(upper_constraint_bounds,
                                              lower_constraint_bounds);
    double constant_eqn_term;
    if (prog.GetSolverOptionsDouble("Mosek").count("constant")
         == 0) {
      constant_eqn_term = 0;
    } else {
      constant_eqn_term = prog.GetSolverOptionsDouble("Mosek").at(
          "constant");
    }
    Eigen::MatrixXd quad_obj(0, 0);

    // Assume only one quadratic objective over the whole problem.
    DRAKE_ASSERT(prog.quadratic_costs().size() == 1);
    QuadraticConstraint *quad_obj_ptr =
        prog.quadratic_costs().front().constraint().get();
    quad_obj = (*quad_obj_ptr).Q();
    std::vector<double> linobj((*quad_obj_ptr).b().data(),
        (*quad_obj_ptr).b().data() +
        (*quad_obj_ptr).b().rows() * (*quad_obj_ptr).b().cols());
    totalconnum = 1;  // Defined for object declaration below.
    MosekWrapper opt(prog.num_vars(),
                     totalconnum,
                     linobj,
                     linear_cons,
                     mosek_constraint_bounds,
                     upper_constraint_bounds,
                     lower_constraint_bounds,
                     mosek_variable_bounds,
                     upper_variable_bounds,
                     lower_variable_bounds,
                     constant_eqn_term,
                     quad_obj,
                     quad_cons);

    std::string mom = prog.GetSolverOptionsStr("Mosek").at("maxormin");
    std::string ptype = prog.GetSolverOptionsStr("Mosek").at("problemtype");
    SolutionResult s = opt.OptimizeTask(mom, ptype);
    prog.SetDecisionVariableValues(opt.GetEigenVectorSolutions());
    return s;
  } else if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("sdp")
      != std::string::npos) {
    /* SDP STUFF HERE */
    /* TODO(alexdunyak): Add support for SDP constraints to Constraint.h and
    OptimizationProblem, including getters and setters, When those are in
    place.
    */

    // As before, assume there is one objective.
    DRAKE_ASSERT(prog.generic_costs().size() == 1);
    SemidefiniteConstraint *sdp_objective =
        dynamic_cast<SemidefiniteConstraint *>(
        prog.generic_costs().front().constraint().get());
    std::vector<SemidefiniteConstraint> sdp_constraints;
    totalconnum = prog.generic_constraints().size();
    for (const auto& sdp_con : prog.generic_constraints()) {
      SemidefiniteConstraint *sdp_con_ptr =
          dynamic_cast<SemidefiniteConstraint *>(sdp_con.constraint().get());
      SemidefiniteConstraint obj = *sdp_con_ptr;
      sdp_constraints.push_back(obj);
      if (sdp_con_ptr->upper_bound()(0) !=
          std::numeric_limits<double>::infinity())
        upper_constraint_bounds.push_back(sdp_con_ptr->upper_bound()(0));
      else
        upper_constraint_bounds.push_back(+MSK_INFINITY);
      if (sdp_con_ptr->lower_bound()(0) !=
          -std::numeric_limits<double>::infinity())
        lower_constraint_bounds.push_back(sdp_con_ptr->lower_bound()(0));
      else
        lower_constraint_bounds.push_back(-MSK_INFINITY);
    }
    mosek_constraint_bounds = FindMosekBounds(upper_constraint_bounds,
                                              lower_constraint_bounds);
    double constant_eqn_term;
    std::vector<int> lorentz_cone_subscripts;
    if (prog.GetSolverOptionsDouble("Mosek").count("constant")
         == 0) {
      constant_eqn_term = 0;
    } else {
      constant_eqn_term = prog.GetSolverOptionsDouble("Mosek").at(
          "constant");
    }
    if (prog.GetSolverOptionsInt("Mosek").count("conesubscript") != 0) {
      lorentz_cone_subscripts.push_back(
          prog.GetSolverOptionsInt("Mosek").at("conesubscript"));
      for (int k = 0; k < static_cast<int>(prog.num_vars()); k++) {
        if (lorentz_cone_subscripts[0] != k)
          lorentz_cone_subscripts.push_back(k);
      }
    }
    int numbarvar = 0;
    if (prog.GetSolverOptionsInt("Mosek").count("numbarvar") != 0)
      numbarvar = prog.GetSolverOptionsInt("Mosek").at("numbarvar");
    MosekWrapper opt(prog.num_vars() - numbarvar,
                     totalconnum,
                     mosek_constraint_bounds,
                     upper_constraint_bounds,
                     lower_constraint_bounds,
                     mosek_variable_bounds,
                     upper_variable_bounds,
                     lower_variable_bounds,
                     constant_eqn_term,
                     *sdp_objective,
                     sdp_constraints,
                     lorentz_cone_subscripts,
                     numbarvar);
    std::string mom = prog.GetSolverOptionsStr("Mosek").at("maxormin");
    std::string ptype = prog.GetSolverOptionsStr("Mosek").at("problemtype");
    SolutionResult s = opt.OptimizeTask(mom, ptype);
    // Add decision variables to hold bar variables.
    prog.SetDecisionVariableValues(opt.GetEigenVectorSolutions());
    return s;
  }
  return kUnknownError;
}

SolutionResult MosekWrapper::OptimizeTask(const std::string& maxormin,
                                          const std::string& ptype) {
  solutions_.clear();
  MSKsoltypee problemtype = MSK_SOL_BAS;
  if (ptype.find("quadratic") != std::string::npos ||
      ptype.find("sdp") != std::string::npos)
    problemtype = MSK_SOL_ITR;
  else if (ptype.find("linear") != std::string::npos)
    problemtype = MSK_SOL_BAS;
  else
    return kUnknownError;
  if (result_ == MSK_RES_OK && maxormin == "max")
    result_ = MSK_putobjsense(task_, MSK_OBJECTIVE_SENSE_MAXIMIZE);
  else if (result_ == MSK_RES_OK && maxormin == "min")
    result_ = MSK_putobjsense(task_, MSK_OBJECTIVE_SENSE_MINIMIZE);
  if (result_ == MSK_RES_OK) {
    MSKrescodee trmcode;
    result_ = MSK_optimizetrm(task_, &trmcode);
    if (result_ == MSK_RES_OK) {
      MSKsolstae solsta;
      if (result_ == MSK_RES_OK) {
        result_ = MSK_getsolsta(task_, problemtype, &solsta);
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
              if (ptype.find("sdp") != std::string::npos) {
                std::unique_ptr<double[]> barx(new double[numbarvar_]);
                MSK_getbarxj(task_, MSK_SOL_ITR, 0, barx.get());
                for (int j = 0; j < numbarvar_; ++j) {
                  solutions_.push_back(barx[j]);
                }
              }
              return kSolutionFound;
            } else {
              result_ = MSK_RES_ERR_SPACE;
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

Eigen::VectorXd MosekWrapper::GetEigenVectorSolutions() const {
  Eigen::VectorXd soln;
  if (solutions_.empty())
    return soln;
  soln.resize(solutions_.size());
  for (int i = 0; i < static_cast<int>(solutions_.size()); ++i) {
    soln(i) = solutions_[i];
  }
  return soln.transpose();
}

}  // namespace solvers
}  // namespace drake
