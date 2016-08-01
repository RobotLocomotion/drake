// Copyright 2016, Alex Dunyak

#include "drake/solvers/MosekWrapper.h"

#include <memory>
#include <string>
#include <vector>


#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_assert.h"
#include "drake/solvers/Constraint.h"


namespace drake {
namespace solvers {

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
    const Eigen::MatrixXd& quad_cons,
    const std::vector<Eigen::MatrixXd>& sdp_objectives,
    const std::vector<Eigen::MatrixXd>& sdp_constraints,
    const std::vector<int>& sdp_cone_subscripts)
      : numvar_(static_cast<MSKint32t>(num_variables)),
        numcon_(static_cast<MSKint32t>(num_constraints)),
        env_(NULL), task_(NULL), solutions_(std::vector<double>()) {
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
    // Add fixed term, optional
    if (r_ == MSK_RES_OK && constant_eqn_term != 0)
      r_ = MSK_putcfix(task_, constant_eqn_term);
  }
  // add the equation to maximize to the environment.
  int j = 0;
  for (j = 0; j < numvar_ && r_ == MSK_RES_OK; j++) {
    if (r_ == MSK_RES_OK)
      r_ = MSK_putcj(task_, j, equation_scalars[j]);
  }
  if (!quad_objective.isZero())
    AddQuadraticObjective(quad_objective);
  if (!quad_cons.isZero())
    AddQuadraticConstraintMatrix(quad_cons);
  if (!sdp_objective.empty())
    AddSDPObjectives(sdp_objectives);
  if (!sdp_constraints.empty())
    AddSDPConstraints(sdp_constraints);
  AddVariableBounds(mosek_variable_bounds, upper_variable_bounds,
      lower_variable_bounds);
  AddLinearConstraintMatrix(linear_cons);
  AddLinearConstraintBounds(mosek_constraint_bounds, upper_constraint_bounds,
      lower_constraint_bounds);
  if (!sdp_constraints.empty())
    AppendCone(sdp_cone_subscripts);
}

void MosekWrapper::AppendCone(const std::vector<int>& sdp_cone_subscripts) {
  if (r_ == MSK_RES_OK) {
    // If no subscripts set, assume that x_0 >= sqrt(sum(x_i^2)) for i > 0.
    // Adapted from http://docs.mosek.com/7.1/capi/MSK_appendcone_.html
    if (sdp_cone_subscripts.empty()) {
      std::vector<int> empty_cone_subscripts(numvar_);
      for (int i = 0; i < numvar_; i++) {
        empty_cone_subscripts[i] = i;
      }
      r_ = MSK_appendcone(task_, MSK_CT_QUAD, 0.0, numvar_,
                          &empty_cone_subscripts[0]);
    } else {
      r_ = MSK_appendcone(task_, MSK_CT_QUAD, 0.0, numvar_,
                          &empty_cone_subscripts[0]);
    }
  }
}

void MosekWrapper::AddSDPObjectives(
    const std::vector<Eigen::MatrixXd>& sdp_objectives) {

}

void MosekWrapper::AddSDPConstraints(
    const std::vector<Eigen::MatrixXd>& sdp_constraints) {

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
  if (r_ == MSK_RES_OK)
    r_ = MSK_putqobj(task_, lowtrinonzero, &qsubi[0], &qsubj[0], &qval[0]);
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
  if (r_ == MSK_RES_OK)
    r_ = MSK_putqconk(task_, 0, lowtrinonzero, &qsubi[0], &qsubj[0], &qval[0]);
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

  // The following code is adapted from http://docs.mosek.com/7.1/capi/Linear_optimization.html
  for (j = 0; j < numvar_ && r_ == MSK_RES_OK; j++) {
    r_ = MSK_putacol(task_,
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
  for (i = 0; i < numcon_ && r_ == MSK_RES_OK; i++) {
    r_ = MSK_putconbound(task_, i, mosek_bounds[i],
                         lower_bounds[i], upper_bounds[i]);
  }
}

void MosekWrapper::AddVariableBounds(
    const std::vector<MSKboundkeye>& mosek_bounds,
    const std::vector<double>& upper_bounds,
    const std::vector<double>& lower_bounds) {
  assert(mosek_bounds.size() == lower_bounds.size());
  assert(mosek_bounds.size() == upper_bounds.size());
  for (int j = 0; j < numvar_ && r_ == MSK_RES_OK; j++) {
    r_ = MSK_putvarbound(task_, j, mosek_bounds[j], lower_bounds[j],
        upper_bounds[j]);
  }
}

std::vector<MSKboundkeye> MosekWrapper::FindMosekBounds(
    const std::vector<double>& upper_bounds,
    const std::vector<double>& lower_bounds) {
  assert(upper_bounds.size() == lower_bounds.size());
  std::vector<MSKboundkeye> mosek_bounds;
  unsigned int i = 0;
  for (i = 0; i != upper_bounds.size(); i++) {
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


SolutionResult MosekWrapper::Solve(OptimizationProblem &prog) {
  // Check that the problem type is supported.
  if (!prog.GetSolverOptionsStr("Mosek").empty()) {
    if (prog.GetSolverOptionsStr("Mosek").at("problemtype").find("linear")
            == std::string::npos &&
        prog.GetSolverOptionsStr("Mosek").at("problemtype").find("quadratic")
            == std::string::npos) {
      return kUnknownError;  // Not a linear or quadratic optimization
    }
  } else {
      return kUnknownError;
  }
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
      for (i = 0; static_cast<unsigned int>(i) <
          con.constraint()->num_constraints(); i++) {
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
            0,
            Eigen::MatrixXd(0, 0),
            Eigen::MatrixXd(0, 0));

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
    if (prog.GetSolverOptionsDouble("Mosek").count("quadraticconstant")
         == 0) {
      constant_eqn_term = 0;
    } else {
      constant_eqn_term = prog.GetSolverOptionsDouble("Mosek").at(
          "quadraticconstant");
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
  }
  return kUnknownError;
}

SolutionResult MosekWrapper::OptimizeTask(const std::string& maxormin,
                                          const std::string& ptype) {
  solutions_.clear();
  MSKsoltypee problemtype = MSK_SOL_BAS;
  if (ptype.find("quadratic") != std::string::npos)
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

Eigen::VectorXd MosekWrapper::GetEigenVectorSolutions() const {
  Eigen::VectorXd soln(numvar_);
  if (solutions_.empty())
    return soln;
  unsigned int i = 0;
  for (i = 0; i < solutions_.size(); ++i) {
    soln(i) = solutions_[i];
  }
  return soln.transpose();
}

}  // namespace solvers
}  // namespace drake
