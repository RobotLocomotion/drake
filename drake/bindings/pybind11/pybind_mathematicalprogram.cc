#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>


#include "drake/solvers/mathematical_program.h"

namespace py = pybind11;


PYBIND11_MAKE_OPAQUE(drake::solvers::VectorXDecisionVariable);

PYBIND11_PLUGIN(_pybind_mathematicalprogram) {
  py::module m("_pybind_mathematicalprogram", "Drake MathematicalProgram Bindings");

  py::class_<drake::solvers::MathematicalProgram>(m, "MathematicalProgram")
    .def(py::init<>())
    .def("_NewContinuousVariables", (drake::solvers::VectorXDecisionVariable (drake::solvers::MathematicalProgram::*)(
            size_t,
            const std::string&)) &drake::solvers::MathematicalProgram::NewContinuousVariables,
         py::arg("rows"),
         py::arg("name") = "x")
    .def("_NewBinaryVariables", (drake::solvers::VectorXDecisionVariable (drake::solvers::MathematicalProgram::*)(
            size_t,
            const std::string&)) &drake::solvers::MathematicalProgram::NewBinaryVariables,
         py::arg("rows"),
         py::arg("name") = "x")
    .def("_AddLinearConstraint", (std::shared_ptr<drake::solvers::LinearConstraint> 
           (drake::solvers::MathematicalProgram::*)(
           const Eigen::Ref<const Eigen::MatrixXd>&,
           const Eigen::Ref<const Eigen::VectorXd>&,
           const Eigen::Ref<const Eigen::VectorXd>&,
           const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& vars))
           &drake::solvers::MathematicalProgram::AddLinearConstraint)
    .def("_AddLinearConstraint", (std::shared_ptr<drake::solvers::LinearConstraint> 
           (drake::solvers::MathematicalProgram::*)(
           const Eigen::Ref<const Eigen::RowVectorXd>&,
           double lb,
           double ub,
           const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& vars))
           &drake::solvers::MathematicalProgram::AddLinearConstraint)
    .def("_AddLinearCost", (std::shared_ptr<drake::solvers::LinearConstraint> 
           (drake::solvers::MathematicalProgram::*)(
           const Eigen::Ref<const Eigen::VectorXd>&,
           const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& vars))
           &drake::solvers::MathematicalProgram::AddLinearCost)
    .def("Solve", &drake::solvers::MathematicalProgram::Solve)
    .def("_GetSolution", [](const drake::solvers::MathematicalProgram& prog,
                            const drake::symbolic::Variable& var) {
      return prog.GetSolution(var);
    })
    .def("_GetSolution", [](const drake::solvers::MathematicalProgram& prog,
                            const drake::solvers::VectorXDecisionVariable& var) {
      return prog.GetSolution(var);
    })
    ;

  py::enum_<drake::solvers::SolutionResult>(m, "SolutionResult")
    .value("kSolutionFound", drake::solvers::SolutionResult::kSolutionFound)
    .value("kInvalidInput", drake::solvers::SolutionResult::kInvalidInput)
    .value("kInfeasibleConstraints", drake::solvers::SolutionResult::kInfeasibleConstraints)
    .value("kUnknownError", drake::solvers::SolutionResult::kUnknownError);

  py::class_<drake::symbolic::Variable>(m, "Variable")
    .def(py::init<const std::string&>());

  py::class_<drake::solvers::VectorXDecisionVariable>(m, "_VectorXDecisionVariable")
    .def(py::init<size_t>())
    .def("__len__", [](const drake::solvers::VectorXDecisionVariable& v) {
      return v.size();
    })
    .def("__getitem__", [](const drake::solvers::VectorXDecisionVariable& v, size_t i) {
      return v(i);
    }, py::return_value_policy::reference)
    .def("__setitem__", [](drake::solvers::VectorXDecisionVariable& v, size_t i, const drake::symbolic::Variable &var) {
      v(i) = var;
    })
    ;

  py::class_<drake::solvers::LinearConstraint, std::shared_ptr<drake::solvers::LinearConstraint> >(m, "LinearConstraint");

  return m.ptr();
}
