#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/ipopt_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(ipopt, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "Ipopt solver bindings for MathematicalProgram";

  py::module::import("pydrake.common.value");
  py::module::import("pydrake.solvers.mathematicalprogram");

  py::class_<IpoptSolver, SolverInterface>(
      m, "IpoptSolver", doc.IpoptSolver.doc)
      .def(py::init<>(), doc.IpoptSolver.ctor.doc)
      .def_static("id", &IpoptSolver::id, doc.IpoptSolver.id.doc);

  py::class_<IpoptSolverDetails>(
      m, "IpoptSolverDetails", doc.IpoptSolverDetails.doc)
      .def_readonly("status", &IpoptSolverDetails::status,
          doc.IpoptSolverDetails.status.doc)
      .def_readonly(
          "z_L", &IpoptSolverDetails::z_L, doc.IpoptSolverDetails.z_L.doc)
      .def_readonly(
          "z_U", &IpoptSolverDetails::z_U, doc.IpoptSolverDetails.z_U.doc)
      .def_readonly("g", &IpoptSolverDetails::g, doc.IpoptSolverDetails.g.doc)
      .def_readonly("lambda_val", &IpoptSolverDetails::lambda,
          doc.IpoptSolverDetails.lambda.doc)
      .def("ConvertStatusToString", &IpoptSolverDetails::ConvertStatusToString,
          doc.IpoptSolverDetails.ConvertStatusToString.doc);
  AddValueInstantiation<IpoptSolverDetails>(m);
}

}  // namespace pydrake
}  // namespace drake
