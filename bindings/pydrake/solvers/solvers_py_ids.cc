#include <string>

#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_type.h"

namespace drake {
namespace pydrake {
namespace internal {

using solvers::SolverId;
using solvers::SolverType;

void DefineSolversIds(py::module m) {
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;
  py::class_<SolverId>(m, "SolverId", doc.SolverId.doc)
      .def(py::init<std::string>(), py::arg("name"), doc.SolverId.ctor.doc)
      .def("name", &SolverId::name, doc.SolverId.name.doc)
      .def("__hash__",
          [](const SolverId& self) { return std::hash<SolverId>{}(self); })
      .def(
          "__eq__",
          [](const SolverId& self, const SolverId& other) {
            return self == other;
          },
          py::is_operator())
      .def(
          "__ne__",
          [](const SolverId& self, const SolverId& other) {
            return self != other;
          },
          py::is_operator());

  py::enum_<SolverType> solver_type(m, "SolverType", doc.SolverType.doc);
  solver_type  // BR
      .value("kClp", SolverType::kClp, doc.SolverType.kClp.doc)
      .value("kCsdp", SolverType::kCsdp, doc.SolverType.kCsdp.doc)
      .value("kEqualityConstrainedQP", SolverType::kEqualityConstrainedQP,
          doc.SolverType.kEqualityConstrainedQP.doc)
      .value("kGurobi", SolverType::kGurobi, doc.SolverType.kGurobi.doc)
      .value("kIpopt", SolverType::kIpopt, doc.SolverType.kIpopt.doc)
      .value("kLinearSystem", SolverType::kLinearSystem,
          doc.SolverType.kLinearSystem.doc)
      .value("kMobyLCP", SolverType::kMobyLCP, doc.SolverType.kMobyLCP.doc)
      .value("kMosek", SolverType::kMosek, doc.SolverType.kMosek.doc)
      .value("kNlopt", SolverType::kNlopt, doc.SolverType.kNlopt.doc)
      .value("kOsqp", SolverType::kOsqp, doc.SolverType.kOsqp.doc)
      .value("kScs", SolverType::kScs, doc.SolverType.kScs.doc)
      .value("kSnopt", SolverType::kSnopt, doc.SolverType.kSnopt.doc)
      .value("kUnrevisedLemke", SolverType::kUnrevisedLemke,
          doc.SolverType.kUnrevisedLemke.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
