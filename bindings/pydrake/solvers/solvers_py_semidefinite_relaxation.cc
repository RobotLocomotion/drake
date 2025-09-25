#include <vector>

#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/semidefinite_relaxation.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversSemidefiniteRelaxation(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;

  {
    const auto& cls_doc = doc.SemidefiniteRelaxationOptions;
    py::class_<SemidefiniteRelaxationOptions> options(
        m, "SemidefiniteRelaxationOptions", cls_doc.doc);
    options.def(ParamInit<SemidefiniteRelaxationOptions>())
        .def_readwrite("add_implied_linear_equality_constraints",
            &SemidefiniteRelaxationOptions::
                add_implied_linear_equality_constraints,
            cls_doc.add_implied_linear_equality_constraints.doc)
        .def_readwrite("add_implied_linear_constraints",
            &SemidefiniteRelaxationOptions::add_implied_linear_constraints,
            cls_doc.add_implied_linear_constraints.doc);
    options
        .def("set_to_strongest",
            &SemidefiniteRelaxationOptions::set_to_strongest,
            cls_doc.set_to_strongest.doc)
        .def("set_to_weakest", &SemidefiniteRelaxationOptions::set_to_weakest,
            cls_doc.set_to_weakest.doc)
        .def("__repr__", [](const SemidefiniteRelaxationOptions& self) {
          return py::str(
              "SemidefiniteRelaxationOptions("
              "add_implied_linear_equality_constraints={}, "
              "add_implied_linear_constraints={})")
              .format(self.add_implied_linear_equality_constraints,
                  self.add_implied_linear_constraints);
        });
  }

  m.def("MakeSemidefiniteRelaxation",
      py::overload_cast<const MathematicalProgram&,
          const SemidefiniteRelaxationOptions&>(
          &solvers::MakeSemidefiniteRelaxation),
      py::arg("prog"), py::arg("options") = SemidefiniteRelaxationOptions{},
      doc.MakeSemidefiniteRelaxation.doc_2args);
  m.def("MakeSemidefiniteRelaxation",
      py::overload_cast<const MathematicalProgram&,
          const std::vector<symbolic::Variables>&,
          const SemidefiniteRelaxationOptions&>(
          &solvers::MakeSemidefiniteRelaxation),
      py::arg("prog"), py::arg("variable_groups"),
      py::arg("options") = SemidefiniteRelaxationOptions{},
      doc.MakeSemidefiniteRelaxation.doc_3args);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
