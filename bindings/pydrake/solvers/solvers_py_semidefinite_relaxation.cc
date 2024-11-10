#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/semidefinite_relaxation.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversSemidefiniteRelaxation(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  {
    const auto& cls_doc = doc.SemidefiniteRelaxationOptions;
    py::class_<SemidefiniteRelaxationOptions> options(
        m, "SemidefiniteRelaxationOptions", cls_doc.doc);
    options
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        .def(ParamInit<SemidefiniteRelaxationOptions>())
#pragma GCC diagnostic pop
        .def_readwrite("add_implied_linear_equality_constraints",
            &SemidefiniteRelaxationOptions::
                add_implied_linear_equality_constraints,
            cls_doc.add_implied_linear_equality_constraints.doc)
        .def_readwrite("add_implied_linear_constraints",
            &SemidefiniteRelaxationOptions::add_implied_linear_constraints,
            cls_doc.add_implied_linear_constraints.doc)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        .def_readwrite("preserve_convex_quadratic_constraints",
            &SemidefiniteRelaxationOptions::
                preserve_convex_quadratic_constraints,
            cls_doc.preserve_convex_quadratic_constraints.doc_deprecated)
#pragma GCC diagnostic pop
        .def("set_to_strongest",
            &SemidefiniteRelaxationOptions::set_to_strongest,
            cls_doc.set_to_strongest.doc)
        .def("set_to_weakest", &SemidefiniteRelaxationOptions::set_to_weakest,
            cls_doc.set_to_weakest.doc)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        .def("__repr__", [](const SemidefiniteRelaxationOptions& self) {
          return py::str(
              "SemidefiniteRelaxationOptions("
              "add_implied_linear_equality_constraints={}, "
              "add_implied_linear_constraints={}, "
              "preserve_convex_quadratic_constraints={})")
              .format(self.add_implied_linear_equality_constraints,
                  self.add_implied_linear_constraints,
                  self.preserve_convex_quadratic_constraints);
        });
#pragma GCC diagnostic pop
  }

  m.def("MakeSemidefiniteRelaxation",
      py::overload_cast<const MathematicalProgram&,
          const SemidefiniteRelaxationOptions&>(
          &solvers::MakeSemidefiniteRelaxation),
      py::arg("prog"),
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      py::arg("options") = SemidefiniteRelaxationOptions(),
#pragma GCC diagnostic pop
      doc.MakeSemidefiniteRelaxation.doc_2args);
  m.def("MakeSemidefiniteRelaxation",
      py::overload_cast<const MathematicalProgram&,
          const std::vector<symbolic::Variables>&,
          const SemidefiniteRelaxationOptions&>(
          &solvers::MakeSemidefiniteRelaxation),
      py::arg("prog"), py::arg("variable_groups"),
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      py::arg("options") = SemidefiniteRelaxationOptions(),
#pragma GCC diagnostic pop
      doc.MakeSemidefiniteRelaxation.doc_3args);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
