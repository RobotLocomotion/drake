#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversOptions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  py::enum_<CommonSolverOption>(
      m, "CommonSolverOption", doc.CommonSolverOption.doc)
      .value("kPrintFileName", CommonSolverOption::kPrintFileName,
          doc.CommonSolverOption.kPrintFileName.doc)
      .value("kPrintToConsole", CommonSolverOption::kPrintToConsole,
          doc.CommonSolverOption.kPrintToConsole.doc);

  py::class_<SolverOptions>(m, "SolverOptions", doc.SolverOptions.doc)
      .def(py::init<>())
      .def("SetOption",
          py::overload_cast<const SolverId&, std::string,
              SolverOptions::OptionValue>(&SolverOptions::SetOption),
          py::arg("solver_id"), py::arg("key"), py::arg("value"),
          doc.SolverOptions.SetOption.doc_3args)
      .def("SetOption",
          py::overload_cast<CommonSolverOption, SolverOptions::OptionValue>(
              &SolverOptions::SetOption),
          py::arg("key"), py::arg("value"),
          doc.SolverOptions.SetOption.doc_2args)
      .def("get_print_file_name", &SolverOptions::get_print_file_name,
          doc.SolverOptions.get_print_file_name.doc)
      .def("get_print_to_console", &SolverOptions::get_print_to_console,
          doc.SolverOptions.get_print_to_console.doc)
      .def("__repr__",
          [](const SolverOptions& self) { return fmt::to_string(self); });

#if 0
  // XXX dep kwarg name
      .def("SetOption",
          py::overload_cast<const SolverId&, std::string, SolverOptions::OptionValue>(
              &SolverOptions::SetOption),
          py::arg("solver_id"), py::arg("solver_option"),
          py::arg("option_value"),
          doc.SolverOptions.SetOption.doc_double_option)
      .def(
          "GetOptions",
          [](const SolverOptions& solver_options, SolverId solver_id) {
            py::dict out;
            py::object update = out.attr("update");
            update(solver_options.GetOptionsDouble(solver_id));
            update(solver_options.GetOptionsInt(solver_id));
            update(solver_options.GetOptionsStr(solver_id));
            return out;
          },
          py::arg("solver_id"), doc.SolverOptions.GetOptionsDouble.doc)
    // XXX
      .def("common_solver_options", &SolverOptions::common_solver_options,
          doc.SolverOptions.common_solver_options.doc)
#endif
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
