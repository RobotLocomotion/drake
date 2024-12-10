#include "drake/bindings/pydrake/common/deprecation_pybind.h"
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

  {
    py::enum_<CommonSolverOption>(
        m, "CommonSolverOption", doc.CommonSolverOption.doc)
        .value("kPrintFileName", CommonSolverOption::kPrintFileName,
            doc.CommonSolverOption.kPrintFileName.doc)
        .value("kPrintToConsole", CommonSolverOption::kPrintToConsole,
            doc.CommonSolverOption.kPrintToConsole.doc)
        .value("kStandaloneReproductionFileName",
            CommonSolverOption::kStandaloneReproductionFileName,
            doc.CommonSolverOption.kStandaloneReproductionFileName.doc)
        .value("kMaxThreads", CommonSolverOption::kMaxThreads,
            doc.CommonSolverOption.kMaxThreads.doc);
  }

  {
    // TODO(jwnimmer-tri) Bind the accessors for SolverOptions.
    py::class_<SolverOptions> cls(m, "SolverOptions", doc.SolverOptions.doc);
    cls  // BR
        .def(py::init<>(), doc.SolverOptions.ctor.doc)
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
        .def("common_solver_options", &SolverOptions::common_solver_options,
            doc.SolverOptions.common_solver_options.doc)
        .def("get_print_file_name", &SolverOptions::get_print_file_name,
            doc.SolverOptions.get_print_file_name.doc)
        .def("get_print_to_console", &SolverOptions::get_print_to_console,
            doc.SolverOptions.get_print_to_console.doc)
        .def("get_standalone_reproduction_file_name",
            &SolverOptions::get_standalone_reproduction_file_name,
            doc.SolverOptions.get_standalone_reproduction_file_name.doc)
        .def("get_max_threads", &SolverOptions::get_max_threads,
            doc.SolverOptions.get_max_threads.doc)
        .def("__repr__", [](const SolverOptions&) -> std::string {
          // This is a minimal implementation that serves to avoid displaying
          // memory addresses in pydrake docs and help strings. In the future,
          // we should enhance this to provide more details.
          return "<SolverOptions>";
        });
    DefCopyAndDeepCopy(&cls);

    constexpr char kSetOptionKwargNameDeprecation[] =
        "The kwarg names for SolverOptions.SetOption have changed; the new "
        "names are `key` and `value` not `solver_option` and `option_value`; "
        "the old names are deprecated and will be removed from Drake on or "
        "after 2025-05-01.";
    // Deprecated 2025-05.
    cls  // BR
        .def("SetOption",
            WrapDeprecated(kSetOptionKwargNameDeprecation,
                [](SolverOptions& self, const SolverId& solver_id,
                    const std::string& solver_option, double option_value) {
                  self.SetOption(solver_id, solver_option, option_value);
                }),
            py::arg("solver_id"), py::arg("solver_option"),
            py::arg("option_value"), kSetOptionKwargNameDeprecation)
        .def("SetOption",
            WrapDeprecated(kSetOptionKwargNameDeprecation,
                [](SolverOptions& self, const SolverId& solver_id,
                    const std::string& solver_option, int option_value) {
                  self.SetOption(solver_id, solver_option, option_value);
                }),
            py::arg("solver_id"), py::arg("solver_option"),
            py::arg("option_value"), kSetOptionKwargNameDeprecation)
        .def("SetOption",
            WrapDeprecated(kSetOptionKwargNameDeprecation,
                [](SolverOptions& self, const SolverId& solver_id,
                    const std::string& solver_option,
                    const std::string& option_value) {
                  self.SetOption(solver_id, solver_option, option_value);
                }),
            py::arg("solver_id"), py::arg("solver_option"),
            py::arg("option_value"), kSetOptionKwargNameDeprecation);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
