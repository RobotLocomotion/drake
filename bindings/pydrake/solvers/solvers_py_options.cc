#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
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
    using NestedOptionsDict = decltype(SolverOptions{}.options);
    py::class_<SolverOptions> cls(m, "SolverOptions", doc.SolverOptions.doc);
    cls  // BR
        .def(py::init([](NestedOptionsDict& options) {
          auto result = std::make_unique<SolverOptions>();
          result->options = std::move(options);
          return result;
        }),
            py::kw_only(), py::arg("options") = NestedOptionsDict{})
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
        .def_readwrite("options", &SolverOptions::options)
        .def(py::self == py::self)
        .def(py::self != py::self);
    DefAttributesUsingSerialize(&cls);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls  // BR
        .def("GetOptions",
            WrapDeprecated(doc.SolverOptions.GetOptionsDouble.doc_deprecated,
                [](const SolverOptions& solver_options, SolverId solver_id) {
                  py::dict out;
                  py::object update = out.attr("update");
                  update(solver_options.GetOptionsDouble(solver_id));
                  update(solver_options.GetOptionsInt(solver_id));
                  update(solver_options.GetOptionsStr(solver_id));
                  return out;
                }),
            py::arg("solver_id"),
            doc.SolverOptions.GetOptionsDouble.doc_deprecated)
        .def("common_solver_options",
            WrapDeprecated(
                doc.SolverOptions.common_solver_options.doc_deprecated,
                &SolverOptions::common_solver_options),
            doc.SolverOptions.common_solver_options.doc_deprecated)
        .def("get_print_file_name",
            WrapDeprecated(doc.SolverOptions.get_print_file_name.doc_deprecated,
                &SolverOptions::get_print_file_name),
            doc.SolverOptions.get_print_file_name.doc_deprecated)
        .def("get_print_to_console",
            WrapDeprecated(
                doc.SolverOptions.get_print_to_console.doc_deprecated,
                &SolverOptions::get_print_to_console),
            doc.SolverOptions.get_print_to_console.doc_deprecated)
        .def("get_standalone_reproduction_file_name",
            WrapDeprecated(
                doc.SolverOptions.get_standalone_reproduction_file_name
                    .doc_deprecated,
                &SolverOptions::get_standalone_reproduction_file_name))
        .def("get_max_threads",
            WrapDeprecated(doc.SolverOptions.get_max_threads.doc_deprecated,
                &SolverOptions::get_max_threads),
            doc.SolverOptions.get_max_threads.doc_deprecated);
#pragma GCC diagnostic pop
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
