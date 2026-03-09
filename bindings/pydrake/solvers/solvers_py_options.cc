#include <memory>
#include <string>
#include <utility>

#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
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
  constexpr auto& doc = pydrake_doc_solvers.drake.solvers;

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
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
