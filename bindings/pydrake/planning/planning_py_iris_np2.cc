#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/planning/iris/iris_np2.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningIrisNp2(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  // IrisNp2Options
  const auto& cls_doc = doc.IrisNp2Options;
  py::class_<IrisNp2Options> iris_np2_options(m, "IrisNp2Options", cls_doc.doc);
  iris_np2_options.def(py::init<>())
      .def_readwrite("sampled_iris_options",
          &IrisNp2Options::sampled_iris_options,
          cls_doc.sampled_iris_options.doc)
      .def_readwrite("parameterization", &IrisNp2Options::parameterization,
          cls_doc.parameterization.doc)
      .def("__repr__", [](const IrisNp2Options& self) {
        return py::str(
            "IrisNp2Options("
            "sampled_iris_options={}, "
            ")")
            .format(self.sampled_iris_options);
      });

  // The `options` contains a `Parallelism`; we must release the GIL.
  m.def("IrisNp2", &IrisNp2, py::arg("checker"), py::arg("starting_ellipsoid"),
      py::arg("domain"), py::arg("options") = IrisNp2Options(),
      py::call_guard<py::gil_scoped_release>(), doc.IrisNp2.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
