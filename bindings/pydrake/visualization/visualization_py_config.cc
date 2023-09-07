#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/visualization/visualization_py.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineVisualizationConfig(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::visualization;
  constexpr auto& doc = pydrake_doc.drake.visualization;

  {
    using Class = VisualizationConfig;
    constexpr auto& cls_doc = doc.VisualizationConfig;
    py::class_<Class> cls(m, "VisualizationConfig", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  m  // BR
      .def("ApplyVisualizationConfig", &ApplyVisualizationConfig,
          py::arg("config"), py::arg("builder"), py::arg("lcm_buses") = nullptr,
          py::arg("plant") = nullptr, py::arg("scene_graph") = nullptr,
          py::arg("meshcat") = nullptr, py::arg("lcm") = nullptr,
          doc.ApplyVisualizationConfig.doc)
      .def("AddDefaultVisualization", &AddDefaultVisualization,
          py::arg("builder"), py::arg("meshcat") = nullptr,
          doc.AddDefaultVisualization.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
