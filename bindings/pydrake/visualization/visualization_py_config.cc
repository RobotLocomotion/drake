#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/visualization/visualization_py.h"
#include "drake/visualization/visualizer_config.h"
#include "drake/visualization/visualizer_config_functions.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineVisualizationConfig(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::visualization;
  constexpr auto& doc = pydrake_doc.drake.visualization;

  {
    using Class = VisualizerConfig;
    constexpr auto& cls_doc = doc.VisualizerConfig;
    py::class_<Class> cls(m, "VisualizerConfig", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  m  // BR
      .def("ApplyVisualizerConfig", &ApplyVisualizerConfig, py::arg("config"),
          py::arg("builder"), py::arg("lcm_buses") = nullptr,
          py::arg("plant") = nullptr, py::arg("scene_graph") = nullptr,
          py::arg("lcm") = nullptr, doc.ApplyVisualizerConfig.doc)
      .def("AddDefaultVisualizer", &AddDefaultVisualizer, py::arg("builder"),
          doc.AddDefaultVisualizer.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
