#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace pydrake {
namespace {

using systems::LeafSystem;

// TODO(eric.cousineau): Bind additional scalar types.
using T = double;

template <typename Class>
void BindIdentifier(py::module m, const std::string& name) {
  py::class_<Class>(m, name.c_str())
      .def(py::init<>())
      .def("get_value", &Class::get_value)
      .def("is_valid", &Class::is_valid)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(py::self < py::self);
}

PYBIND11_MODULE(geometry, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;

  py::module::import("pydrake.systems.framework");
  py::class_<SceneGraph<T>, LeafSystem<T>>(m, "SceneGraph")
      .def(py::init<>())
      .def("get_source_pose_port", &SceneGraph<T>::get_source_pose_port,
           py_reference_internal)
      .def("get_pose_bundle_output_port",
           &SceneGraph<T>::get_pose_bundle_output_port, py_reference_internal)
      .def("get_query_output_port", &SceneGraph<T>::get_query_output_port,
           py_reference_internal);

  BindIdentifier<SourceId>(m, "SourceId");
  BindIdentifier<FrameId>(m, "FrameId");
  BindIdentifier<GeometryId>(m, "GeometryId");

  m.def("ConnectDrakeVisualizer", &ConnectDrakeVisualizer,
        py::arg("builder"), py::arg("scene_graph"), py::arg("lcm") = nullptr);
  m.def("DispatchLoadMessage", &DispatchLoadMessage,
        py::arg("scene_graph"), py::arg("lcm"));

  // PenetrationAsPointPair
  {
    using Class = PenetrationAsPointPair<T>;
    py::class_<Class>(m, "PenetrationAsPointPair")
      .def(py::init<>())
      .def_readwrite("id_A", &Class::id_A)
      .def_readwrite("id_B", &Class::id_B)
      .def_readwrite("p_WCa", &Class::p_WCa)
      .def_readwrite("p_WCb", &Class::p_WCb)
      .def_readwrite("nhat_BA_W", &Class::nhat_BA_W)
      .def_readwrite("depth", &Class::depth);
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
