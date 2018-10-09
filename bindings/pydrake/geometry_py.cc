#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
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
  constexpr auto& doc = pydrake_doc.drake.geometry;

  py::module::import("pydrake.systems.framework");
  py::class_<SceneGraph<T>, LeafSystem<T>>(m, "SceneGraph", doc.SceneGraph.doc)
      .def(py::init<>(), doc.SceneGraph.ctor.doc_4)
      .def("get_source_pose_port", &SceneGraph<T>::get_source_pose_port,
           py_reference_internal, doc.SceneGraph.get_source_pose_port.doc)
      .def("get_pose_bundle_output_port",
           &SceneGraph<T>::get_pose_bundle_output_port, py_reference_internal,
           doc.SceneGraph.get_pose_bundle_output_port.doc)
      .def("get_query_output_port", &SceneGraph<T>::get_query_output_port,
           py_reference_internal, doc.SceneGraph.get_query_output_port.doc);

  // TODO(m-chaturvedi) Add Pybind11 documentation.
  BindIdentifier<SourceId>(m, "SourceId");
  BindIdentifier<FrameId>(m, "FrameId");
  BindIdentifier<GeometryId>(m, "GeometryId");

  py::module::import("pydrake.systems.lcm");
  m.def("ConnectDrakeVisualizer", &ConnectDrakeVisualizer,
        py::arg("builder"), py::arg("scene_graph"), py::arg("lcm") = nullptr,
        // Keep alive, ownership: `return` keeps `builder` alive.
        py::keep_alive<0, 1>(),
        // TODO(eric.cousineau): Figure out why this is necessary (#9398).
        py_reference, doc.ConnectDrakeVisualizer.doc);
  m.def("DispatchLoadMessage", &DispatchLoadMessage,
        py::arg("scene_graph"), py::arg("lcm"), doc.DispatchLoadMessage.doc);

  // PenetrationAsPointPair
  py::class_<PenetrationAsPointPair<T>>(m, "PenetrationAsPointPair")
    .def(py::init<>(), doc.PenetrationAsPointPair.ctor.doc_3)
    .def_readwrite("id_A", &PenetrationAsPointPair<T>::id_A,
      doc.PenetrationAsPointPair.id_A.doc)
    .def_readwrite("id_B", &PenetrationAsPointPair<T>::id_B,
      doc.PenetrationAsPointPair.id_B.doc)
    .def_readwrite("p_WCa", &PenetrationAsPointPair<T>::p_WCa,
      doc.PenetrationAsPointPair.p_WCa.doc)
    .def_readwrite("p_WCb", &PenetrationAsPointPair<T>::p_WCb,
      doc.PenetrationAsPointPair.p_WCb.doc)
    .def_readwrite("nhat_BA_W", &PenetrationAsPointPair<T>::nhat_BA_W,
      doc.PenetrationAsPointPair.nhat_BA_W.doc)
    .def_readwrite("depth", &PenetrationAsPointPair<T>::depth,
      doc.PenetrationAsPointPair.depth.doc);
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
