#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace pydrake {
namespace {

using systems::LeafSystem;

// TODO(eric.cousineau): Bind additional scalar types.
using T = double;

template <typename Class>
void BindIdentifier(py::module m, const std::string& name) {
  auto& cls_doc = pydrake_doc.drake.geometry.Identifier;

  py::class_<Class> cls(m, name.c_str());
  py::handle cls_handle = cls;
  cls  // BR
      .def(py::init([cls_handle]() {
        WarnDeprecated(
            py::str("The constructor for {} in Python is deprecated. "
                    "Use `get_new_id()` if necessary.")
                .format(cls_handle));
        return Class{};
      }),
          cls_doc.ctor.doc)
      .def("get_value", &Class::get_value, cls_doc.get_value.doc)
      .def("is_valid", &Class::is_valid, cls_doc.is_valid.doc)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(py::self < py::self)
      .def_static("get_new_id", &Class::get_new_id, cls_doc.get_new_id.doc);
}

PYBIND11_MODULE(geometry, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // TODO(m-chaturvedi) Add Pybind11 documentation to aliases (#9599).
  BindIdentifier<SourceId>(m, "SourceId");
  BindIdentifier<FrameId>(m, "FrameId");
  BindIdentifier<GeometryId>(m, "GeometryId");

  py::module::import("pydrake.systems.framework");
  py::class_<SceneGraphInspector<T>>(
      m, "SceneGraphInspector", doc.SceneGraphInspector.doc)
      .def("GetFrameId", &SceneGraphInspector<T>::GetFrameId,
          py::arg("geometry_id"), doc.SceneGraphInspector.GetFrameId.doc);

  py::class_<SceneGraph<T>, LeafSystem<T>>(m, "SceneGraph", doc.SceneGraph.doc)
      .def(py::init<>(), doc.SceneGraph.ctor.doc)
      .def("get_source_pose_port", &SceneGraph<T>::get_source_pose_port,
          py_reference_internal, doc.SceneGraph.get_source_pose_port.doc)
      .def("get_pose_bundle_output_port",
          &SceneGraph<T>::get_pose_bundle_output_port, py_reference_internal,
          doc.SceneGraph.get_pose_bundle_output_port.doc)
      .def("get_query_output_port", &SceneGraph<T>::get_query_output_port,
          py_reference_internal, doc.SceneGraph.get_query_output_port.doc)
      .def("RegisterSource",
          py::overload_cast<const std::string&>(  // BR
              &SceneGraph<T>::RegisterSource),
          py::arg("name") = "", doc.SceneGraph.RegisterSource.doc);

  py::class_<QueryObject<T>>(m, "QueryObject", doc.QueryObject.doc)
      .def("inspector", &QueryObject<T>::inspector, py_reference_internal,
          doc.QueryObject.inspector.doc)
      .def("ComputeSignedDistancePairwiseClosestPoints",
          &QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints,
          doc.QueryObject.ComputeSignedDistancePairwiseClosestPoints.doc)
      .def("ComputePointPairPenetration",
          &QueryObject<T>::ComputePointPairPenetration,
          doc.QueryObject.ComputePointPairPenetration.doc);
  pysystems::AddValueInstantiation<QueryObject<T>>(m);

  py::module::import("pydrake.systems.lcm");
  m.def("ConnectDrakeVisualizer",
      py::overload_cast<systems::DiagramBuilder<double>*,
          const SceneGraph<double>&, lcm::DrakeLcmInterface*>(
          &ConnectDrakeVisualizer),
      py::arg("builder"), py::arg("scene_graph"), py::arg("lcm") = nullptr,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // TODO(eric.cousineau): Figure out why this is necessary (#9398).
      py_reference, doc.ConnectDrakeVisualizer.doc_3args);
  m.def("ConnectDrakeVisualizer",
      py::overload_cast<systems::DiagramBuilder<double>*,
          const SceneGraph<double>&, const systems::OutputPort<double>&,
          lcm::DrakeLcmInterface*>(&ConnectDrakeVisualizer),
      py::arg("builder"), py::arg("scene_graph"),
      py::arg("pose_bundle_output_port"), py::arg("lcm") = nullptr,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // TODO(eric.cousineau): Figure out why this is necessary (#9398).
      py_reference, doc.ConnectDrakeVisualizer.doc_4args);
  m.def("DispatchLoadMessage", &DispatchLoadMessage, py::arg("scene_graph"),
      py::arg("lcm"), doc.DispatchLoadMessage.doc);

  // SignedDistancePair
  py::class_<SignedDistancePair<T>>(m, "SignedDistancePair")
      .def(py::init<>(), doc.SignedDistancePair.ctor.doc_6args)
      .def_readwrite(
          "id_A", &SignedDistancePair<T>::id_A, doc.SignedDistancePair.id_A.doc)
      .def_readwrite(
          "id_B", &SignedDistancePair<T>::id_B, doc.SignedDistancePair.id_B.doc)
      .def_readwrite("p_ACa", &SignedDistancePair<T>::p_ACa,
          doc.SignedDistancePair.p_ACa.doc)
      .def_readwrite("p_BCb", &SignedDistancePair<T>::p_BCb,
          doc.SignedDistancePair.p_BCb.doc)
      .def_readwrite("distance", &SignedDistancePair<T>::distance,
          doc.SignedDistancePair.distance.doc)
      .def_readwrite("nhat_BA_W", &SignedDistancePair<T>::nhat_BA_W,
          doc.SignedDistancePair.nhat_BA_W.doc);

  // PenetrationAsPointPair
  py::class_<PenetrationAsPointPair<T>>(m, "PenetrationAsPointPair")
      .def(py::init<>(), doc.PenetrationAsPointPair.ctor.doc)
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

  // Shape constructors
  {
    py::class_<Shape>(m, "Shape", doc.Shape.doc);
    py::class_<Sphere, Shape>(m, "Sphere", doc.Sphere.doc)
        .def(py::init<double>(), py::arg("radius"), doc.Sphere.ctor.doc);
    py::class_<Cylinder, Shape>(m, "Cylinder", doc.Cylinder.doc)
        .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
            doc.Cylinder.ctor.doc);
    py::class_<Box, Shape>(m, "Box", doc.Box.doc)
        .def(py::init<double, double, double>(), py::arg("width"),
            py::arg("depth"), py::arg("height"), doc.Box.ctor.doc);
    py::class_<HalfSpace, Shape>(m, "HalfSpace", doc.HalfSpace.doc)
        .def(py::init<>(), doc.HalfSpace.ctor.doc);
    py::class_<Mesh, Shape>(m, "Mesh", doc.Mesh.doc)
        .def(py::init<std::string, double>(), py::arg("absolute_filename"),
            py::arg("scale") = 1.0, doc.Mesh.ctor.doc);
    py::class_<Convex, Shape>(m, "Convex", doc.Convex.doc)
        .def(py::init<std::string, double>(), py::arg("absolute_filename"),
            py::arg("scale") = 1.0, doc.Convex.ctor.doc);
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
