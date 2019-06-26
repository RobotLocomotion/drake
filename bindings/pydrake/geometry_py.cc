#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace pydrake {
namespace {
using systems::LeafSystem;

template <typename Class>
void BindIdentifier(py::module m, const std::string& name, const char* id_doc) {
  auto& cls_doc = pydrake_doc.drake.geometry.Identifier;

  py::class_<Class> cls(m, name.c_str(), id_doc);
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

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  constexpr auto& doc = pydrake_doc.drake.geometry;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  py::module::import("pydrake.systems.framework");

  //  SceneGraphInspector
  {
    using Class = SceneGraphInspector<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SceneGraphInspector", param, doc.SceneGraphInspector.doc);
    cls  // BR
        .def("GetFrameId", &SceneGraphInspector<T>::GetFrameId,
            py::arg("geometry_id"), doc.SceneGraphInspector.GetFrameId.doc);
  }

  //  SceneGraph
  {
    auto cls = DefineTemplateClassWithDefault<SceneGraph<T>, LeafSystem<T>>(
        m, "SceneGraph", param, doc.SceneGraph.doc);
    cls  // BR
        .def(py::init<>(), doc.SceneGraph.ctor.doc)
        .def("get_source_pose_port", &SceneGraph<T>::get_source_pose_port,
            py_reference_internal, doc.SceneGraph.get_source_pose_port.doc)
        .def("get_pose_bundle_output_port",
            [](SceneGraph<T>* self) -> const systems::OutputPort<T>& {
              return self->get_pose_bundle_output_port();
            },
            py_reference_internal,
            doc.SceneGraph.get_pose_bundle_output_port.doc)
        .def("get_query_output_port", &SceneGraph<T>::get_query_output_port,
            py_reference_internal, doc.SceneGraph.get_query_output_port.doc)
        .def("RegisterSource",
            py::overload_cast<const std::string&>(  // BR
                &SceneGraph<T>::RegisterSource),
            py::arg("name") = "", doc.SceneGraph.RegisterSource.doc);
  }

  //  FramePoseVector
  {
    using Class = FramePoseVector<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "FramePoseVector", param, doc.FrameKinematicsVector.doc);
    cls  // BR
        .def(py::init<>(), doc.FrameKinematicsVector.ctor.doc_0args)
        .def(py::init([](SourceId source_id, const std::vector<FrameId>& ids) {
          WarnDeprecated("See API docs for deprecation notice.");
          return std::make_unique<FramePoseVector<T>>(source_id, ids);
        }),
            py::arg("source_id"), py::arg("ids"),
            doc.FrameKinematicsVector.ctor.doc_deprecated_2args)
        .def("clear", &FramePoseVector<T>::clear,
            doc.FrameKinematicsVector.clear.doc)
        .def("set_value", &FramePoseVector<T>::set_value, py::arg("id"),
            py::arg("value"), doc.FrameKinematicsVector.set_value.doc)
        .def("size", &FramePoseVector<T>::size,
            doc.FrameKinematicsVector.size.doc)
        // This intentionally copies the value to avoid segfaults from accessing
        // the result after clear() is called. (see #11583)
        .def("value", &FramePoseVector<T>::value, py::arg("id"),
            doc.FrameKinematicsVector.value.doc)
        .def("has_id", &FramePoseVector<T>::has_id, py::arg("id"),
            doc.FrameKinematicsVector.has_id.doc)
        .def("frame_ids", &FramePoseVector<T>::frame_ids,
            doc.FrameKinematicsVector.frame_ids.doc);
    AddValueInstantiation<FramePoseVector<T>>(m);
  }

  //  QueryObject
  {
    using Class = QueryObject<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "QueryObject", param, doc.QueryObject.doc);
    cls  // BR
        .def("inspector", &QueryObject<T>::inspector, py_reference_internal,
            doc.QueryObject.inspector.doc)
        .def("ComputeSignedDistancePairwiseClosestPoints",
            &QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints,
            py::arg("max_distance") = std::numeric_limits<double>::infinity(),
            doc.QueryObject.ComputeSignedDistancePairwiseClosestPoints.doc)
        .def("ComputePointPairPenetration",
            &QueryObject<T>::ComputePointPairPenetration,
            doc.QueryObject.ComputePointPairPenetration.doc)
        .def("ComputeSignedDistanceToPoint",
            &QueryObject<T>::ComputeSignedDistanceToPoint, py::arg("p_WQ"),
            py::arg("threshold") = std::numeric_limits<double>::infinity(),
            doc.QueryObject.ComputeSignedDistanceToPoint.doc);
    AddValueInstantiation<QueryObject<T>>(m);
  }

  // SignedDistancePair
  {
    using Class = SignedDistancePair<T>;
    auto cls =
        DefineTemplateClassWithDefault<Class>(m, "SignedDistancePair", param);
    cls  // BR
        .def(py::init<>(), doc.SignedDistancePair.ctor.doc_7args)
        .def_readwrite("id_A", &SignedDistancePair<T>::id_A,
            doc.SignedDistancePair.id_A.doc)
        .def_readwrite("id_B", &SignedDistancePair<T>::id_B,
            doc.SignedDistancePair.id_B.doc)
        .def_readwrite("p_ACa", &SignedDistancePair<T>::p_ACa,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistancePair.p_ACa.doc)
        .def_readwrite("p_BCb", &SignedDistancePair<T>::p_BCb,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistancePair.p_BCb.doc)
        .def_readwrite("distance", &SignedDistancePair<T>::distance,
            doc.SignedDistancePair.distance.doc)
        .def_readwrite("nhat_BA_W", &SignedDistancePair<T>::nhat_BA_W,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistancePair.nhat_BA_W.doc)
        .def_readwrite("is_nhat_BA_W_unique",
            &SignedDistancePair<T>::is_nhat_BA_W_unique,
            doc.SignedDistancePair.is_nhat_BA_W_unique.doc);
  }

  // SignedDistanceToPoint
  {
    using Class = SignedDistanceToPoint<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SignedDistanceToPoint", param);
    cls  // BR
        .def(py::init<>(), doc.SignedDistanceToPoint.ctor.doc)
        .def_readwrite("id_G", &SignedDistanceToPoint<T>::id_G,
            doc.SignedDistanceToPoint.id_G.doc)
        .def_readwrite("p_GN", &SignedDistanceToPoint<T>::p_GN,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistanceToPoint.p_GN.doc)
        .def_readwrite("distance", &SignedDistanceToPoint<T>::distance,
            doc.SignedDistanceToPoint.distance.doc)
        .def_readwrite("grad_W", &SignedDistanceToPoint<T>::grad_W,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistanceToPoint.grad_W.doc);
  }

  // PenetrationAsPointPair
  {
    using Class = PenetrationAsPointPair<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "PenetrationAsPointPair", param);
    cls  // BR
        .def(py::init<>(), doc.PenetrationAsPointPair.ctor.doc)
        .def_readwrite("id_A", &PenetrationAsPointPair<T>::id_A,
            doc.PenetrationAsPointPair.id_A.doc)
        .def_readwrite("id_B", &PenetrationAsPointPair<T>::id_B,
            doc.PenetrationAsPointPair.id_B.doc)
        .def_readwrite("p_WCa", &PenetrationAsPointPair<T>::p_WCa,
            py::return_value_policy::copy, doc.PenetrationAsPointPair.p_WCa.doc)
        .def_readwrite("p_WCb", &PenetrationAsPointPair<T>::p_WCb,
            py::return_value_policy::copy, doc.PenetrationAsPointPair.p_WCb.doc)
        .def_readwrite("nhat_BA_W", &PenetrationAsPointPair<T>::nhat_BA_W,
            doc.PenetrationAsPointPair.nhat_BA_W.doc)
        .def_readwrite("depth", &PenetrationAsPointPair<T>::depth,
            doc.PenetrationAsPointPair.depth.doc);
  }
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;

  constexpr auto& doc = pydrake_doc.drake.geometry;
  BindIdentifier<SourceId>(m, "SourceId", doc.SourceId.doc);
  BindIdentifier<FrameId>(m, "FrameId", doc.FrameId.doc);
  BindIdentifier<GeometryId>(m, "GeometryId", doc.GeometryId.doc);

  m.def("ConnectDrakeVisualizer",
      py::overload_cast<systems::DiagramBuilder<double>*,
          const SceneGraph<double>&, lcm::DrakeLcmInterface*>(
          &ConnectDrakeVisualizer),
      py::arg("builder"), py::arg("scene_graph"), py::arg("lcm") = nullptr,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // See #11531 for why `py_reference` is needed.
      py_reference, doc.ConnectDrakeVisualizer.doc_3args);
  m.def("ConnectDrakeVisualizer",
      py::overload_cast<systems::DiagramBuilder<double>*,
          const SceneGraph<double>&, const systems::OutputPort<double>&,
          lcm::DrakeLcmInterface*>(&ConnectDrakeVisualizer),
      py::arg("builder"), py::arg("scene_graph"),
      py::arg("pose_bundle_output_port"), py::arg("lcm") = nullptr,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // See #11531 for why `py_reference` is needed.
      py_reference, doc.ConnectDrakeVisualizer.doc_4args);
  m.def("DispatchLoadMessage", &DispatchLoadMessage, py::arg("scene_graph"),
      py::arg("lcm"), doc.DispatchLoadMessage.doc);

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

PYBIND11_MODULE(geometry, m) {
  py::module::import("pydrake.systems.lcm");
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
