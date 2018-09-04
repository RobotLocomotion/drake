#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_viewer_link_data.hpp"

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
      .def("get_query_output_port", &SceneGraph<T>::get_query_output_port,
            py_reference_internal)
      .def("get_pose_bundle_output_port",
           &SceneGraph<T>::get_pose_bundle_output_port, py_reference_internal);

  BindIdentifier<SourceId>(m, "SourceId");
  BindIdentifier<FrameId>(m, "FrameId");
  BindIdentifier<GeometryId>(m, "GeometryId");

  // Yuck!  Until we support meshcat directly from c++, or finish the API for
  // SceneGraphInspector, we have to expose the innards (and the lcm types)
  // in order to populate the load message.
  py::class_<internal::GeometryVisualizationImpl>(m,
                                                  "GeometryVisualizationImpl")
      .def_static("BuildLoadMessage",
                  &internal::GeometryVisualizationImpl::BuildLoadMessage,
                  py::arg("scene_graph"));

  py::class_<lcmt_viewer_load_robot>(m, "lcmt_viewer_load_robot")
      .def_readwrite("num_links", &lcmt_viewer_load_robot::num_links)
      .def_readwrite("link", &lcmt_viewer_load_robot::link);

  py::class_<lcmt_viewer_link_data>(m, "lcmt_viewer_link_data")
      .def_readwrite("name", &lcmt_viewer_link_data::name)
      .def_readwrite("robot_num", &lcmt_viewer_link_data::robot_num)
      .def_readwrite("num_geom", &lcmt_viewer_link_data::num_geom)
      .def_readwrite("geom", &lcmt_viewer_link_data::geom);

  py::class_<lcmt_viewer_geometry_data>(m, "lcmt_viewer_geometry_data")
      .def_readwrite("type", &lcmt_viewer_geometry_data::type)
      .def("position",
           [](const lcmt_viewer_geometry_data* self) {
             return Eigen::Vector3d(self->position[0], self->position[1],
                                    self->position[2]);
           })
      .def("quaternion",
           [](const lcmt_viewer_geometry_data* self) {
             return Eigen::Vector4d(self->quaternion[0], self->quaternion[1],
                                    self->quaternion[2], self->quaternion[3]);
           })
      .def("color",
           [](const lcmt_viewer_geometry_data* self) {
             return Eigen::Vector4d(self->color[0], self->color[1],
                                    self->color[2], self->color[3]);
           })
      .def_readwrite("string_data", &lcmt_viewer_geometry_data::string_data)
      .def_readwrite("num_float_data",
                     &lcmt_viewer_geometry_data::num_float_data)
      .def_readwrite("float_data", &lcmt_viewer_geometry_data::float_data);
  // end Yuck!

  m.def("ConnectVisualization", &ConnectVisualization,
        py::arg("scene_graph"), py::arg("builder"), py::arg("lcm"));
  m.def("DispatchLoadMessage", &DispatchLoadMessage,
        py::arg("scene_graph"), py::arg("lcm"));
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
