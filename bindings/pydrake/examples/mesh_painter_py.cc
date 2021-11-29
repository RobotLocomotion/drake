#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/mesh_painter/mask_image_camera.h"
#include "drake/examples/mesh_painter/mesh_painter_system.h"
#include "drake/examples/mesh_painter/render_engine_vtk_with_mask_images_factory.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(mesh_painter, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::mesh_painter;
  constexpr auto& doc = pydrake_doc.drake.examples.mesh_painter;

  m.doc() = "Bindings for the MeshPainter example.";

  py::module::import("pydrake.systems.framework");

  using T = double;

  {
    using Class = MeshPainterSystem;
    constexpr auto& cls_doc = doc.MeshPainterSystem;

    py::class_<MeshPainterSystem, LeafSystem<T>>(
        m, "MeshPainterSystem", doc.MeshPainterSystem.doc)
        .def(py::init<geometry::GeometryId, geometry::GeometryId,
                 const systems::sensors::ImageRgba8U&,
                 const geometry::SceneGraph<double>&, double, bool>(),
            py::arg("canvas_id"), py::arg("painter_id"), py::arg("image"),
            py::arg("scene_graph"), py::arg("period_sec"),
            py::arg("parallelize_raster") = false, cls_doc.ctor.doc_6args)
        .def(py::init<geometry::GeometryId, geometry::GeometryId, int, int,
                 const geometry::SceneGraph<double>&, double, bool>(),
            py::arg("canvas_id"), py::arg("painter_id"), py::arg("width"),
            py::arg("height"), py::arg("scene_graph"), py::arg("period_sec"),
            py::arg("parallelize_raster") = false, cls_doc.ctor.doc_7args)
        .def("geometry_query_input_port", &Class::geometry_query_input_port,
            py_rvp::reference_internal, cls_doc.geometry_query_input_port.doc)
        .def("texture_output_port", &Class::texture_output_port,
            py_rvp::reference_internal, cls_doc.texture_output_port.doc);
  }

  m.def("MakeRenderEngineVtkWithMaskImages", &MakeRenderEngineVtkWithMaskImages,
      py::arg("params"), doc.MakeRenderEngineVtkWithMaskImages.doc);

  {
    using Class = MaskImageCamera;
    constexpr auto& cls_doc = doc.MaskImageCamera;

    py::class_<MaskImageCamera, LeafSystem<T>>(
        m, "MaskImageCamera", doc.MaskImageCamera.doc)
        .def(py::init<geometry::FrameId, const drake::math::RigidTransformd&,
                 std::string, const geometry::GeometryId,
                 const geometry::render::ColorRenderCamera&,
                 geometry::render::DepthRenderCamera&>(),
            py::arg("parent_id"), py::arg("X_PB"),
            py::arg("render_engine_name"), py::arg("masked_id"),
            py::arg("color_properties"), py::arg("depth_properties"),
            cls_doc.ctor.doc_color_props)
        .def(py::init<geometry::FrameId, const drake::math::RigidTransformd&,
                 std::string, const geometry::GeometryId,
                 geometry::render::DepthRenderCamera&, bool>(),
            py::arg("parent_id"), py::arg("X_PB"),
            py::arg("render_engine_name"), py::arg("masked_id"),
            py::arg("properties"), py::arg("show_color_window") = false,
            cls_doc.ctor.doc_use_depth_props)
        .def("query_object_input_port", &Class::query_object_input_port,
            py_rvp::reference_internal, cls_doc.query_object_input_port.doc)
        .def("mask_texture_input_port", &Class::mask_texture_input_port,
            py_rvp::reference_internal, cls_doc.mask_texture_input_port.doc)
        .def("color_image_output_port", &Class::color_image_output_port,
            py_rvp::reference_internal, cls_doc.color_image_output_port.doc)
        .def("depth_image_32F_output_port", &Class::depth_image_32F_output_port,
            py_rvp::reference_internal, cls_doc.depth_image_32F_output_port.doc)
        .def("depth_image_16U_output_port", &Class::depth_image_16U_output_port,
            py_rvp::reference_internal, cls_doc.depth_image_16U_output_port.doc)
        .def("label_image_output_port", &Class::label_image_output_port,
            py_rvp::reference_internal, cls_doc.label_image_output_port.doc)
        .def("camera_body_pose_in_world_output_port",
            &Class::camera_body_pose_in_world_output_port,
            py_rvp::reference_internal,
            cls_doc.camera_body_pose_in_world_output_port.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
