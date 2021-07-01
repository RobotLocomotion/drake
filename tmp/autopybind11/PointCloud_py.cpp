#include "drake/perception/point_cloud.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::perception;

namespace py = pybind11;

void apb11_pydrake_PointCloud_py_register(py::module &m) {
  py::class_<PointCloud> PyPointCloud(m, "PointCloud");

  PyPointCloud
      .def(py::init<int, pc_flags::Fields, bool>(),
           py::arg("new_size") = int(0),
           py::arg("fields") = pc_flags::Fields(pc_flags::kXYZs),
           py::arg("skip_initialize") = bool(false))
      .def(py::init<PointCloud const &>(), py::arg("other"))
      .def(py::init<PointCloud const &, pc_flags::Fields>(), py::arg("other"),
           py::arg("copy_fields"))
      .def("Expand",
           static_cast<void (PointCloud::*)(int, bool)>(&PointCloud::Expand),
           py::arg("add_size"), py::arg("skip_initialization") = bool(false))
      .def("HasExactFields",
           static_cast<bool (PointCloud::*)(pc_flags::Fields) const>(
               &PointCloud::HasExactFields),
           py::arg("fields_in"))
      .def("HasFields",
           static_cast<bool (PointCloud::*)(pc_flags::Fields) const>(
               &PointCloud::HasFields),
           py::arg("fields_in"))
      .def_static(
          "IsDefaultValue",
          static_cast<bool (*)(PointCloud::T)>(&PointCloud::IsDefaultValue),
          py::arg("value"))
      .def_static(
          "IsInvalidValue",
          static_cast<bool (*)(PointCloud::T)>(&PointCloud::IsInvalidValue),
          py::arg("value"))
      .def("RequireExactFields",
           static_cast<void (PointCloud::*)(pc_flags::Fields) const>(
               &PointCloud::RequireExactFields),
           py::arg("field_set"))
      .def("RequireFields",
           static_cast<void (PointCloud::*)(pc_flags::Fields) const>(
               &PointCloud::RequireFields),
           py::arg("fields_in"))
      .def(
          "SetFrom",
          static_cast<void (PointCloud::*)(PointCloud const &, pc_flags::Fields,
                                           bool)>(&PointCloud::SetFrom),
          py::arg("other"),
          py::arg("fields_in") = pc_flags::Fields(pc_flags::kInherit),
          py::arg("allow_resize") = bool(true))
      .def("descriptor",
           static_cast<::Eigen::Matrix<float, -1, 1, 0, -1, 1> (PointCloud::*)(
               int) const>(&PointCloud::descriptor),
           py::arg("i"))
      .def("descriptor_type",
           static_cast<pc_flags::DescriptorType const &(PointCloud::*)() const>(
               &PointCloud::descriptor_type))
      .def("descriptors",
           static_cast<
               ::Eigen::Ref<const Eigen::Matrix<float, -1, -1, 0, -1, -1>, 0,
                            Eigen::OuterStride<-1>> (PointCloud::*)() const>(
               &PointCloud::descriptors))
      .def("fields", static_cast<pc_flags::Fields (PointCloud::*)() const>(
                         &PointCloud::fields))
      .def("has_descriptors", static_cast<bool (PointCloud::*)() const>(
                                  &PointCloud::has_descriptors))
      .def("has_descriptors",
           static_cast<bool (PointCloud::*)(pc_flags::DescriptorType const &)
                           const>(&PointCloud::has_descriptors),
           py::arg("descriptor_type"))
      .def("has_normals",
           static_cast<bool (PointCloud::*)() const>(&PointCloud::has_normals))
      .def("has_rgbs",
           static_cast<bool (PointCloud::*)() const>(&PointCloud::has_rgbs))
      .def("has_xyzs",
           static_cast<bool (PointCloud::*)() const>(&PointCloud::has_xyzs))
      .def(
          "mutable_descriptor",
          static_cast<::Eigen::Ref<Eigen::Matrix<float, -1, 1, 0, -1, 1>, 0,
                                   Eigen::InnerStride<1>> (PointCloud::*)(int)>(
              &PointCloud::mutable_descriptor),
          py::arg("i"))
      .def("mutable_descriptors",
           static_cast<::Eigen::Ref<Eigen::Matrix<float, -1, -1, 0, -1, -1>, 0,
                                    Eigen::OuterStride<-1>> (PointCloud::*)()>(
               &PointCloud::mutable_descriptors))
      .def(
          "mutable_normal",
          static_cast<::Eigen::Ref<Eigen::Matrix<float, 3, 1, 0, 3, 1>, 0,
                                   Eigen::InnerStride<1>> (PointCloud::*)(int)>(
              &PointCloud::mutable_normal),
          py::arg("i"))
      .def("mutable_normals",
           static_cast<::Eigen::Ref<Eigen::Matrix<float, 3, -1, 0, 3, -1>, 0,
                                    Eigen::OuterStride<-1>> (PointCloud::*)()>(
               &PointCloud::mutable_normals))
      .def("mutable_rgb",
           static_cast<::Eigen::Ref<Eigen::Matrix<unsigned char, 3, 1, 0, 3, 1>,
                                    0, Eigen::InnerStride<1>> (PointCloud::*)(
               int)>(&PointCloud::mutable_rgb),
           py::arg("i"))
      .def("mutable_rgbs",
           static_cast<
               ::Eigen::Ref<Eigen::Matrix<unsigned char, 3, -1, 0, 3, -1>, 0,
                            Eigen::OuterStride<-1>> (PointCloud::*)()>(
               &PointCloud::mutable_rgbs))
      .def(
          "mutable_xyz",
          static_cast<::Eigen::Ref<Eigen::Matrix<float, 3, 1, 0, 3, 1>, 0,
                                   Eigen::InnerStride<1>> (PointCloud::*)(int)>(
              &PointCloud::mutable_xyz),
          py::arg("i"))
      .def("mutable_xyzs",
           static_cast<::Eigen::Ref<Eigen::Matrix<float, 3, -1, 0, 3, -1>, 0,
                                    Eigen::OuterStride<-1>> (PointCloud::*)()>(
               &PointCloud::mutable_xyzs))
      .def("normal",
           static_cast<::Eigen::Matrix<float, 3, 1, 0, 3, 1> (PointCloud::*)(
               int) const>(&PointCloud::normal),
           py::arg("i"))
      .def("normals",
           static_cast<::Eigen::Ref<const Eigen::Matrix<float, 3, -1, 0, 3, -1>,
                                    0, Eigen::OuterStride<-1>> (PointCloud::*)()
                           const>(&PointCloud::normals))
      .def("resize",
           static_cast<void (PointCloud::*)(int, bool)>(&PointCloud::resize),
           py::arg("new_size"), py::arg("skip_initialize") = bool(false))
      .def("rgb",
           static_cast<::Eigen::Matrix<unsigned char, 3, 1, 0, 3, 1> (
               PointCloud::*)(int) const>(&PointCloud::rgb),
           py::arg("i"))
      .def("rgbs",
           static_cast<
               ::Eigen::Ref<const Eigen::Matrix<unsigned char, 3, -1, 0, 3, -1>,
                            0, Eigen::OuterStride<-1>> (PointCloud::*)() const>(
               &PointCloud::rgbs))
      .def("size", static_cast<int (PointCloud::*)() const>(&PointCloud::size))
      .def("xyz",
           static_cast<::Eigen::Matrix<float, 3, 1, 0, 3, 1> (PointCloud::*)(
               int) const>(&PointCloud::xyz),
           py::arg("i"))
      .def("xyzs",
           static_cast<::Eigen::Ref<const Eigen::Matrix<float, 3, -1, 0, 3, -1>,
                                    0, Eigen::OuterStride<-1>> (PointCloud::*)()
                           const>(&PointCloud::xyzs))
      .def_readonly_static("kDefaultColor", &PointCloud::kDefaultColor)
      .def_readonly_static("kDefaultValue", &PointCloud::kDefaultValue)

      ;
}
