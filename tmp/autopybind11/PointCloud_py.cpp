#include "drake/perception/point_cloud.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_PointCloud_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::perception;

  py::class_<PointCloud> PyPointCloud(
      m, "PointCloud",
      R"""(/// Implements a point cloud (with contiguous storage), whose main goal is to 
/// offer a convenient, synchronized interface to commonly used fields and 
/// data types applicable for basic 3D perception. 
/// 
/// This is a mix between the philosophy of PCL (templated interface to 
/// provide a compile-time open set, run-time closed set) and VTK (non-templated 
/// interface to provide a very free form run-time open set). 
/// You may construct one PointCloud which will contain different sets of 
/// data, but you cannot change the contained data types after construction. 
/// However, you can mutate the data contained within the structure and resize 
/// the cloud. 
/// 
/// Definitions: 
/// 
/// - point - An entry in a point cloud (not exclusively an XYZ point). 
/// - feature - Abstract representation of local properties (geometric and 
///   non-geometric) 
/// - descriptor - Concrete representation of a feature. 
/// - field - A feature or descriptor described by the point cloud. 
/// 
/// This point cloud class provides the following fields: 
/// 
/// - xyz - Cartesian XYZ coordinates (float[3]). 
/// - descriptor - A descriptor that is run-time defined (float[X]). 
/// 
/// @note "contiguous" here means contiguous in memory. This was chosen to 
/// avoid ambiguity between PCL and Eigen, where in PCL "dense" implies that 
/// the point cloud corresponds to a cloud with invalid values, and in Eigen 
/// "dense" implies contiguous storage. 
/// 
/// @note The accessors / mutators for the point fields of this class returns 
/// references to the original Eigen matrices. This implies that they are 
/// invalidated whenever memory is reallocated for the values. Given this, 
/// minimize the lifetime of these references to be as short as possible. 
/// Additionally, algorithms wanting fast access to values should avoid the 
/// single point accessors / mutatotrs (e.g. `xyz(i)`, `mutable_descriptor(i)`) 
/// to avoid overhead when accessing a single element (either copying or 
/// creating a reference). 
/// 
/// @note The definitions presented here for "feature" and "descriptor" are 
/// loosely based on their definitions within PCL and Radu Rusu's dissertation: 
///   Rusu, Radu Bogdan. "Semantic 3d object maps for everyday manipulation in 
///   human living environments." KI-Künstliche Intelligenz 24.4 (2010): 
///   345-348. 
/// This differs from other definitions, such as having "feature" 
/// describe geometric quantities and "descriptor" describe non-geometric 
/// quantities which is presented in the following survey paper: 
///   Pomerleau, François, Francis Colas, and Roland Siegwart. "A review of 
///   point cloud registration algorithms for mobile robotics." Foundations and 
///   Trends® in Robotics 4.1 (2015): 1-104.)""");

  PyPointCloud
      .def(py::init<int, pc_flags::Fields, bool>(),
           py::arg("new_size") = int(0),
           py::arg("fields") =
               pc_flags::Fields(drake::perception::pc_flags::kXYZs),
           py::arg("skip_initialize") = bool(false))
      .def(py::init<PointCloud const &>(), py::arg("other"))
      .def(py::init<PointCloud const &, pc_flags::Fields>(), py::arg("other"),
           py::arg("copy_fields"))
      .def("Expand",
           static_cast<void (PointCloud::*)(int, bool)>(&PointCloud::Expand),
           py::arg("add_size"), py::arg("skip_initialization") = bool(false),
           R"""(/// Adds `add_size` default-initialized points. 
/// @param add_size 
///    Number of points to add. 
/// @param skip_initialization 
///    Do not require that the new values be initialized.)""")
      .def("HasExactFields",
           static_cast<bool (PointCloud::*)(pc_flags::Fields) const>(
               &PointCloud::HasExactFields),
           py::arg("fields_in"),
           R"""(/// Returns if a point cloud has exactly a given set of fields. 
/// @see HasFields for preconditions.)""")
      .def("HasFields",
           static_cast<bool (PointCloud::*)(pc_flags::Fields) const>(
               &PointCloud::HasFields),
           py::arg("fields_in"),
           R"""(/// Returns if a point cloud has a given set of fields.)""")
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
           py::arg("field_set"),
           R"""(/// Requires the exact given set of fields. 
/// @see HasFields for preconditions. 
/// @throws std::runtime_error if this point cloud does not have exactly 
/// these fields.)""")
      .def("RequireFields",
           static_cast<void (PointCloud::*)(pc_flags::Fields) const>(
               &PointCloud::RequireFields),
           py::arg("fields_in"),
           R"""(/// Requires a given set of fields. 
/// @see HasFields for preconditions. 
/// @throws std::runtime_error if this point cloud does not have these 
/// fields.)""")
      .def(
          "SetFrom",
          static_cast<void (PointCloud::*)(PointCloud const &, pc_flags::Fields,
                                           bool)>(&PointCloud::SetFrom),
          py::arg("other"),
          py::arg("fields_in") =
              pc_flags::Fields(drake::perception::pc_flags::kInherit),
          py::arg("allow_resize") = bool(true),
          R"""(/// Copies all points from another point cloud. 
/// @param other 
///    Other point cloud. 
/// @param fields_in 
///    Fields to copy. If this is `kInherit`, then both clouds must have the 
///    exact same fields. Otherwise, both clouds must support the fields 
///    indicated this parameter. 
/// @param allow_resize 
///    Permit resizing to the other cloud's size.)""")
      .def("descriptor",
           static_cast<::Eigen::Matrix<float, -1, 1, 0, -1, 1> (PointCloud::*)(
               int) const>(&PointCloud::descriptor),
           py::arg("i"),
           R"""(/// Returns access to a descriptor value. 
/// @pre `has_descriptors()` must be true.)""")
      .def("descriptor_type",
           static_cast<pc_flags::DescriptorType const &(PointCloud::*)() const>(
               &PointCloud::descriptor_type),
           R"""(/// Returns the descriptor type.)""")
      .def("descriptors",
           static_cast<
               ::Eigen::Ref<const Eigen::Matrix<float, -1, -1, 0, -1, -1>, 0,
                            Eigen::OuterStride<-1>> (PointCloud::*)() const>(
               &PointCloud::descriptors),
           R"""(/// Returns access to descriptor values. 
/// @pre `has_descriptors()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def("fields",
           static_cast<pc_flags::Fields (PointCloud::*)() const>(
               &PointCloud::fields),
           R"""(/// Returns the fields provided by this point cloud.)""")
      .def("has_descriptors",
           static_cast<bool (PointCloud::*)() const>(
               &PointCloud::has_descriptors),
           R"""(/// Returns if this point cloud provides descriptor values.)""")
      .def(
          "has_descriptors",
          static_cast<bool (PointCloud::*)(pc_flags::DescriptorType const &)
                          const>(&PointCloud::has_descriptors),
          py::arg("descriptor_type"),
          R"""(/// Returns if the point cloud provides a specific descriptor.)""")
      .def("has_normals",
           static_cast<bool (PointCloud::*)() const>(&PointCloud::has_normals),
           R"""(/// Returns if this cloud provides normals.)""")
      .def("has_rgbs",
           static_cast<bool (PointCloud::*)() const>(&PointCloud::has_rgbs),
           R"""(/// Returns if this cloud provides RGB colors.)""")
      .def("has_xyzs",
           static_cast<bool (PointCloud::*)() const>(&PointCloud::has_xyzs),
           R"""(/// Returns if this cloud provides XYZ values.)""")
      .def(
          "mutable_descriptor",
          static_cast<::Eigen::Ref<Eigen::Matrix<float, -1, 1, 0, -1, 1>, 0,
                                   Eigen::InnerStride<1>> (PointCloud::*)(int)>(
              &PointCloud::mutable_descriptor),
          py::arg("i"),
          R"""(/// Returns mutable access to a descriptor value. 
/// @pre `has_descriptors()` must be true.)""",
          py::return_value_policy::reference_internal)
      .def("mutable_descriptors",
           static_cast<::Eigen::Ref<Eigen::Matrix<float, -1, -1, 0, -1, -1>, 0,
                                    Eigen::OuterStride<-1>> (PointCloud::*)()>(
               &PointCloud::mutable_descriptors),
           R"""(/// Returns mutable access to descriptor values. 
/// @pre `has_descriptors()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def(
          "mutable_normal",
          static_cast<::Eigen::Ref<Eigen::Matrix<float, 3, 1, 0, 3, 1>, 0,
                                   Eigen::InnerStride<1>> (PointCloud::*)(int)>(
              &PointCloud::mutable_normal),
          py::arg("i"),
          R"""(/// Returns mutable access to a normal. 
/// @pre `has_normals()` must be true.)""",
          py::return_value_policy::reference_internal)
      .def("mutable_normals",
           static_cast<::Eigen::Ref<Eigen::Matrix<float, 3, -1, 0, 3, -1>, 0,
                                    Eigen::OuterStride<-1>> (PointCloud::*)()>(
               &PointCloud::mutable_normals),
           R"""(/// Returns mutable access to normals. 
/// @pre `has_normals()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def("mutable_rgb",
           static_cast<::Eigen::Ref<Eigen::Matrix<unsigned char, 3, 1, 0, 3, 1>,
                                    0, Eigen::InnerStride<1>> (PointCloud::*)(
               int)>(&PointCloud::mutable_rgb),
           py::arg("i"),
           R"""(/// Returns mutable access to an RGB color. 
/// @pre `has_rgbs()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def("mutable_rgbs",
           static_cast<
               ::Eigen::Ref<Eigen::Matrix<unsigned char, 3, -1, 0, 3, -1>, 0,
                            Eigen::OuterStride<-1>> (PointCloud::*)()>(
               &PointCloud::mutable_rgbs),
           R"""(/// Returns mutable access to RGB colors. 
/// @pre `has_rgbs()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def(
          "mutable_xyz",
          static_cast<::Eigen::Ref<Eigen::Matrix<float, 3, 1, 0, 3, 1>, 0,
                                   Eigen::InnerStride<1>> (PointCloud::*)(int)>(
              &PointCloud::mutable_xyz),
          py::arg("i"),
          R"""(/// Returns mutable access to an XYZ value. 
/// @pre `has_xyzs()` must be true.)""",
          py::return_value_policy::reference_internal)
      .def("mutable_xyzs",
           static_cast<::Eigen::Ref<Eigen::Matrix<float, 3, -1, 0, 3, -1>, 0,
                                    Eigen::OuterStride<-1>> (PointCloud::*)()>(
               &PointCloud::mutable_xyzs),
           R"""(/// Returns mutable access to XYZ values. 
/// @pre `has_xyzs()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def("normal",
           static_cast<::Eigen::Matrix<float, 3, 1, 0, 3, 1> (PointCloud::*)(
               int) const>(&PointCloud::normal),
           py::arg("i"),
           R"""(/// Returns access to a normal. 
/// @pre `has_normals()` must be true.)""")
      .def("normals",
           static_cast<::Eigen::Ref<const Eigen::Matrix<float, 3, -1, 0, 3, -1>,
                                    0, Eigen::OuterStride<-1>> (PointCloud::*)()
                           const>(&PointCloud::normals),
           R"""(/// Returns access to normals. 
/// @pre `has_normals()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def(
          "resize",
          static_cast<void (PointCloud::*)(int, bool)>(&PointCloud::resize),
          py::arg("new_size"), py::arg("skip_initialize") = bool(false),
          R"""(/// Conservative resize; will maintain existing data, and initialize new 
/// data to their invalid values. 
/// @param new_size 
///    The new size of the value. If less than the present `size()`, then 
///    the values will be truncated. If greater than the present `size()`, 
///    then the new values will be uninitialized if `skip_initialize` is not 
///    true. 
/// @param skip_initialize 
///    Do not default-initialize new values.)""")
      .def("rgb",
           static_cast<::Eigen::Matrix<unsigned char, 3, 1, 0, 3, 1> (
               PointCloud::*)(int) const>(&PointCloud::rgb),
           py::arg("i"),
           R"""(/// Returns access to an RGB color. 
/// @pre `has_rgbs()` must be true.)""")
      .def("rgbs",
           static_cast<
               ::Eigen::Ref<const Eigen::Matrix<unsigned char, 3, -1, 0, 3, -1>,
                            0, Eigen::OuterStride<-1>> (PointCloud::*)() const>(
               &PointCloud::rgbs),
           R"""(/// Returns access to RGB colors. 
/// @pre `has_rgbs()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def("size", static_cast<int (PointCloud::*)() const>(&PointCloud::size),
           R"""(/// Returns the number of points in this point cloud.)""")
      .def("xyz",
           static_cast<::Eigen::Matrix<float, 3, 1, 0, 3, 1> (PointCloud::*)(
               int) const>(&PointCloud::xyz),
           py::arg("i"),
           R"""(/// Returns access to an XYZ value. 
/// @pre `has_xyzs()` must be true.)""")
      .def("xyzs",
           static_cast<::Eigen::Ref<const Eigen::Matrix<float, 3, -1, 0, 3, -1>,
                                    0, Eigen::OuterStride<-1>> (PointCloud::*)()
                           const>(&PointCloud::xyzs),
           R"""(/// Returns access to XYZ values. 
/// @pre `has_xyzs()` must be true.)""",
           py::return_value_policy::reference_internal)
      .def_readonly_static("kDefaultColor", &PointCloud::kDefaultColor)
      .def_readonly_static("kDefaultValue", &PointCloud::kDefaultValue)

      ;
}
