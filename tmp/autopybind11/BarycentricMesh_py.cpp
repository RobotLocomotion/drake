#include "drake/math/barycentric.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::math;

namespace py = pybind11;

template <typename Class, typename... Options>
py::class_<Class, Options...> DefineTemplateClass(py::handle scope,
                                                  const char *name,
                                                  const char *doc_string = "") {
  py::class_<Class, Options...> py_class(scope, name, doc_string);
  return py_class;
};

void apb11_pydrake_BarycentricMesh_py_register(py::module &m) {
  // Instantiation of BarycentricMesh<double>
  auto PyBarycentricMesh_double =
      DefineTemplateClass<BarycentricMesh<double>>(m, "BarycentricMesh_double");

  PyBarycentricMesh_double
      .def(py::init<BarycentricMesh<double> const &>(), py::arg("arg0"))
      .def(py::init<drake::math::BarycentricMesh<double>::MeshGrid>(),
           py::arg("input_grid"))
      .def("Eval",
           static_cast<void (BarycentricMesh<double>::*)(
               ::Eigen::Ref<const Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                            Eigen::OuterStride<-1>> const &,
               ::Eigen::Ref<const Eigen::VectorXd, 0,
                            Eigen::InnerStride<1>> const &,
               ::drake::EigenPtr<Eigen::VectorXd>) const>(
               &BarycentricMesh<double>::Eval),
           py::arg("mesh_values"), py::arg("input"), py::arg("output"))
      .def("Eval",
           static_cast<Eigen::VectorXd (BarycentricMesh<double>::*)(
               ::Eigen::Ref<const Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                            Eigen::OuterStride<-1>> const &,
               ::Eigen::Ref<const Eigen::VectorXd, 0,
                            Eigen::InnerStride<1>> const &) const>(
               &BarycentricMesh<double>::Eval),
           py::arg("mesh_values"), py::arg("input"))
      .def("EvalBarycentricWeights",
           static_cast<void (BarycentricMesh<double>::*)(
               ::Eigen::Ref<const Eigen::VectorXd, 0,
                            Eigen::InnerStride<1>> const &,
               ::drake::EigenPtr<Eigen::Matrix<int, -1, 1, 0, -1, 1>>,
               ::drake::EigenPtr<Eigen::VectorXd>) const>(
               &BarycentricMesh<double>::EvalBarycentricWeights),
           py::arg("input"), py::arg("mesh_indices"), py::arg("weights"))
      .def("MeshValuesFrom",
           static_cast<::Eigen::Matrix<double, -1, -1, 0, -1, -1> (
               BarycentricMesh<double>::*)(
               ::std::function<Eigen::VectorXd(
                   const Eigen::Ref<const Eigen::VectorXd, 0,
                                    Eigen::InnerStride<1>> &)> const &) const>(
               &BarycentricMesh<double>::MeshValuesFrom),
           py::arg("vector_func"))
      .def("get_all_mesh_points",
           static_cast<::Eigen::Matrix<double, -1, -1, 0, -1, -1> (
               BarycentricMesh<double>::*)() const>(
               &BarycentricMesh<double>::get_all_mesh_points))
      .def("get_input_grid",
           static_cast<drake::math::BarycentricMesh<double>::MeshGrid const &(
               BarycentricMesh<double>::*)() const>(
               &BarycentricMesh<double>::get_input_grid))
      .def("get_input_size",
           static_cast<int (BarycentricMesh<double>::*)() const>(
               &BarycentricMesh<double>::get_input_size))
      .def("get_mesh_point",
           static_cast<void (BarycentricMesh<double>::*)(
               int, ::drake::EigenPtr<Eigen::VectorXd>) const>(
               &BarycentricMesh<double>::get_mesh_point),
           py::arg("index"), py::arg("point"))
      .def("get_mesh_point",
           static_cast<Eigen::VectorXd (BarycentricMesh<double>::*)(int) const>(
               &BarycentricMesh<double>::get_mesh_point),
           py::arg("index"))
      .def("get_num_interpolants",
           static_cast<int (BarycentricMesh<double>::*)() const>(
               &BarycentricMesh<double>::get_num_interpolants))
      .def("get_num_mesh_points",
           static_cast<int (BarycentricMesh<double>::*)() const>(
               &BarycentricMesh<double>::get_num_mesh_points))

      ;
}
