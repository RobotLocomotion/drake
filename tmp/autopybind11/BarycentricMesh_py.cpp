#include "drake/math/barycentric.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::math;

namespace py = pybind11;
void apb11_pydrake_BarycentricMesh_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  py::class_<BarycentricMesh<double>> PyBarycentricMesh_double(
      m, "BarycentricMesh_double");

  PyBarycentricMesh_double
      .def(py::init<BarycentricMesh<double> const &>(), py::arg("arg0"))
      .def(py::init<drake::math::BarycentricMesh<double>::MeshGrid>(),
           py::arg("input_grid"))
      .def("Eval",
           [](BarycentricMesh<double> &self,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                           Eigen::OuterStride<-1>> const &mesh_values,
              ::Eigen::Ref<const Eigen::VectorXd, 0,
                           Eigen::InnerStride<1>> const &input,
              Eigen::Ref<::drake::EigenPtr<Eigen::VectorXd>, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  output) { return self.Eval(mesh_values, input, output); })
      .def(
          "Eval",
          [](BarycentricMesh<double> &self,
             ::Eigen::Ref<const Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                          Eigen::OuterStride<-1>> const &mesh_values,
             ::Eigen::Ref<const Eigen::VectorXd, 0, Eigen::InnerStride<1>> const
                 &input) { return self.Eval(mesh_values, input); })
      .def("EvalBarycentricWeights",
           [](BarycentricMesh<double> &self,
              ::Eigen::Ref<const Eigen::VectorXd, 0,
                           Eigen::InnerStride<1>> const &input,
              Eigen::Ref<::drake::EigenPtr<Eigen::Matrix<int, -1, 1, 0, -1, 1>>,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  mesh_indices,
              Eigen::Ref<::drake::EigenPtr<Eigen::VectorXd>, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  weights) {
             return self.EvalBarycentricWeights(input, mesh_indices, weights);
           })
      .def("MeshValuesFrom",
           [](BarycentricMesh<double> &self,
              ::std::function<Eigen::VectorXd(
                  const Eigen::Ref<const Eigen::VectorXd, 0,
                                   Eigen::InnerStride<1>> &)> const
                  &vector_func) { return self.MeshValuesFrom(vector_func); })
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
           [](BarycentricMesh<double> &self, int index,
              Eigen::Ref<::drake::EigenPtr<Eigen::VectorXd>, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  point) { return self.get_mesh_point(index, point); })
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
