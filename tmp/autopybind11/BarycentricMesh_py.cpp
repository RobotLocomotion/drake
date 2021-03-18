#include "drake/math/barycentric.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
void apb11_pydrake_BarycentricMesh_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::math;

  using PyBarycentricMesh_double_0 = double;

  py::class_<BarycentricMesh<PyBarycentricMesh_double_0>>
      PyBarycentricMesh_double(m, "BarycentricMesh_double");

  PyBarycentricMesh_double
      .def(py::init<BarycentricMesh<PyBarycentricMesh_double_0> const &>(),
           py::arg("arg0"))
      .def(py::init<::std::vector<
               std::set<double, std::less<PyBarycentricMesh_double_0>,
                        std::allocator<PyBarycentricMesh_double_0>>,
               std::allocator<
                   std::set<double, std::less<PyBarycentricMesh_double_0>,
                            std::allocator<PyBarycentricMesh_double_0>>>>>(),
           py::arg("input_grid"))
      .def("Eval",
           [](BarycentricMesh<PyBarycentricMesh_double_0> &self,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                           Eigen::OuterStride<-1>> const &mesh_values,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &input,
              Eigen::Ref<
                  ::drake::EigenPtr<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  output) { return self.Eval(mesh_values, input, output); })
      .def("Eval",
           [](BarycentricMesh<PyBarycentricMesh_double_0> &self,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                           Eigen::OuterStride<-1>> const &mesh_values,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &input) {
             return self.Eval(mesh_values, input);
           })
      .def("EvalBarycentricWeights",
           [](BarycentricMesh<PyBarycentricMesh_double_0> &self,
              ::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                           Eigen::InnerStride<1>> const &input,
              Eigen::Ref<::drake::EigenPtr<Eigen::Matrix<int, -1, 1, 0, -1, 1>>,
                         0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  mesh_indices,
              Eigen::Ref<
                  ::drake::EigenPtr<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  weights) {
             return self.EvalBarycentricWeights(input, mesh_indices, weights);
           })
      .def("MeshValuesFrom",
           [](BarycentricMesh<PyBarycentricMesh_double_0> &self,
              ::std::function<Eigen::Matrix<double, -1, 1, 0, -1, 1>(
                  const Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>,
                                   0, Eigen::InnerStride<1>> &)> const
                  &vector_func) { return self.MeshValuesFrom(vector_func); })
      .def(
          "get_all_mesh_points",
          static_cast<::Eigen::Matrix<double, -1, -1, 0, -1, -1> (
              BarycentricMesh<PyBarycentricMesh_double_0>::*)() const>(
              &BarycentricMesh<
                  PyBarycentricMesh_double_0>::get_all_mesh_points),
          R"""(/// Returns a matrix with all of the mesh points, one per column.)""")
      .def("get_input_grid",
           static_cast<::std::vector<
               std::set<double, std::less<PyBarycentricMesh_double_0>,
                        std::allocator<PyBarycentricMesh_double_0>>,
               std::allocator<
                   std::set<double, std::less<PyBarycentricMesh_double_0>,
                            std::allocator<PyBarycentricMesh_double_0>>>> const
                           &(BarycentricMesh<PyBarycentricMesh_double_0>::*)()
                               const>(
               &BarycentricMesh<PyBarycentricMesh_double_0>::get_input_grid))
      .def("get_input_size",
           static_cast<int (BarycentricMesh<PyBarycentricMesh_double_0>::*)()
                           const>(
               &BarycentricMesh<PyBarycentricMesh_double_0>::get_input_size))
      .def("get_mesh_point",
           [](BarycentricMesh<PyBarycentricMesh_double_0> &self, int index,
              Eigen::Ref<
                  ::drake::EigenPtr<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 0,
                  Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  point) { return self.get_mesh_point(index, point); })
      .def(
          "get_mesh_point",
          static_cast<::Eigen::Matrix<double, -1, 1, 0, -1, 1> (
              BarycentricMesh<PyBarycentricMesh_double_0>::*)(int) const>(
              &BarycentricMesh<PyBarycentricMesh_double_0>::get_mesh_point),
          py::arg("index"),
          R"""(/// Returns the position of a mesh point in the input space referenced by its 
/// scalar index to @p point. 
/// @param index must be in [0, get_num_mesh_points).)""")
      .def("get_num_interpolants",
           static_cast<int (BarycentricMesh<PyBarycentricMesh_double_0>::*)()
                           const>(
               &BarycentricMesh<
                   PyBarycentricMesh_double_0>::get_num_interpolants))
      .def("get_num_mesh_points",
           static_cast<int (BarycentricMesh<PyBarycentricMesh_double_0>::*)()
                           const>(
               &BarycentricMesh<
                   PyBarycentricMesh_double_0>::get_num_mesh_points))

      ;
}
