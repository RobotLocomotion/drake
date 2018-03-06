#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/math/barycentric.h"
#include "drake/math/wrap_to.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(math, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;

  m.doc() = "Bindings for //math.";

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  m.def("wrap_to", &wrap_to<T, T>, py::arg("value"), py::arg("low"),
        py::arg("high"));

  py::class_<BarycentricMesh<T>>(m, "BarycentricMesh")
      .def(py::init<BarycentricMesh<T>::MeshGrid>())
      .def("get_input_grid", &BarycentricMesh<T>::get_input_grid)
      .def("get_input_size", &BarycentricMesh<T>::get_input_size)
      .def("get_num_mesh_points", &BarycentricMesh<T>::get_num_mesh_points)
      .def("get_num_interpolants", &BarycentricMesh<T>::get_num_interpolants)
      .def("get_mesh_point", overload_cast_explicit<VectorX<T>, int>(
                                 &BarycentricMesh<T>::get_mesh_point))
      .def("get_all_mesh_points", &BarycentricMesh<T>::get_all_mesh_points)
      .def("EvalBarycentricWeights",
           [](const BarycentricMesh<T>* self,
              const Eigen::Ref<const VectorX<T>>& input) {
             const int n = self->get_num_interpolants();
             Eigen::VectorXi indices(n);
             VectorX<T> weights(n);
             self->EvalBarycentricWeights(input, &indices, &weights);
             return std::make_pair(indices, weights);
           })
      .def("Eval", overload_cast_explicit<VectorX<T>,
                                          const Eigen::Ref<const MatrixX<T>>&,
                                          const Eigen::Ref<const VectorX<T>>&>(
                       &BarycentricMesh<T>::Eval))
      .def("MeshValuesFrom", &BarycentricMesh<T>::MeshValuesFrom);
}

}  // namespace pydrake
}  // namespace drake
