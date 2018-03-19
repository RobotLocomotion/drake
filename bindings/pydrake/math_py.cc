#include <cmath>

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

  // General math overloads.
  // N.B. Additional overloads will be added for autodiff, symbolic, etc, by
  // those respective modules.
  // TODO(eric.cousineau): If possible, delegate these to NumPy UFuncs, either
  // using __array_ufunc__ or user dtypes.
  // N.B. The ordering in which the overloads are resolved will change based on
  // when modules are loaded. However, there should not be ambiguous implicit
  // conversions between autodiff and symbolic, and double overloads should
  // always occur first, so it shouldn't be a problem.
  // See `math_overloads_test`, which tests this specifically.
  m
      .def("log", [](double x) { return log(x); })
      .def("abs", [](double x) { return fabs(x); })
      .def("exp", [](double x) { return exp(x); })
      .def("sqrt", [](double x) { return sqrt(x); })
      .def("pow", [](double x, double y) { return pow(x, y); })
      .def("sin", [](double x) { return sin(x); })
      .def("cos", [](double x) { return cos(x); })
      .def("tan", [](double x) { return tan(x); })
      .def("asin", [](double x) { return asin(x); })
      .def("acos", [](double x) { return acos(x); })
      .def("atan", [](double x) { return atan(x); })
      .def("atan2", [](double y, double x) { return atan2(y, x); },
           py::arg("y"), py::arg("x"))
      .def("sinh", [](double x) { return sinh(x); })
      .def("cosh", [](double x) { return cosh(x); })
      .def("tanh", [](double x) { return tanh(x); })
      .def("min", [](double x, double y) { return fmin(x, y); })
      .def("max", [](double x, double y) { return fmax(x, y); })
      .def("ceil", [](double x) { return ceil(x); })
      .def("floor", [](double x) { return floor(x); });
}

}  // namespace pydrake
}  // namespace drake
