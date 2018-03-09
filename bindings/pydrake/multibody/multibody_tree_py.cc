#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/eigen_geometry_pybind.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"

namespace drake {
namespace pydrake {

void init_math(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  m.doc() = "MultiBodyTree math functionality.";

  using T = double;

  py::class_<SpatialVector<SpatialVelocity, T>>(m, "SpatialVector")
      .def("rotational",
           [](const SpatialVector<SpatialVelocity, T>* self)
               -> const Vector3<T>& { return self->rotational(); },
           py_reference_internal)
      .def("translational",
           [](const SpatialVector<SpatialVelocity, T>* self)
               -> const Vector3<T>& { return self->translational(); },
           py_reference_internal);

  py::class_<SpatialVelocity<T>, SpatialVector<SpatialVelocity, T>>(
      m, "SpatialVelocity")
      .def(py::init())
      .def(py::init<const Eigen::Ref<const Vector3<T>>&,
                    const Eigen::Ref<const Vector3<T>>&>(),
           py::arg("w"), py::arg("v"));

  // TODO(jadecastro, eric.cousineau): Bind additional classes as necessary.
}

void init_all(py::module m) {
  // Not sure if relative imports will work in this context, so we will
  // manually spell it out.
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.multibody.multibody_tree.math import *",
      py::globals(), vars);
}

PYBIND11_MODULE(multibody_tree, m) {
  m.doc() = "MultiBodyTree functionality.";

  // N.B. At present, we canont have `math` as a submodule here, and in
  // `pydrake`. The current solution is to manually define submodules.
  // See the dicussion in #8282 for more information.
  init_math(m.def_submodule("math"));
  init_all(m.def_submodule("all"));
}

}  // namespace pydrake
}  // namespace drake
