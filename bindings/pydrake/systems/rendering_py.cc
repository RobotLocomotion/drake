#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include <Eigen/Dense>

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rendering, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::rendering;
  constexpr auto& doc = pydrake_doc.drake.systems.rendering;

  m.doc() = "Bindings for the rendering portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  using T = double;

  // See the todo in multibody_position_to_geometry_pose.h. This should
  // ultimately move into a different module.
  py::class_<MultibodyPositionToGeometryPose<T>, LeafSystem<T>>(m,
      "MultibodyPositionToGeometryPose",
      doc.MultibodyPositionToGeometryPose.doc)
      .def(py::init<const multibody::MultibodyPlant<T>&, bool>(),
          py::arg("plant"), py::arg("input_multibody_state") = false,
          // Keep alive, reference: `self` keeps `plant` alive.
          py::keep_alive<1, 2>(),
          doc.MultibodyPositionToGeometryPose.ctor
              .doc_2args_plant_input_multibody_state);
}

}  // namespace pydrake
}  // namespace drake
