#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_param_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/ball_paddle/ball_paddle.h"
#include "drake/systems/framework/diagram.h"
namespace py = pybind11;

namespace drake {
namespace examples {
namespace {
template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = drake::pydrake::GetPyParam<T>();
  {
    using Class = BallPaddle<T>;
    auto cls = drake::pydrake::DefineTemplateClassWithDefault<Class>(
        m, "BallPaddle", param);
    cls  // BR
        .def(py::init<double,
                 const std::optional<drake::math::RigidTransformd>&>(),
            py::arg("time_step"), py::arg("p_WPaddle_fixed") = std::nullopt)
        .def("plant", &Class::plant, drake::pydrake::py_rvp::reference_internal)
        .def("scene_graph", &Class::scene_graph,
            drake::pydrake::py_rvp::reference_internal)
        .def("ball_sphere_geometry_id", &Class::ball_sphere_geometry_id)
        .def("paddle_box_geometry_id", &Class::paddle_box_geometry_id)
        .def("paddle_body", &Class::paddle_body,
            drake::pydrake::py_rvp::reference_internal)
        .def("ball_body", &Class::ball_body,
            drake::pydrake::py_rvp::reference_internal)
        .def("paddle_translate_y_joint", &Class::paddle_translate_y_joint,
            drake::pydrake::py_rvp::reference_internal)
        .def("paddle_translate_z_joint", &Class::paddle_translate_z_joint,
            drake::pydrake::py_rvp::reference_internal)
        .def("AddToBuilder", &Class::AddToBuilder, py::arg("builder"))
        .def("ball_mass", &Class::ball_mass)
        .def("paddle_mass", &Class::paddle_mass)
        .def("ball_radius", &Class::ball_radius)
        .def("paddle_size", &Class::paddle_size);
  }

  {
    drake::pydrake::AddTemplateFunction(m, "ConstructBallPaddlePlant",
        &ConstructBallPaddlePlant<T>, drake::pydrake::GetPyParam<T>(),
        py::arg("paddle_mass"), py::arg("ball_mass"), py::arg("ball_radius"),
        py::arg("paddle_size"), py::arg("paddle_fixed_pose"), py::arg("plant"),
        py::arg("paddle_body_id"), py::arg("ball_body_id"),
        py::arg("paddle_translate_y_joint_index"),
        py::arg("paddle_translate_z_joint_index"),
        py::arg("ball_sphere_geometry_id"), py::arg("paddle_box_geometry_id"));
  }
}
}  // namespace

PYBIND11_MODULE(ball_paddle, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "Bindings for //drake/examples/ball_paddle/...";
  py::module_::import("pydrake.geometry");
  py::module_::import("pydrake.multibody.plant");

  drake::type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      drake::pydrake::NonSymbolicScalarPack{});
}
}  // namespace examples
}  // namespace drake
