#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace py = pybind11;

PYBIND11_PLUGIN(_rbtree) {
  py::module m("_rbtree", "Bindings for the RigidBodyTree class");

  py::class_<RigidBodyTree<double>>(m, "RigidBodyTree")
    .def("__init__",
         [](RigidBodyTree<double> &instance, const std::string& urdf_filename, const std::string& joint_type) {
            // FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2
            drake::multibody::joints::FloatingBaseType floating_base_type;

            if (joint_type == "FIXED")
              floating_base_type = drake::multibody::joints::kFixed;
            else if (joint_type == "ROLLPITCHYAW")
              floating_base_type = drake::multibody::joints::kRollPitchYaw;
            else if (joint_type == "QUATERNION")
              floating_base_type = drake::multibody::joints::kQuaternion;
            else {
              throw(std::invalid_argument("Joint type not supported"));
            }
            new (&instance) RigidBodyTree<double>();
            drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                urdf_filename, floating_base_type, &instance);
        }
      );

  return m.ptr();
}
