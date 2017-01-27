#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "drake/multibody/parsers/package_map.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"

namespace py = pybind11;

PYBIND11_PLUGIN(_rbtree) {
  py::module m("_rbtree", "Bindings for the RigidBodyTree class");

  py::class_<RigidBodyTree<double>>(m, "RigidBodyTree")
    .def(py::init<>())
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
        },
        py::arg("urdf_filename"), py::arg("joint_type")="ROLLPITCHYAW"
      )
    .def("getRandomConfiguration", [](const RigidBodyTree<double>& tree) {
      std::default_random_engine generator(std::random_device{}());
      return tree.getRandomConfiguration(generator);
    })
    .def("getZeroConfiguration", &RigidBodyTree<double>::getZeroConfiguration)
    .def("doKinematics", [](const RigidBodyTree<double>& tree, const Eigen::VectorXd& q) {
      return tree.doKinematics(q);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree, 
                            const Eigen::VectorXd& q,
                            const Eigen::VectorXd& v) {
      return tree.doKinematics(q, v);
    })
    .def("centerOfMass", &RigidBodyTree<double>::centerOfMass<double>,
         py::arg("cache"), 
         py::arg("model_instance_id_set") = RigidBodyTreeConstants::default_model_instance_id_set)
    .def("centerOfMassJacobian", &RigidBodyTree<double>::centerOfMassJacobian<double>,
         py::arg("cache"), 
         py::arg("model_instance_id_set") = RigidBodyTreeConstants::default_model_instance_id_set,
         py::arg("in_terms_of_qdot") = false)
    .def("get_num_bodies", &RigidBodyTree<double>::get_num_bodies)
    .def("number_of_positions", &RigidBodyTree<double>::get_num_positions)
    .def("get_num_positions", &RigidBodyTree<double>::get_num_positions)
    .def("number_of_velocities", &RigidBodyTree<double>::get_num_velocities)
    .def("get_num_velocities", &RigidBodyTree<double>::get_num_velocities)
    .def("transformPoints", [](const RigidBodyTree<double>& tree,
                               const KinematicsCache<double>& cache,
                               const Eigen::Matrix<double, 3, Eigen::Dynamic>& points,
                               int from_body_or_frame_ind,
                               int to_body_or_frame_ind) {
      return tree.transformPoints(cache, points, from_body_or_frame_ind, to_body_or_frame_ind);
    })
    ;

  py::class_<KinematicsCache<double>>(m, "KinematicsCache");

  py::class_<drake::parsers::PackageMap>(m, "PackageMap")
    .def(py::init<>())
    ;

  py::class_<RigidBodyFrame<double>>(m, "RigidBodyFrame");

  py::enum_<drake::multibody::joints::FloatingBaseType>(m, "FloatingBaseType")
    .value("kFixed", drake::multibody::joints::FloatingBaseType::kFixed)
    .value("kRollPitchYaw", drake::multibody::joints::FloatingBaseType::kRollPitchYaw)
    .value("kQuaternion", drake::multibody::joints::FloatingBaseType::kQuaternion);

  m.def("AddModelInstanceFromUrdfStringSearchingInRosPackages", 
        &drake::parsers::urdf::AddModelInstanceFromUrdfStringSearchingInRosPackages);

  return m.ptr();
}
