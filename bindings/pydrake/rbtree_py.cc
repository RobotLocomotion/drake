#include <iostream>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsers/package_map.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(_rbtree_py, m) {
  m.doc() = "Bindings for the RigidBodyTree class";

  using drake::multibody::joints::FloatingBaseType;
  using drake::parsers::PackageMap;
  namespace sdf = drake::parsers::sdf;

  py::module::import("pydrake.multibody.shapes");
  py::module::import("pydrake.parsers");

  py::enum_<FloatingBaseType>(m, "FloatingBaseType")
    .value("kFixed", FloatingBaseType::kFixed)
    .value("kRollPitchYaw", FloatingBaseType::kRollPitchYaw)
    .value("kQuaternion", FloatingBaseType::kQuaternion);

  py::class_<RigidBodyTree<double>>(m, "RigidBodyTree")
    .def(py::init<>())
    .def("__init__",
         [](RigidBodyTree<double> &instance,
            const std::string& urdf_filename,
            const PackageMap& pmap,
            FloatingBaseType floating_base_type
            ) {
          new (&instance) RigidBodyTree<double>();
          drake::parsers::urdf::
            AddModelInstanceFromUrdfFileSearchingInRosPackages(
            urdf_filename,
            pmap,
            floating_base_type,
            nullptr,
            &instance);
        },
        py::arg("urdf_filename"),
        py::arg("package_map"),
        py::arg("floating_base_type") = FloatingBaseType::kRollPitchYaw)
    .def("__init__",
         [](RigidBodyTree<double> &instance,
            const std::string& urdf_filename,
            FloatingBaseType floating_base_type
            ) {
          new (&instance) RigidBodyTree<double>();
          drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf_filename, floating_base_type, &instance);
        },
        py::arg("urdf_filename"),
        py::arg("floating_base_type") = FloatingBaseType::kRollPitchYaw)
    .def("__init__",
         [](RigidBodyTree<double> &instance,
            const std::string& urdf_filename,
            const std::string& joint_type) {
            // FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2
            FloatingBaseType floating_base_type;
            std::cerr << "WARNING: passing joint_type as a string is "
              << "deprecated. Please pass a FloatingBaseType value such as "
              << "pydrake.rbtree.FloatingBaseType.kRollPitchYaw" << std::endl;
            if (joint_type == "FIXED") {
              floating_base_type = FloatingBaseType::kFixed;
            } else if (joint_type == "ROLLPITCHYAW") {
              floating_base_type = FloatingBaseType::kRollPitchYaw;
            } else if (joint_type == "QUATERNION") {
              floating_base_type = FloatingBaseType::kQuaternion;
            } else {
              throw(std::invalid_argument("Joint type not supported"));
            }
            new (&instance) RigidBodyTree<double>();
            drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                urdf_filename, floating_base_type, &instance);
        },
        py::arg("urdf_filename"), py::arg("joint_type") = "ROLLPITCHYAW"
      )
    .def("getRandomConfiguration", [](const RigidBodyTree<double>& tree) {
      std::default_random_engine generator(std::random_device {}());
      return tree.getRandomConfiguration(generator);
    })
    .def("getZeroConfiguration", &RigidBodyTree<double>::getZeroConfiguration)
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const Eigen::VectorXd& q) {
      return tree.doKinematics(q);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const Eigen::VectorXd& q,
                            const Eigen::VectorXd& v) {
      return tree.doKinematics(q, v);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const VectorX<AutoDiffXd>& q) {
      return tree.doKinematics(q);
    })
    .def("doKinematics", [](const RigidBodyTree<double>& tree,
                            const VectorX<AutoDiffXd>& q,
                            const VectorX<AutoDiffXd>& v) {
      return tree.doKinematics(q, v);
    })
    .def("CalcBodyPoseInWorldFrame", [](const RigidBodyTree<double>& tree,
                                        const KinematicsCache<double> &cache,
                                        const RigidBody<double> &body) {
      return tree.CalcBodyPoseInWorldFrame(cache, body).matrix();
    })
    .def("centerOfMass", &RigidBodyTree<double>::centerOfMass<double>,
         py::arg("cache"),
         py::arg("model_instance_id_set") =
           RigidBodyTreeConstants::default_model_instance_id_set)
    .def("centerOfMassJacobian",
         &RigidBodyTree<double>::centerOfMassJacobian<double>,
         py::arg("cache"),
         py::arg("model_instance_id_set") =
           RigidBodyTreeConstants::default_model_instance_id_set,
         py::arg("in_terms_of_qdot") = false)
    .def("get_num_bodies", &RigidBodyTree<double>::get_num_bodies)
    .def("get_num_frames", &RigidBodyTree<double>::get_num_frames)
    .def("get_num_actuators", &RigidBodyTree<double>::get_num_actuators)
    .def("getBodyOrFrameName",
         &RigidBodyTree<double>::getBodyOrFrameName,
         py::arg("body_or_frame_id"))
    .def("number_of_positions", &RigidBodyTree<double>::get_num_positions)
    .def("get_num_positions", &RigidBodyTree<double>::get_num_positions)
    .def("number_of_velocities", &RigidBodyTree<double>::get_num_velocities)
    .def("get_num_velocities", &RigidBodyTree<double>::get_num_velocities)
    .def("get_body", &RigidBodyTree<double>::get_body,
         py::return_value_policy::reference)
    .def("get_position_name", &RigidBodyTree<double>::get_position_name)
    .def("transformPoints", [](const RigidBodyTree<double>& tree,
                               const KinematicsCache<double>& cache,
                               const Eigen::Matrix<double, 3,
                                                   Eigen::Dynamic>& points,
                               int from_body_or_frame_ind,
                               int to_body_or_frame_ind) {
      return tree.transformPoints(cache, points,
                                  from_body_or_frame_ind, to_body_or_frame_ind);
    })
    .def("transformPoints", [](const RigidBodyTree<double>& tree,
                               const KinematicsCache<AutoDiffXd>& cache,
                               const Eigen::Matrix<double, 3,
                                                   Eigen::Dynamic>& points,
                               int from_body_or_frame_ind,
                               int to_body_or_frame_ind) {
      return tree.transformPoints(cache, points,
                                  from_body_or_frame_ind, to_body_or_frame_ind);
    })
    .def("relativeTransform", [](const RigidBodyTree<double>& tree,
                                  const KinematicsCache<double>& cache,
                                  int base_or_frame_ind,
                                  int body_or_frame_ind) {
      return tree.relativeTransform(cache, base_or_frame_ind,
        body_or_frame_ind).matrix();
    })
    .def("relativeTransform", [](const RigidBodyTree<double>& tree,
                                  const KinematicsCache<AutoDiffXd>& cache,
                                  int base_or_frame_ind,
                                  int body_or_frame_ind) {
      return tree.relativeTransform(cache, base_or_frame_ind,
        body_or_frame_ind).matrix();
    })
    .def("addFrame", &RigidBodyTree<double>::addFrame)
    .def("FindBody", [](const RigidBodyTree<double>& self,
                        const std::string& body_name,
                        const std::string& model_name = "",
                        int model_id = -1) {
      return self.FindBody(body_name, model_name, model_id);
    }, py::arg("body_name"),
       py::arg("model_name") = "",
       py::arg("model_id") = -1,
       py::return_value_policy::reference)
    .def("world",
         static_cast<RigidBody<double>& (RigidBodyTree<double>::*)()>(
             &RigidBodyTree<double>::world),
         py::return_value_policy::reference)
    .def("findFrame", &RigidBodyTree<double>::findFrame,
         py::arg("frame_name"), py::arg("model_id") = -1)
    .def("getTerrainContactPoints",
         [](const RigidBodyTree<double>& self,
            const RigidBody<double>& body,
            const std::string& group_name = "") {
          auto pts = Eigen::Matrix3Xd(3, 0);
          self.getTerrainContactPoints(body, &pts, group_name);
          return pts;
        }, py::arg("body"), py::arg("group_name")="")
    .def("massMatrix", &RigidBodyTree<double>::massMatrix<double>)
    .def("dynamicsBiasTerm", &RigidBodyTree<double>::dynamicsBiasTerm<double>,
         py::arg("cache"), py::arg("external_wrenches"),
         py::arg("include_velocity_terms") = true)
    .def("inverseDynamics", &RigidBodyTree<double>::inverseDynamics<double>,
         py::arg("cache"),
         py::arg("external_wrenches"),
         py::arg("vd"),
         py::arg("include_velocity_terms") = true)
    .def("frictionTorques",
         [](const RigidBodyTree<double>* self, const VectorX<double>& v) {
           return self->frictionTorques(v);
         })
    .def_readonly("B", &RigidBodyTree<double>::B);

  py::class_<KinematicsCache<double> >(m, "KinematicsCacheDouble");
  py::class_<KinematicsCache<AutoDiffXd> >(m, "KinematicsCacheAutoDiffXd");

  py::class_<RigidBody<double> >(m, "RigidBody")
    .def("get_name", &RigidBody<double>::get_name)
    .def("get_body_index", &RigidBody<double>::get_body_index)
    .def("get_center_of_mass", &RigidBody<double>::get_center_of_mass)
    .def("get_visual_elements", &RigidBody<double>::get_visual_elements);

  py::class_<RigidBodyFrame<double>,
             std::shared_ptr<RigidBodyFrame<double> > >(m, "RigidBodyFrame")
    .def(py::init<const std::string&,
                  RigidBody<double>*,
                  const Eigen::VectorXd&,
                  const Eigen::VectorXd& >())
    .def("get_frame_index", &RigidBodyFrame<double>::get_frame_index);

  m.def("AddModelInstanceFromUrdfStringSearchingInRosPackages",
        &drake::parsers::urdf::\
          AddModelInstanceFromUrdfStringSearchingInRosPackages);
  m.def("AddModelInstancesFromSdfString",
        &sdf::AddModelInstancesFromSdfString);
  m.def("AddModelInstancesFromSdfStringSearchingInRosPackages",
        &sdf::AddModelInstancesFromSdfStringSearchingInRosPackages);
}

}  // namespace pydrake
}  // namespace drake
