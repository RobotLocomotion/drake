#include <iostream>
#include <memory>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsers/package_map.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_actuator.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using std::make_unique;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(rigid_body_tree, m) {
  constexpr auto& doc = pydrake_doc;

  m.doc() = "Bindings for the RigidBodyTree class";

  using drake::multibody::joints::FloatingBaseType;
  using drake::parsers::PackageMap;
  namespace sdf = drake::parsers::sdf;
  using std::shared_ptr;

  py::module::import("pydrake.attic.multibody.collision");
  py::module::import("pydrake.attic.multibody.joints");
  py::module::import("pydrake.attic.multibody.parsers");
  py::module::import("pydrake.attic.multibody.rigid_body");
  py::module::import("pydrake.attic.multibody.shapes");
  py::module::import("pydrake.common.eigen_geometry");

  constexpr auto& joints_doc = doc.drake.multibody.joints;
  py::enum_<FloatingBaseType>(
      m, "FloatingBaseType", doc.drake.multibody.joints.FloatingBaseType.doc)
      .value("kFixed", FloatingBaseType::kFixed,
          joints_doc.FloatingBaseType.kFixed.doc)
      .value("kRollPitchYaw", FloatingBaseType::kRollPitchYaw,
          joints_doc.FloatingBaseType.kRollPitchYaw.doc)
      .value("kQuaternion", FloatingBaseType::kQuaternion,
          joints_doc.FloatingBaseType.kQuaternion.doc)
      .value("kExperimentalMultibodyPlantStyle",
          FloatingBaseType::kExperimentalMultibodyPlantStyle,
          joints_doc.FloatingBaseType.kExperimentalMultibodyPlantStyle.doc);

  // TODO(eric.cousineau): Try to decouple these APIs so that `rigid_body_tree`
  // and `parsers` do not form a dependency cycle.
  py::class_<RigidBodyTree<double>> tree_cls(
      m, "RigidBodyTree", doc.RigidBodyTree.doc);
  tree_cls  // BR
      .def(py::init<>())
      .def(py::init([](const std::string& urdf_filename, const PackageMap& pmap,
                        FloatingBaseType floating_base_type) {
        auto instance = make_unique<RigidBodyTree<double>>();
        drake::parsers::urdf::
            AddModelInstanceFromUrdfFileSearchingInRosPackages(urdf_filename,
                pmap, floating_base_type, nullptr, instance.get());
        return instance;
      }),
          py::arg("urdf_filename"), py::arg("package_map"),
          py::arg("floating_base_type") = FloatingBaseType::kRollPitchYaw,
          // N.B. There is no corresponding C++ method, so the docstring here
          // is a literal, not a reference to documentation_pybind.h
          "Constructs a tree and loads an URDF file.")
      .def(py::init([](const std::string& urdf_filename,
                        FloatingBaseType floating_base_type) {
        auto instance = make_unique<RigidBodyTree<double>>();
        drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf_filename, floating_base_type, instance.get());
        return instance;
      }),
          py::arg("urdf_filename"),
          py::arg("floating_base_type") = FloatingBaseType::kRollPitchYaw,
          // N.B. There is no corresponding C++ method, so the docstring here
          // is a literal, not a reference to documentation_pybind.h
          "Constructs a tree and loads an URDF file.")
      .def(py::init([](const std::string& urdf_filename,
                        const std::string& joint_type) {
        // FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2
        FloatingBaseType floating_base_type;
        std::cerr << "WARNING: passing joint_type as a string is "
                  << "deprecated. Please pass a FloatingBaseType value such as "
                  << "FloatingBaseType.kRollPitchYaw" << std::endl;
        if (joint_type == "FIXED") {
          floating_base_type = FloatingBaseType::kFixed;
        } else if (joint_type == "ROLLPITCHYAW") {
          floating_base_type = FloatingBaseType::kRollPitchYaw;
        } else if (joint_type == "QUATERNION") {
          floating_base_type = FloatingBaseType::kQuaternion;
        } else {
          throw(std::invalid_argument("Joint type not supported"));
        }
        auto instance = make_unique<RigidBodyTree<double>>();
        drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            urdf_filename, floating_base_type, instance.get());
        return instance;
      }),
          py::arg("urdf_filename"), py::arg("joint_type") = "ROLLPITCHYAW",
          // N.B. There is no corresponding C++ method, so the docstring here
          // is a literal, not a reference to documentation_pybind.h
          "Constructs a tree and loads an URDF file.")
      .def("compile", &RigidBodyTree<double>::compile,
          doc.RigidBodyTree.compile.doc)
      .def("initialized", &RigidBodyTree<double>::initialized,
          doc.RigidBodyTree.initialized.doc)
      .def("get_bodies",
          [](const RigidBodyTree<double>& tree) {
            auto& body_unique_ptrs = tree.get_bodies();
            py::list py_bodies;
            // Get self-reference to so that we can leverage keep-alive
            // behavior.
            py::object self = py::cast(&tree);
            for (auto& body_unique_ptr : body_unique_ptrs) {
              py::object body_py =
                  py::cast(body_unique_ptr.get(), py_reference);
              py_bodies.append(py_keep_alive(body_py, self));
            }
            return py_bodies;
          },
          doc.RigidBodyTree.get_bodies.doc)
      .def("get_frames", &RigidBodyTree<double>::get_frames,
          doc.RigidBodyTree.get_frames.doc)
      .def("drawKinematicTree", &RigidBodyTree<double>::drawKinematicTree,
          doc.RigidBodyTree.drawKinematicTree.doc)
      .def("getRandomConfiguration",
          [](const RigidBodyTree<double>& tree) {
            std::random_device device;
            std::default_random_engine generator(device());
            return tree.getRandomConfiguration(generator);
          },
          doc.RigidBodyTree.getRandomConfiguration.doc)
      .def("getZeroConfiguration", &RigidBodyTree<double>::getZeroConfiguration,
          doc.RigidBodyTree.getZeroConfiguration.doc)
      .def("CalcBodyPoseInWorldFrame",
          [](const RigidBodyTree<double>& tree,
              const KinematicsCache<double>& cache,
              const RigidBody<double>& body) {
            return tree.CalcBodyPoseInWorldFrame(cache, body).matrix();
          },
          doc.RigidBodyTree.CalcBodyPoseInWorldFrame.doc)
      .def("get_num_bodies", &RigidBodyTree<double>::get_num_bodies,
          doc.RigidBodyTree.get_num_bodies.doc)
      .def("get_num_frames", &RigidBodyTree<double>::get_num_frames,
          doc.RigidBodyTree.get_num_frames.doc)
      .def("get_num_actuators", &RigidBodyTree<double>::get_num_actuators,
          doc.RigidBodyTree.get_num_actuators.doc)
      .def("get_num_model_instances",
          &RigidBodyTree<double>::get_num_model_instances,
          doc.RigidBodyTree.get_num_model_instances.doc)
      .def("getBodyOrFrameName", &RigidBodyTree<double>::getBodyOrFrameName,
          py::arg("body_or_frame_id"), doc.RigidBodyTree.getBodyOrFrameName.doc)
      .def("number_of_positions", &RigidBodyTree<double>::get_num_positions,
          doc.RigidBodyTree.number_of_positions.doc_deprecated)
      .def("get_num_positions", &RigidBodyTree<double>::get_num_positions,
          doc.RigidBodyTree.get_num_positions.doc)
      .def("number_of_velocities", &RigidBodyTree<double>::get_num_velocities,
          doc.RigidBodyTree.number_of_velocities.doc_deprecated)
      .def("get_num_velocities", &RigidBodyTree<double>::get_num_velocities,
          doc.RigidBodyTree.get_num_velocities.doc)
      .def("get_body", &RigidBodyTree<double>::get_body,
          py::return_value_policy::reference, doc.RigidBodyTree.get_body.doc)
      .def("get_position_name", &RigidBodyTree<double>::get_position_name,
          doc.RigidBodyTree.get_position_name.doc)
      .def("add_rigid_body", &RigidBodyTree<double>::add_rigid_body,
          doc.RigidBodyTree.add_rigid_body.doc)
      .def("addCollisionElement", &RigidBodyTree<double>::addCollisionElement,
          doc.RigidBodyTree.addCollisionElement.doc)
      .def("AddCollisionFilterGroupMember",
          &RigidBodyTree<double>::AddCollisionFilterGroupMember,
          py::arg("group_name"), py::arg("body_name"), py::arg("model_id"),
          doc.RigidBodyTree.AddCollisionFilterGroupMember.doc)
      .def("AddCollisionFilterIgnoreTarget",
          &RigidBodyTree<double>::AddCollisionFilterIgnoreTarget,
          py::arg("group_name"), py::arg("target_group_name"),
          doc.RigidBodyTree.AddCollisionFilterIgnoreTarget.doc)
      .def("DefineCollisionFilterGroup",
          &RigidBodyTree<double>::DefineCollisionFilterGroup, py::arg("name"),
          doc.RigidBodyTree.DefineCollisionFilterGroup.doc)
      .def("collisionDetectFromPoints",
          [](RigidBodyTree<double>& tree, const KinematicsCache<double>& cache,
              const Eigen::Matrix3Xd& points, bool use_margins) {
            Eigen::VectorXd phi;
            Eigen::Matrix3Xd normal;
            Eigen::Matrix3Xd x;
            Eigen::Matrix3Xd body_x;
            std::vector<int> body_idx;
            tree.collisionDetectFromPoints(
                cache, points, phi, normal, x, body_x, body_idx, use_margins);
            return std::tuple<Eigen::VectorXd, Eigen::Matrix3Xd,
                Eigen::Matrix3Xd, Eigen::Matrix3Xd, std::vector<int>>(
                phi, normal, x, body_x, body_idx);
          },
          py::arg("cache"), py::arg("points"), py::arg("use_margins"),
          doc.RigidBodyTree.collisionDetectFromPoints.doc)
      .def("FindCollisionElement", &RigidBodyTree<double>::FindCollisionElement,
          py::arg("id"), py::return_value_policy::reference,
          doc.RigidBodyTree.FindCollisionElement.doc)
      .def("addFrame", &RigidBodyTree<double>::addFrame, py::arg("frame"),
          doc.RigidBodyTree.addFrame.doc)
      .def("FindBody",
          [](const RigidBodyTree<double>& self, const std::string& body_name,
              const std::string& model_name = "", int model_id = -1) {
            return self.FindBody(body_name, model_name, model_id);
          },
          py::arg("body_name"), py::arg("model_name") = "",
          py::arg("model_id") = -1, py::return_value_policy::reference,
          doc.RigidBodyTree.FindBody.doc_3args)
      .def("FindBodyIndex", &RigidBodyTree<double>::FindBodyIndex,
          py::arg("body_name"), py::arg("model_id") = -1,
          doc.RigidBodyTree.FindBodyIndex.doc)
      .def("FindChildBodyOfJoint",
          [](const RigidBodyTree<double>& self, const std::string& joint_name,
              int model_id) {
            return self.FindChildBodyOfJoint(joint_name, model_id);
          },
          py::arg("joint_name"), py::arg("model_id") = -1,
          py::return_value_policy::reference,
          doc.RigidBodyTree.FindChildBodyOfJoint.doc)
      .def("FindIndexOfChildBodyOfJoint",
          &RigidBodyTree<double>::FindIndexOfChildBodyOfJoint,
          py::arg("joint_name"), py::arg("model_id") = -1,
          doc.RigidBodyTree.FindIndexOfChildBodyOfJoint.doc)
      .def("world",
          static_cast<RigidBody<double>& (RigidBodyTree<double>::*)()>(
              &RigidBodyTree<double>::world),
          py::return_value_policy::reference,
          doc.RigidBodyTree.world.doc_0args_nonconst)
      .def("findFrame", &RigidBodyTree<double>::findFrame,
          py::arg("frame_name"), py::arg("model_id") = -1,
          doc.RigidBodyTree.findFrame.doc)
      .def("getTerrainContactPoints",
          [](const RigidBodyTree<double>& self, const RigidBody<double>& body,
              const std::string& group_name = "") {
            auto pts = Eigen::Matrix3Xd(3, 0);
            self.getTerrainContactPoints(body, &pts, group_name);
            return pts;
          },
          py::arg("body"), py::arg("group_name") = "",
          doc.RigidBodyTree.getTerrainContactPoints.doc)
      .def_readonly("B", &RigidBodyTree<double>::B, doc.RigidBodyTree.B.doc)
      .def_readonly("joint_limit_min", &RigidBodyTree<double>::joint_limit_min,
          doc.RigidBodyTree.joint_limit_min.doc)
      .def_readonly("joint_limit_max", &RigidBodyTree<double>::joint_limit_max,
          doc.RigidBodyTree.joint_limit_max.doc)
      // N.B. This will return *copies* of the actuators.
      // N.B. `def_readonly` implicitly adds `reference_internal` to the getter,
      // which is necessary since an actuator references a `RigidBody` that is
      // most likely owned by this tree.
      .def_readonly("actuators", &RigidBodyTree<double>::actuators,
          doc.RigidBodyTree.actuators.doc)
      .def("GetActuator", &RigidBodyTree<double>::GetActuator,
          py_reference_internal, doc.RigidBodyTree.GetActuator.doc)
      .def("FindBaseBodies", &RigidBodyTree<double>::FindBaseBodies,
          py::arg("model_instance_id") = -1,
          doc.RigidBodyTree.FindBaseBodies.doc)
      .def("addDistanceConstraint",
          &RigidBodyTree<double>::addDistanceConstraint,
          py::arg("bodyA_index_in"), py::arg("r_AP_in"),
          py::arg("bodyB_index_in"), py::arg("r_BQ_in"), py::arg("distance_in"),
          doc.RigidBodyTree.addDistanceConstraint.doc)
      .def("getNumPositionConstraints",
          &RigidBodyTree<double>::getNumPositionConstraints,
          doc.RigidBodyTree.getNumPositionConstraints.doc)
      .def("Clone", &RigidBodyTree<double>::Clone, doc.RigidBodyTree.Clone.doc)
      .def("__copy__", &RigidBodyTree<double>::Clone);

  // This lambda defines RigidBodyTree methods which are defined for a given
  // templated type. The methods are either (a) direct explicit template
  // instantiations defined in `rigid_body_tree.cc` or (b) defined in
  // `rigid_body_tree.h` and dependent upon methods of type (a).
  // Methods of type (a) follow the same order as the explicit instantiations in
  // `rigid_body_tree.cc`; if the method is not yet bound, the name has a
  // comment as a placeholder.
  // Methods of type (b) are declared below methods of type (a).
  // N.B. Capturing `&doc` should not be required; workaround per #9600.
  auto add_rigid_body_tree_typed_methods = [m, &doc, &tree_cls](auto dummy) {
    // N.B. The header files use `Scalar` as the scalar-type template
    // parameter, but `T` is used here for brevity.
    using T = decltype(dummy);
    // Type (a) methods:
    tree_cls
        .def("massMatrix", &RigidBodyTree<double>::massMatrix<T>,
            doc.RigidBodyTree.massMatrix.doc)
        .def("centerOfMass", &RigidBodyTree<double>::centerOfMass<T>,
            py::arg("cache"),
            py::arg("model_instance_id_set") =
                RigidBodyTreeConstants::default_model_instance_id_set,
            doc.RigidBodyTree.centerOfMass.doc)
        .def("transformVelocityToQDot",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, const VectorX<T>& v) {
              return tree.transformVelocityToQDot(cache, v);
            },
            doc.RigidBodyTree.transformVelocityToQDot.doc)
        .def("transformQDotToVelocity",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, const VectorX<T>& qdot) {
              return tree.transformQDotToVelocity(cache, qdot);
            },
            doc.RigidBodyTree.transformQDotToVelocity.doc)
        .def("GetVelocityToQDotMapping",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache) {
              return tree.GetVelocityToQDotMapping(cache);
            },
            doc.RigidBodyTree.GetVelocityToQDotMapping.doc)
        .def("GetQDotToVelocityMapping",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache) {
              return tree.GetQDotToVelocityMapping(cache);
            },
            doc.RigidBodyTree.GetQDotToVelocityMapping.doc)
        .def("dynamicsBiasTerm", &RigidBodyTree<double>::dynamicsBiasTerm<T>,
            py::arg("cache"), py::arg("external_wrenches"),
            py::arg("include_velocity_terms") = true,
            doc.RigidBodyTree.dynamicsBiasTerm.doc)
        .def("geometricJacobian",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, int base_body_or_frame_ind,
                int end_effector_body_or_frame_ind,
                int expressed_in_body_or_frame_ind, bool in_terms_of_qdot) {
              std::vector<int> v_indices;
              auto J = tree.geometricJacobian(cache, base_body_or_frame_ind,
                  end_effector_body_or_frame_ind,
                  expressed_in_body_or_frame_ind, in_terms_of_qdot, &v_indices);
              return py::make_tuple(J, v_indices);
            },
            py::arg("cache"), py::arg("base_body_or_frame_ind"),
            py::arg("end_effector_body_or_frame_ind"),
            py::arg("expressed_in_body_or_frame_ind"),
            py::arg("in_terms_of_qdot") = false,
            doc.RigidBodyTree.geometricJacobian.doc)
        .def("relativeTransform",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, int base_or_frame_ind,
                int body_or_frame_ind) {
              return tree
                  .relativeTransform(
                      cache, base_or_frame_ind, body_or_frame_ind)
                  .matrix();
            },
            py::arg("cache"), py::arg("base_or_frame_ind"),
            py::arg("body_or_frame_ind"),
            doc.RigidBodyTree.relativeTransform.doc)
        .def("centerOfMassJacobian",
            &RigidBodyTree<double>::centerOfMassJacobian<T>, py::arg("cache"),
            py::arg("model_instance_id_set") =
                RigidBodyTreeConstants::default_model_instance_id_set,
            py::arg("in_terms_of_qdot") = false,
            doc.RigidBodyTree.centerOfMassJacobian.doc)
        .def("centroidalMomentumMatrix",
            &RigidBodyTree<double>::centroidalMomentumMatrix<T>,
            py::arg("cache"),
            py::arg("model_instance_id_set") =
                RigidBodyTreeConstants::default_model_instance_id_set,
            py::arg("in_terms_of_qdot") = false,
            doc.RigidBodyTree.centroidalMomentumMatrix.doc)
        // forwardKinPositionGradient
        .def("geometricJacobianDotTimesV",
            &RigidBodyTree<double>::geometricJacobianDotTimesV<T>,
            py::arg("cache"), py::arg("base_body_or_frame_ind"),
            py::arg("end_effector_body_or_frame_ind"),
            py::arg("expressed_in_body_or_frame_ind"),
            doc.RigidBodyTree.geometricJacobianDotTimesV.doc)
        .def("centerOfMassJacobianDotTimesV",
            &RigidBodyTree<double>::centerOfMassJacobianDotTimesV<T>,
            py::arg("cache"),
            py::arg("model_instance_id_set") =
                RigidBodyTreeConstants::default_model_instance_id_set,
            doc.RigidBodyTree.centerOfMassJacobianDotTimesV.doc)
        .def("centroidalMomentumMatrixDotTimesV",
            &RigidBodyTree<double>::centroidalMomentumMatrixDotTimesV<T>,
            py::arg("cache"),
            py::arg("model_instance_id_set") =
                RigidBodyTreeConstants::default_model_instance_id_set,
            doc.RigidBodyTree.centroidalMomentumMatrixDotTimesV.doc)
        .def("positionConstraints",
            &RigidBodyTree<double>::positionConstraints<T>, py::arg("cache"),
            doc.RigidBodyTree.positionConstraints.doc)
        .def("positionConstraintsJacobian",
            &RigidBodyTree<double>::positionConstraintsJacobian<T>,
            py::arg("cache"), py::arg("in_terms_of_qdot") = true,
            doc.RigidBodyTree.positionConstraintsJacobian.doc)
        .def("positionConstraintsJacDotTimesV",
            &RigidBodyTree<double>::positionConstraintsJacDotTimesV<T>,
            py::arg("cache"),
            doc.RigidBodyTree.positionConstraintsJacDotTimesV.doc)
        // jointLimitConstriants
        .def("relativeTwist", &RigidBodyTree<double>::relativeTwist<T>,
            py::arg("cache"), py::arg("base_or_frame_ind"),
            py::arg("body_or_frame_ind"),
            py::arg("expressed_in_body_or_frame_ind"),
            doc.RigidBodyTree.relativeTwist.doc)
        // worldMomentumMatrix
        // worldMomentumMatrixDotTimesV
        // transformSpatialAcceleration
        .def("frictionTorques",
            [](const RigidBodyTree<double>* self, const VectorX<T>& v) {
              return self->frictionTorques(v);
            },
            doc.RigidBodyTree.frictionTorques.doc)
        .def("inverseDynamics", &RigidBodyTree<double>::inverseDynamics<T>,
            py::arg("cache"), py::arg("external_wrenches"), py::arg("vd"),
            py::arg("include_velocity_terms") = true,
            doc.RigidBodyTree.inverseDynamics.doc)
        // resolveCenterOfPressure
        .def("transformVelocityMappingToQDotMapping",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, const MatrixX<T>& Av) {
              return tree.transformVelocityMappingToQDotMapping(cache, Av);
            },
            doc.RigidBodyTree.transformVelocityMappingToQDotMapping.doc)
        .def("transformQDotMappingToVelocityMapping",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, const MatrixX<T>& Ap) {
              return tree.transformQDotMappingToVelocityMapping(cache, Ap);
            },
            doc.RigidBodyTree.transformQDotMappingToVelocityMapping.doc)
        // relativeQuaternionJacobian
        // relativeRollPitchYawJacobian
        // relativeRollPitchYawJacobianDotTimesV
        // relativeQuaternionJacobianDotTimesV
        // CheckCacheValidity
        .def("doKinematics",
            [](const RigidBodyTree<double>& tree, const VectorX<T>& q) {
              return tree.doKinematics(q);
            },
            doc.RigidBodyTree.doKinematics.doc_1args)
        .def("doKinematics",
            [](const RigidBodyTree<double>& tree, const VectorX<T>& q,
                const VectorX<T>& v) { return tree.doKinematics(q, v); },
            doc.RigidBodyTree.doKinematics.doc_3args)
        // CreateKinematicsCacheWithType
        .def("ComputeMaximumDepthCollisionPoints",
            &RigidBodyTree<double>::ComputeMaximumDepthCollisionPoints<T>,
            py::arg("cache"), py::arg("use_margins") = true,
            py::arg("throw_if_missing_gradient") = true,
            doc.RigidBodyTree.ComputeMaximumDepthCollisionPoints.doc);
    // Type (b) methods:
    tree_cls
        .def("transformPoints",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache,
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& points,
                int from_body_or_frame_ind, int to_body_or_frame_ind) {
              return tree.transformPoints(
                  cache, points, from_body_or_frame_ind, to_body_or_frame_ind);
            },
            doc.RigidBodyTree.transformPoints.doc)
        .def("relativeRollPitchYaw",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, int from_body_or_frame_ind,
                int to_body_or_frame_ind) {
              return tree.relativeRollPitchYaw(
                  cache, from_body_or_frame_ind, to_body_or_frame_ind);
            },
            py::arg("cache"), py::arg("from_body_or_frame_ind"),
            py::arg("to_body_or_frame_ind"),
            doc.RigidBodyTree.relativeRollPitchYaw.doc)
        .def("transformPointsJacobian",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, const Matrix3X<double>& points,
                int from_body_or_frame_ind, int to_body_or_frame_ind,
                bool in_terms_of_qdot) {
              return tree.transformPointsJacobian(cache, points,
                  from_body_or_frame_ind, to_body_or_frame_ind,
                  in_terms_of_qdot);
            },
            py::arg("cache"), py::arg("points"),
            py::arg("from_body_or_frame_ind"), py::arg("to_body_or_frame_ind"),
            py::arg("in_terms_of_qdot"),
            doc.RigidBodyTree.transformPointsJacobian.doc)
        .def("transformPointsJacobianDotTimesV",
            [](const RigidBodyTree<double>& tree,
                const KinematicsCache<T>& cache, const Matrix3X<double>& points,
                int from_body_or_frame_ind, int to_body_or_frame_ind) {
              return tree.transformPointsJacobianDotTimesV(
                  cache, points, from_body_or_frame_ind, to_body_or_frame_ind);
            },
            py::arg("cache"), py::arg("points"),
            py::arg("from_body_or_frame_ind"), py::arg("to_body_or_frame_ind"),
            doc.RigidBodyTree.transformPointsJacobianDotTimesV.doc);
  };
  // Bind for double and AutoDiff.
  type_visit(
      add_rigid_body_tree_typed_methods, type_pack<double, AutoDiffXd>{});

  // This class template does not have documentation.
  py::class_<KinematicsCache<double>>(m, "KinematicsCacheDouble");
  py::class_<KinematicsCache<AutoDiffXd>>(m, "KinematicsCacheAutoDiffXd");

  py::class_<RigidBodyFrame<double>, shared_ptr<RigidBodyFrame<double>>>(
      m, "RigidBodyFrame", doc.RigidBodyFrame.doc)
      .def(py::init<const std::string&, RigidBody<double>*,
               const Eigen::VectorXd&, const Eigen::VectorXd&>(),
          py::arg("name"), py::arg("body"),
          py::arg("xyz") = Eigen::Vector3d::Zero(),
          py::arg("rpy") = Eigen::Vector3d::Zero())
      .def(py::init<const std::string&, RigidBody<double>*,
               const Eigen::Isometry3d&>(),
          py::arg("name"), py::arg("body"), py::arg("transform_to_body"))
      .def("get_name", &RigidBodyFrame<double>::get_name,
          doc.RigidBodyFrame.get_name.doc)
      .def("get_frame_index", &RigidBodyFrame<double>::get_frame_index,
          doc.RigidBodyFrame.get_frame_index.doc)
      .def("get_rigid_body", &RigidBodyFrame<double>::get_rigid_body,
          py_reference,
          // Keep alive: `self` keeps `return` alive.
          py::keep_alive<1, 0>(), doc.RigidBodyFrame.get_rigid_body.doc)
      .def("get_transform_to_body",
          &RigidBodyFrame<double>::get_transform_to_body,
          doc.RigidBodyFrame.get_transform_to_body.doc);

  const char* const doc_AddModelInstanceFromUrdfFile =
      doc.drake.parsers.urdf.AddModelInstanceFromUrdfFile.doc_deprecated_5args;
  m.def("AddModelInstanceFromUrdfFile",
      [](const std::string& urdf_filename,
          const FloatingBaseType floating_base_type,
          shared_ptr<RigidBodyFrame<double>> weld_to_frame,
          RigidBodyTree<double>* tree, bool do_compile) {
        return parsers::urdf::AddModelInstanceFromUrdfFile(
            urdf_filename, floating_base_type, weld_to_frame, do_compile, tree);
      },
      py::arg("urdf_filename"), py::arg("floating_base_type"),
      py::arg("weld_to_frame"), py::arg("tree"), py::arg("do_compile") = true,
      doc_AddModelInstanceFromUrdfFile);

  m.def("AddModelInstanceFromUrdfStringSearchingInRosPackages",
      py::overload_cast<const std::string&, const PackageMap&,
          const std::string&, const FloatingBaseType,
          shared_ptr<RigidBodyFrame<double>>,
          RigidBodyTree<double>*>(
          &parsers::urdf::  // BR
          AddModelInstanceFromUrdfStringSearchingInRosPackages),
      doc.drake.parsers.urdf
          .AddModelInstanceFromUrdfStringSearchingInRosPackages.doc);
  m.def("AddModelInstancesFromSdfFile",
      [](const std::string& sdf_filename,
          const FloatingBaseType floating_base_type,
          std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
          RigidBodyTree<double>* tree, bool do_compile) {
        return sdf::AddModelInstancesFromSdfFile(
            sdf_filename, floating_base_type, weld_to_frame, do_compile, tree);
      },
      py::arg("sdf_filename"), py::arg("floating_base_type"),
      py::arg("weld_to_frame"), py::arg("tree"), py::arg("do_compile") = true,
      doc.drake.parsers.sdf.AddModelInstancesFromSdfFile.doc);
  m.def("AddModelInstancesFromSdfString",
      py::overload_cast<const std::string&, const FloatingBaseType,
          shared_ptr<RigidBodyFrame<double>>, RigidBodyTree<double>*>(
          &sdf::AddModelInstancesFromSdfString),
      doc.drake.parsers.sdf.AddModelInstancesFromSdfString.doc);
  m.def("AddModelInstancesFromSdfStringSearchingInRosPackages",
      py::overload_cast<const std::string&, const PackageMap&,
          const FloatingBaseType, shared_ptr<RigidBodyFrame<double>>,
          RigidBodyTree<double>*>(
          &sdf::  // BR
          AddModelInstancesFromSdfStringSearchingInRosPackages),
      doc.drake.parsers.sdf.AddModelInstancesFromSdfStringSearchingInRosPackages
          .doc);
  m.def("AddFlatTerrainToWorld", &multibody::AddFlatTerrainToWorld,
      py::arg("tree"), py::arg("box_size") = 1000, py::arg("box_depth") = 10,
      doc.drake.multibody.AddFlatTerrainToWorld.doc);

  py::class_<RigidBodyActuator>(
      m, "RigidBodyActuator", doc.RigidBodyActuator.doc)
      .def_readonly(
          "name", &RigidBodyActuator::name_, doc.RigidBodyActuator.name_.doc)
      .def_readonly(
          "body", &RigidBodyActuator::body_, doc.RigidBodyActuator.body_.doc)
      .def_readonly("reduction", &RigidBodyActuator::reduction_,
          doc.RigidBodyActuator.reduction_.doc)
      .def_readonly("effort_limit_min", &RigidBodyActuator::effort_limit_min_,
          doc.RigidBodyActuator.effort_limit_min_.doc)
      .def_readonly("effort_limit_max", &RigidBodyActuator::effort_limit_max_,
          doc.RigidBodyActuator.effort_limit_max_.doc);
}  // NOLINT(readability/fn_size)

}  // namespace pydrake
}  // namespace drake
