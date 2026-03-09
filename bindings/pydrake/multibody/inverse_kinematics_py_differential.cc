#include <memory>
#include <vector>

#include "drake/bindings/generated_docstrings/multibody_inverse_kinematics.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/multibody/inverse_kinematics_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

using Ingredient = multibody::DifferentialInverseKinematicsSystem::Ingredient;
using multibody::DifferentialInverseKinematicsSystem;
using planning::CollisionChecker;
using planning::DofMask;
using planning::JointLimits;
using Recipe = DifferentialInverseKinematicsSystem::Recipe;
using systems::LeafSystem;

void DefineDifferentialIkLegacy(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  constexpr auto& doc =
      pydrake_doc_multibody_inverse_kinematics.drake.multibody;

  py::module::import("pydrake.systems.framework");

  py::enum_<DifferentialInverseKinematicsStatus>(m,
      "DifferentialInverseKinematicsStatus",
      doc.DifferentialInverseKinematicsStatus.doc)
      .value("kSolutionFound",
          DifferentialInverseKinematicsStatus::kSolutionFound,
          doc.DifferentialInverseKinematicsStatus.kSolutionFound.doc)
      .value("kNoSolutionFound",
          DifferentialInverseKinematicsStatus::kNoSolutionFound,
          doc.DifferentialInverseKinematicsStatus.kNoSolutionFound.doc)
      .value("kStuck", DifferentialInverseKinematicsStatus::kStuck,
          doc.DifferentialInverseKinematicsStatus.kStuck.doc);

  {
    using Class = DifferentialInverseKinematicsResult;
    constexpr auto& cls_doc = doc.DifferentialInverseKinematicsResult;
    py::class_<Class> cls(m, "DifferentialInverseKinematicsResult",
        doc.DifferentialInverseKinematicsResult.doc);

    // TODO(m-chaturvedi) Add Pybind11 documentation.
    cls  // BR
        .def(ParamInit<Class>())
        .def_readwrite("joint_velocities", &Class::joint_velocities,
            cls_doc.joint_velocities.doc)
        .def_readwrite("status", &Class::status, cls_doc.status.doc);
  }
  {
    using Class = DifferentialInverseKinematicsParameters;
    constexpr auto& cls_doc = doc.DifferentialInverseKinematicsParameters;

    py::class_<Class> cls(m, "DifferentialInverseKinematicsParameters",
        doc.DifferentialInverseKinematicsParameters.doc);

    cls.def(py::init([](int num_positions, int num_velocities) {
         return Class{num_positions, num_velocities};
       }),
           py::arg("num_positions"), py::arg("num_velocities") = std::nullopt,
           cls_doc.ctor.doc)
        .def("get_time_step", &Class::get_time_step, cls_doc.get_time_step.doc)
        .def("set_time_step", &Class::set_time_step, py::arg("dt"),
            cls_doc.set_time_step.doc)
        .def("get_num_positions", &Class::get_num_positions,
            cls_doc.get_num_positions.doc)
        .def("get_num_velocities", &Class::get_num_velocities,
            cls_doc.get_num_velocities.doc)
        .def("get_nominal_joint_position", &Class::get_nominal_joint_position,
            cls_doc.get_nominal_joint_position.doc)
        .def("set_nominal_joint_position", &Class::set_nominal_joint_position,
            cls_doc.set_nominal_joint_position.doc)
        .def("get_joint_centering_gain", &Class::get_joint_centering_gain,
            cls_doc.get_joint_centering_gain.doc)
        .def("set_joint_centering_gain", &Class::set_joint_centering_gain,
            py::arg("K"), cls_doc.set_joint_centering_gain.doc)
        .def("get_end_effector_velocity_flag",
            &Class::get_end_effector_velocity_flag,
            cls_doc.get_end_effector_velocity_flag.doc)
        .def("set_end_effector_velocity_flag",
            &Class::set_end_effector_velocity_flag, py::arg("flag_E"),
            cls_doc.set_end_effector_velocity_flag.doc)
        .def("get_joint_position_limits", &Class::get_joint_position_limits,
            cls_doc.get_joint_position_limits.doc)
        .def("set_joint_position_limits", &Class::set_joint_position_limits,
            cls_doc.set_joint_position_limits.doc)
        .def("get_joint_velocity_limits", &Class::get_joint_velocity_limits,
            cls_doc.get_joint_velocity_limits.doc)
        .def("set_joint_velocity_limits", &Class::set_joint_velocity_limits,
            cls_doc.set_joint_velocity_limits.doc)
        .def("get_joint_acceleration_limits",
            &Class::get_joint_acceleration_limits,
            cls_doc.get_joint_acceleration_limits.doc)
        .def("set_joint_acceleration_limits",
            &Class::set_joint_acceleration_limits,
            cls_doc.set_joint_acceleration_limits.doc)
        .def("get_maximum_scaling_to_report_stuck",
            &Class::get_maximum_scaling_to_report_stuck,
            cls_doc.get_maximum_scaling_to_report_stuck.doc)
        .def("set_maximum_scaling_to_report_stuck",
            &Class::set_maximum_scaling_to_report_stuck, py::arg("scaling"),
            cls_doc.set_maximum_scaling_to_report_stuck.doc)
        .def("get_end_effector_angular_speed_limit",
            &Class::get_end_effector_angular_speed_limit,
            cls_doc.get_end_effector_angular_speed_limit.doc)
        .def("set_end_effector_angular_speed_limit",
            &Class::set_end_effector_angular_speed_limit, py::arg("speed"),
            cls_doc.set_end_effector_angular_speed_limit.doc)
        .def("get_end_effector_translational_velocity_limits",
            &Class::get_end_effector_translational_velocity_limits,
            cls_doc.get_end_effector_translational_velocity_limits.doc)
        .def("set_end_effector_translational_velocity_limits",
            &Class::set_end_effector_translational_velocity_limits,
            py::arg("lower"), py::arg("upper"),
            cls_doc.set_end_effector_translational_velocity_limits.doc)
        .def("get_mutable_solver_options", &Class::get_mutable_solver_options,
            py_rvp::reference_internal, cls_doc.get_mutable_solver_options.doc);
  }

  m.def(
      "DoDifferentialInverseKinematics",
      [](const Eigen::VectorXd& q_current, const Eigen::VectorXd& v_current,
          const Eigen::VectorXd& V, const Eigen::MatrixXd& J,
          const DifferentialInverseKinematicsParameters& parameters,
          const std::optional<Eigen::Ref<const Eigen::MatrixXd>>& N,
          const std::optional<Eigen::Ref<const Eigen::MatrixXd>>& Nplus) {
        std::optional<Eigen::SparseMatrix<double>> N_sparse{std::nullopt};
        std::optional<Eigen::SparseMatrix<double>> Nplus_sparse{std::nullopt};
        if (N) {
          N_sparse = N->sparseView();
        }
        if (Nplus) {
          Nplus_sparse = Nplus->sparseView();
        }
        return DoDifferentialInverseKinematics(
            q_current, v_current, V, J, parameters, N_sparse, Nplus_sparse);
      },
      py::arg("q_current"), py::arg("v_current"), py::arg("V"), py::arg("J"),
      py::arg("parameters"), py::arg("N") = std::nullopt,
      py::arg("Nplus") = std::nullopt,
      doc.DoDifferentialInverseKinematics
          .doc_7args_q_current_v_current_V_J_parameters_N_Nplus);

  m.def(
      "DoDifferentialInverseKinematics",
      [](const multibody::MultibodyPlant<double>& robot,
          const systems::Context<double>& context,
          const Vector6<double>& V_WE_desired,
          const multibody::Frame<double>& frame_E,
          const DifferentialInverseKinematicsParameters& parameters) {
        return DoDifferentialInverseKinematics(
            robot, context, V_WE_desired, frame_E, parameters);
      },
      py::arg("robot"), py::arg("context"), py::arg("V_WE_desired"),
      py::arg("frame_E"), py::arg("parameters"),
      doc.DoDifferentialInverseKinematics
          .doc_5args_robot_context_V_WE_desired_frame_E_parameters);

  m.def(
      "DoDifferentialInverseKinematics",
      [](const multibody::MultibodyPlant<double>& robot,
          const systems::Context<double>& context,
          const Vector6<double>& V_AE_desired,
          const multibody::Frame<double>& frame_A,
          const multibody::Frame<double>& frame_E,
          const DifferentialInverseKinematicsParameters& parameters) {
        return DoDifferentialInverseKinematics(
            robot, context, V_AE_desired, frame_A, frame_E, parameters);
      },
      py::arg("robot"), py::arg("context"), py::arg("V_AE_desired"),
      py::arg("frame_A"), py::arg("frame_E"), py::arg("parameters"),
      doc.DoDifferentialInverseKinematics
          .doc_6args_robot_context_V_AE_desired_frame_A_frame_E_parameters);

  m.def(
      "DoDifferentialInverseKinematics",
      [](const multibody::MultibodyPlant<double>& robot,
          const systems::Context<double>& context,
          const math::RigidTransform<double>& X_WE_desired,
          const multibody::Frame<double>& frame_E,
          const DifferentialInverseKinematicsParameters& parameters) {
        return DoDifferentialInverseKinematics(
            robot, context, X_WE_desired, frame_E, parameters);
      },
      py::arg("robot"), py::arg("context"), py::arg("X_WE_desired"),
      py::arg("frame_E"), py::arg("parameters"),
      doc.DoDifferentialInverseKinematics
          .doc_5args_robot_context_X_WE_desired_frame_E_parameters);

  m.def(
      "DoDifferentialInverseKinematics",
      [](const multibody::MultibodyPlant<double>& robot,
          const systems::Context<double>& context,
          const math::RigidTransform<double>& X_AE_desired,
          const multibody::Frame<double>& frame_A,
          const multibody::Frame<double>& frame_E,
          const DifferentialInverseKinematicsParameters& parameters) {
        return DoDifferentialInverseKinematics(
            robot, context, X_AE_desired, frame_A, frame_E, parameters);
      },
      py::arg("robot"), py::arg("context"), py::arg("X_AE_desired"),
      py::arg("frame_A"), py::arg("frame_E"), py::arg("parameters"),
      doc.DoDifferentialInverseKinematics
          .doc_6args_robot_context_X_AE_desired_frame_A_frame_E_parameters);

  {
    using Class = DifferentialInverseKinematicsIntegrator;
    constexpr auto& cls_doc = doc.DifferentialInverseKinematicsIntegrator;
    py::class_<Class, LeafSystem<double>>(
        m, "DifferentialInverseKinematicsIntegrator", cls_doc.doc)
        .def(py::init<const multibody::MultibodyPlant<double>&,
                 const multibody::Frame<double>&,
                 const multibody::Frame<double>&, double,
                 const DifferentialInverseKinematicsParameters&,
                 const systems::Context<double>*, bool>(),
            // Keep alive, reference: `self` keeps `robot` alive.
            py::keep_alive<1, 2>(), py::arg("robot"), py::arg("frame_A"),
            py::arg("frame_E"), py::arg("time_step"), py::arg("parameters"),
            py::arg("robot_context") = nullptr,
            py::arg("log_only_when_result_state_changes") = true,
            cls_doc.ctor.doc_7args)
        .def(py::init<const multibody::MultibodyPlant<double>&,
                 const multibody::Frame<double>&, double,
                 const DifferentialInverseKinematicsParameters&,
                 const systems::Context<double>*, bool>(),
            // Keep alive, reference: `self` keeps `robot` alive.
            py::keep_alive<1, 2>(), py::arg("robot"), py::arg("frame_E"),
            py::arg("time_step"), py::arg("parameters"),
            py::arg("robot_context") = nullptr,
            py::arg("log_only_when_result_state_changes") = true,
            cls_doc.ctor.doc_6args)
        .def("SetPositions", &Class::SetPositions, py::arg("context"),
            py::arg("positions"), cls_doc.SetPositions.doc)
        .def("ForwardKinematics", &Class::ForwardKinematics, py::arg("context"),
            cls_doc.ForwardKinematics.doc)
        .def("get_parameters", &Class::get_parameters,
            py_rvp::reference_internal, cls_doc.get_parameters.doc)
        .def("get_mutable_parameters", &Class::get_mutable_parameters,
            py_rvp::reference_internal, cls_doc.get_mutable_parameters.doc);
  }
}

// The pybind class for the DifferentialInverseKinematicsSystem.
using PyClassDifferentialInverseKinematicsSystem =
    py::class_<DifferentialInverseKinematicsSystem, LeafSystem<double>>;

// The pybind class for DifferentialInverseKinematicsSystem's ingredients.
template <typename Derived>
using PyClassIngredient = py::class_<Derived, Ingredient>;

// Bind the common Ingredient API for both Ingredient and its Config struct.
// The bound class gets returned so non-common APIs can be subsequently bound.
//
// Note: most Ingredients construct with only a Config parameter. For
// Ingredients with a different constructor signature, the second parameter
// allows that binding to opt-out of the common constructor. Locally, it can add
// the unique constructor on the returned binding after the fact.
template <typename Derived, bool bind_config_ctor = true, typename ClassDoc>
PyClassIngredient<Derived> BindIngredient(const char* class_name,
    const ClassDoc& cls_doc,
    PyClassDifferentialInverseKinematicsSystem* diff_ik_cls) {
  using Config = typename Derived::Config;

  PyClassIngredient<Derived> cls(*diff_ik_cls, class_name, cls_doc.doc);

  py::class_<Config> config_cls(cls, "Config", cls_doc.Config.doc);
  config_cls.def(ParamInit<Config>());
  DefAttributesUsingSerialize(&config_cls, cls_doc.Config);
  DefReprUsingSerialize(&config_cls);
  DefCopyAndDeepCopy(&config_cls);

  cls  // BR
      .def("GetConfig", &Derived::GetConfig, py_rvp::reference_internal,
          cls_doc.GetConfig.doc)
      .def("SetConfig", &Derived::SetConfig, py::arg("config"),
          cls_doc.SetConfig.doc);
  if constexpr (bind_config_ctor) {
    cls.def(py::init<const Config&>(), py::arg("config"), cls_doc.ctor.doc);
  }

  return cls;
}

void DefineDifferentialIkSystem(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc =
      pydrake_doc_multibody_inverse_kinematics.drake.multibody;

  constexpr auto& cls_doc = doc.DifferentialInverseKinematicsSystem;

  // Instantiate the class so we can instantiate the nested types.
  PyClassDifferentialInverseKinematicsSystem cls(
      m, "DifferentialInverseKinematicsSystem", cls_doc.doc);

  {
    constexpr auto nested_cls_doc = cls_doc.Ingredient;
    py::class_<Ingredient>(cls, "Ingredient", nested_cls_doc.doc);
    // We're explicitly not binding `AddToProgram` because we expect only the
    // C++ DifferentialInverseKinematicsSystem would call it.
  }

  {
    using NestedClass = Recipe;
    constexpr auto nested_cls_doc = cls_doc.Recipe;
    py::class_<NestedClass>(cls, "Recipe", nested_cls_doc.doc)
        .def(py::init<>(), nested_cls_doc.ctor.doc)
        .def(
            "AddIngredient",
            [](NestedClass* self, py::object ingredient) {
              self->AddIngredient(
                  make_shared_ptr_from_py_object<Ingredient>(ingredient));
            },
            py::arg("ingredient"), nested_cls_doc.AddIngredient.doc)
        .def("num_ingredients", &NestedClass::num_ingredients,
            nested_cls_doc.num_ingredients.doc)
        .def("ingredient", &NestedClass::ingredient, py::arg("i"),
            py_rvp::reference_internal, nested_cls_doc.ingredient.doc);
    // We're explicitly not binding `AddToProgram` because we expect only the
    // C++ DifferentialInverseKinematicsSystem would call it.
  }

  // We're explicitly not binding the nested `CallbackDetails` because it is
  // part of the `AddToProgram` API that we expect only the C++
  // DifferentialInverseKinematicsSystem would call.

  using Class = DifferentialInverseKinematicsSystem;
  cls  // BR
      .def(py::init([](py::object recipe, std::string_view task_frame,
                        py::object collision_checker, const DofMask& active_dof,
                        double time_step, double K_VX,
                        const SpatialVelocity<double>& Vd_TG_limit) {
        return std::make_unique<Class>(
            make_shared_ptr_from_py_object<Recipe>(recipe), task_frame,
            make_shared_ptr_from_py_object<CollisionChecker>(collision_checker),
            active_dof, time_step, K_VX, Vd_TG_limit);
      }),
          py::arg("recipe"), py::arg("task_frame"),
          py::arg("collision_checker"), py::arg("active_dof"),
          py::arg("time_step"), py::arg("K_VX"), py::arg("Vd_TG_limit"),
          cls_doc.ctor.doc)
      .def("recipe", &Class::recipe, py_rvp::reference_internal,
          cls_doc.recipe.doc)
      .def("task_frame", &Class::task_frame, py_rvp::reference_internal,
          cls_doc.task_frame.doc)
      .def(
          "plant", &Class::plant, py_rvp::reference_internal, cls_doc.plant.doc)
      .def("collision_checker", &Class::collision_checker,
          py_rvp::reference_internal, cls_doc.collision_checker.doc)
      .def("active_dof", &Class::active_dof, py_rvp::reference_internal,
          cls_doc.active_dof.doc)
      .def("time_step", &Class::time_step, cls_doc.time_step.doc)
      .def("K_VX", &Class::K_VX, cls_doc.K_VX.doc)
      .def("Vd_TG_limit", &Class::Vd_TG_limit, py_rvp::reference_internal,
          cls_doc.Vd_TG_limit.doc)
      .def("get_input_port_position", &Class::get_input_port_position,
          py_rvp::reference_internal, cls_doc.get_input_port_position.doc)
      .def("get_input_port_nominal_posture",
          &Class::get_input_port_nominal_posture, py_rvp::reference_internal,
          cls_doc.get_input_port_nominal_posture.doc)
      .def("get_input_port_desired_cartesian_poses",
          &Class::get_input_port_desired_cartesian_poses,
          py_rvp::reference_internal,
          cls_doc.get_input_port_desired_cartesian_poses.doc)
      .def("get_input_port_desired_cartesian_velocities",
          &Class::get_input_port_desired_cartesian_velocities,
          py_rvp::reference_internal,
          cls_doc.get_input_port_desired_cartesian_velocities.doc)
      .def("get_output_port_commanded_velocity",
          &Class::get_output_port_commanded_velocity,
          py_rvp::reference_internal,
          cls_doc.get_output_port_commanded_velocity.doc);

  BindIngredient<Class::LeastSquaresCost>(
      "LeastSquaresCost", cls_doc.LeastSquaresCost, &cls);

  BindIngredient<Class::JointCenteringCost>(
      "JointCenteringCost", cls_doc.JointCenteringCost, &cls);

  BindIngredient<Class::CartesianPositionLimitConstraint>(
      "CartesianPositionLimitConstraint",
      cls_doc.CartesianPositionLimitConstraint, &cls);

  BindIngredient<Class::CartesianVelocityLimitConstraint>(
      "CartesianVelocityLimitConstraint",
      cls_doc.CartesianVelocityLimitConstraint, &cls);

  {
    using NestedClass = Class::CollisionConstraint;
    constexpr auto& nested_cls_doc = cls_doc.CollisionConstraint;
    PyClassIngredient<NestedClass> nested_cls = BindIngredient<NestedClass>(
        "CollisionConstraint", nested_cls_doc, &cls);
    nested_cls  // BR
        .def("SetSelectDataForCollisionConstraintFunction",
            &NestedClass::SetSelectDataForCollisionConstraintFunction,
            py::arg("select_data_for_collision_constraint"),
            nested_cls_doc.SetSelectDataForCollisionConstraintFunction.doc);
  }

  {
    using NestedClass = Class::JointVelocityLimitConstraint;
    using Config = NestedClass::Config;
    constexpr auto& nested_cls_doc = cls_doc.JointVelocityLimitConstraint;
    PyClassIngredient<NestedClass> nested_cls =
        BindIngredient<NestedClass, /* bind_config_ctor= */ false>(
            "JointVelocityLimitConstraint", nested_cls_doc, &cls);
    nested_cls  // BR
        .def(py::init<const Config&, const JointLimits&>(), py::arg("config"),
            py::arg("joint_limits"), cls_doc.ctor.doc)
        .def("GetJointLimits", &NestedClass::GetJointLimits,
            py_rvp::reference_internal, nested_cls_doc.GetJointLimits.doc)
        .def("SetJointLimits", &NestedClass::SetJointLimits,
            py::arg("joint_limits"), nested_cls_doc.SetJointLimits.doc);
  }
}

void DefineDifferentialIkController(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc =
      pydrake_doc_multibody_inverse_kinematics.drake.multibody;

  using Class = DifferentialInverseKinematicsController;
  constexpr auto& cls_doc = doc.DifferentialInverseKinematicsController;
  py::class_<Class, systems::Diagram<double>>(
      m, "DifferentialInverseKinematicsController")
      .def(py::init([](py::object differential_inverse_kinematics,
                        const std::vector<int>& planar_rotation_dof_indices) {
        return std::make_unique<Class>(
            make_shared_ptr_from_py_object<DifferentialInverseKinematicsSystem>(
                differential_inverse_kinematics),
            planar_rotation_dof_indices);
      }),
          py::arg("differential_inverse_kinematics"),
          py::arg("planar_rotation_dof_indices"), cls_doc.ctor.doc)
      .def("set_initial_position", &Class::set_initial_position,
          py::arg("context"), py::arg("value"),
          cls_doc.set_initial_position.doc)
      .def("differential_inverse_kinematics",
          &Class::differential_inverse_kinematics, py_rvp::reference_internal,
          cls_doc.differential_inverse_kinematics.doc)
      .def("get_mutable_differential_inverse_kinematics",
          &Class::get_mutable_differential_inverse_kinematics,
          py_rvp::reference_internal,
          cls_doc.get_mutable_differential_inverse_kinematics.doc);
}

}  // namespace

void DefineIkDifferential(py::module m) {
  DefineDifferentialIkLegacy(m);
  DefineDifferentialIkSystem(m);
  DefineDifferentialIkController(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
