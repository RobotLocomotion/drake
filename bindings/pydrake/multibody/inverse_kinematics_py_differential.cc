#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
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

using planning::CollisionChecker;
using planning::DofMask;
using planning::JointLimits;

void DefineIkDifferentialLegacy(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  using drake::systems::LeafSystem;

  constexpr auto& doc = pydrake_doc.drake.multibody;

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

// The pybind class for the DiffIkSystem.
using PyDiffIkSystemClass =
    py::class_<multibody::DifferentialInverseKinematicsSystem,
        systems::LeafSystem<double>>;

// The pybind class for ingredients.
template <typename Derived>
using PyIngredientClass = py::class_<Derived,
    multibody::DifferentialInverseKinematicsSystem::Ingredient>;

// Functor for adding a constructor to an Ingredient.
template <typename Derived>
using ConstructorAdder =
    std::function<void(PyIngredientClass<Derived>*, const char*)>;

// The default constructor adder for ingredients; constructs on a Config
// instance.
template <typename Derived>
void AddConfigConstructor(PyIngredientClass<Derived>* cls, const char* doc) {
  using Config = typename Derived::Config;
  cls->def(py::init<const Config&>(), py::arg("config"), doc);
}

// Bind the common Ingredient API for both Ingredient and its Config struct.
// The bound class gets returned so non-common APIs can be subsequently bound.
template <typename IngredientType, typename ClassDoc>
PyIngredientClass<IngredientType> BindIngredient(const char* class_name,
    const ClassDoc& class_doc, PyDiffIkSystemClass* diff_ik_cls,
    ConstructorAdder<IngredientType> constructor_adder =
        AddConfigConstructor<IngredientType>) {
  using Config = typename IngredientType::Config;

  PyIngredientClass<IngredientType> cls(
      *diff_ik_cls, class_name, class_doc.doc);

  py::class_<Config> config_cls(cls, "Config", class_doc.Config.doc);
  config_cls.def(ParamInit<Config>());
  DefAttributesUsingSerialize(&config_cls, class_doc.Config);
  DefReprUsingSerialize(&config_cls);
  DefCopyAndDeepCopy(&config_cls);

  cls  // BR
      .def("GetConfig", &IngredientType::GetConfig, py_rvp::reference_internal,
          class_doc.GetConfig.doc)
      .def("SetConfig", &IngredientType::SetConfig, py::arg("config"),
          class_doc.SetConfig.doc);
  constructor_adder(&cls, class_doc.ctor.doc);

  return cls;
}

void DefinePlanningDifferentialIkSystem(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  using DiffIkSysClass = DifferentialInverseKinematicsSystem;
  constexpr auto& diff_ik_sys_cls_doc = doc.DifferentialInverseKinematicsSystem;

  // Instantiate the class so we can instantiate the nested functions.
  PyDiffIkSystemClass diff_ik_sys_cls(
      m, "DifferentialInverseKinematicsSystem", diff_ik_sys_cls_doc.doc);

  py::class_<DiffIkSysClass::Ingredient>(
      diff_ik_sys_cls, "Ingredient", diff_ik_sys_cls_doc.Recipe.doc);

  py::class_<DiffIkSysClass::Recipe>(
      diff_ik_sys_cls, "Recipe", diff_ik_sys_cls_doc.Recipe.doc)
      .def(py::init<>(), diff_ik_sys_cls_doc.Recipe.ctor.doc)
      .def(
          "AddIngredient",
          [](DiffIkSysClass::Recipe* self, py::object ingredient) {
            self->AddIngredient(
                make_shared_ptr_from_py_object<DiffIkSysClass::Ingredient>(
                    ingredient));
          },
          py::arg("ingredient"), diff_ik_sys_cls_doc.Recipe.AddIngredient.doc);

  diff_ik_sys_cls  // BR
      .def(py::init([](py::object recipe, std::string_view task_frame,
                        py::object checker, const DofMask& active_dof,
                        double time_step, double K_VX,
                        const multibody::SpatialVelocity<double>& Vd_TG_limit) {
        return std::make_unique<DiffIkSysClass>(
            make_shared_ptr_from_py_object<DiffIkSysClass::Recipe>(recipe),
            task_frame,
            make_shared_ptr_from_py_object<CollisionChecker>(checker),
            active_dof, time_step, K_VX, Vd_TG_limit);
      }),
          py::arg("recipe"), py::arg("task_frame"),
          py::arg("collision_checker"), py::arg("active_dof"),
          py::arg("time_step"), py::arg("K_VX"), py::arg("Vd_TG_limit"),
          diff_ik_sys_cls_doc.ctor.doc)
      .def("plant", &DiffIkSysClass::plant, py_rvp::reference_internal,
          diff_ik_sys_cls_doc.plant.doc)
      .def("collision_checker", &DiffIkSysClass::collision_checker,
          py_rvp::reference_internal, diff_ik_sys_cls_doc.collision_checker.doc)
      .def("active_dof", &DiffIkSysClass::active_dof,
          py_rvp::reference_internal, diff_ik_sys_cls_doc.active_dof.doc)
      .def("time_step", &DiffIkSysClass::time_step,
          diff_ik_sys_cls_doc.time_step.doc)
      .def("task_frame", &DiffIkSysClass::task_frame,
          py_rvp::reference_internal, diff_ik_sys_cls_doc.task_frame.doc)
      .def("get_input_port_position", &DiffIkSysClass::get_input_port_position,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_input_port_position.doc)
      .def("get_input_port_nominal_posture",
          &DiffIkSysClass::get_input_port_nominal_posture,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_input_port_nominal_posture.doc)
      .def("get_input_port_desired_cartesian_poses",
          &DiffIkSysClass::get_input_port_desired_cartesian_poses,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_input_port_desired_cartesian_poses.doc)
      .def("get_input_port_desired_cartesian_velocities",
          &DiffIkSysClass::get_input_port_desired_cartesian_velocities,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_input_port_desired_cartesian_velocities.doc)
      .def("get_output_port_commanded_velocity",
          &DiffIkSysClass::get_output_port_commanded_velocity,
          py_rvp::reference_internal,
          diff_ik_sys_cls_doc.get_output_port_commanded_velocity.doc);

  BindIngredient<DiffIkSysClass::LeastSquaresCost>("LeastSquaresCost",
      diff_ik_sys_cls_doc.LeastSquaresCost, &diff_ik_sys_cls);

  BindIngredient<DiffIkSysClass::JointCenteringCost>("JointCenteringCost",
      diff_ik_sys_cls_doc.JointCenteringCost, &diff_ik_sys_cls);

  BindIngredient<DiffIkSysClass::CartesianPositionLimitConstraint>(
      "CartesianPositionLimitConstraint",
      diff_ik_sys_cls_doc.CartesianPositionLimitConstraint, &diff_ik_sys_cls);

  BindIngredient<DiffIkSysClass::CartesianVelocityLimitConstraint>(
      "CartesianVelocityLimitConstraint",
      diff_ik_sys_cls_doc.CartesianVelocityLimitConstraint, &diff_ik_sys_cls);

  PyIngredientClass<DiffIkSysClass::CollisionConstraint> collision_cls =
      BindIngredient<DiffIkSysClass::CollisionConstraint>("CollisionConstraint",
          diff_ik_sys_cls_doc.CollisionConstraint, &diff_ik_sys_cls);
  collision_cls  // BR
      .def("SetSelectDataForCollisionConstraintFunction",
          &DiffIkSysClass::CollisionConstraint::
              SetSelectDataForCollisionConstraintFunction,
          py::arg("select_data_for_collision_constraint"),
          diff_ik_sys_cls_doc.CollisionConstraint
              .SetSelectDataForCollisionConstraintFunction.doc);

  auto add_special_ctor =
      [](PyIngredientClass<DiffIkSysClass::JointVelocityLimitConstraint>* cls,
          const char* ctor_doc) {
        using Config = DiffIkSysClass::JointVelocityLimitConstraint::Config;
        cls->def(py::init<const Config&, const JointLimits&>(),
            py::arg("config"), py::arg("joint_limits"), ctor_doc);
      };
  PyIngredientClass<DiffIkSysClass::JointVelocityLimitConstraint> jv_cls =
      BindIngredient<DiffIkSysClass::JointVelocityLimitConstraint>(
          "JointVelocityLimitConstraint",
          diff_ik_sys_cls_doc.JointVelocityLimitConstraint, &diff_ik_sys_cls,
          /* constructor_adder = */ add_special_ctor);
  jv_cls  // BR
      .def("GetJointLimits",
          &DiffIkSysClass::JointVelocityLimitConstraint::GetJointLimits,
          diff_ik_sys_cls_doc.JointVelocityLimitConstraint.GetJointLimits.doc)
      .def("SetJointLimits",
          &DiffIkSysClass::JointVelocityLimitConstraint::SetJointLimits,
          py::arg("joint_limits"),
          diff_ik_sys_cls_doc.JointVelocityLimitConstraint.SetJointLimits.doc);
}

void DefinePlanningDifferentialIkController(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  using Class = DifferentialInverseKinematicsController;
  constexpr auto& cls_doc = doc.DifferentialInverseKinematicsController;
  py::class_<Class, systems::Diagram<double>>(
      m, "DifferentialInverseKinematicsController")
      .def(py::init([](py::object diff_ik_sys,
                        const std::vector<int>& indices) {
        return std::make_unique<Class>(
            make_shared_ptr_from_py_object<DifferentialInverseKinematicsSystem>(
                diff_ik_sys),
            indices);
      }),
          py::arg("differential_inverse_kinematics"),
          py::arg("planar_rotation_dof_indices"), cls_doc.ctor.doc)
      .def("set_initial_position", &Class::set_initial_position,
          py::arg("context"), py::arg("value"),
          cls_doc.set_initial_position.doc)
      .def("SetDefaultState", &Class::SetDefaultState, py::arg("context"),
          py::arg("state"), cls_doc.SetDefaultState.doc)
      .def("SetRandomState", &Class::SetRandomState, py::arg("context"),
          py::arg("state"), py::arg("generator"), cls_doc.SetRandomState.doc)
      // This method has no documentation.
      .def("differential_inverse_kinematics",
          &Class::differential_inverse_kinematics, py_rvp::reference_internal)
      // This method has no documentation.
      .def("get_mutable_differential_inverse_kinematics",
          &Class::get_mutable_differential_inverse_kinematics,
          py_rvp::reference_internal);
}

}  // namespace

void DefineIkDifferential(py::module m) {
  DefineIkDifferentialLegacy(m);
  DefinePlanningDifferentialIkSystem(m);
  DefinePlanningDifferentialIkController(m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
