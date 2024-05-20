#pragma once

#include <memory>
#include <optional>
#include <type_traits>
#include <utility>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {

namespace internal {
// Friend helper class to access SapDriver private internals for testing.
class SapDriverTest {
 public:
  template <typename T>
  static const ContactProblemCache<T>& EvalContactProblemCache(
      const SapDriver<T>& driver, const systems::Context<T>& context) {
    return driver.EvalContactProblemCache(context);
  }
};
}  // namespace internal

namespace test {

// Options for RobotModel.
struct RobotModelConfig {
  // If true, loads a model with collision geometry. Otherwise all proximity
  // geometry is removed.
  bool with_contact_geometry{true};

  // Specifies the discrete approximation of contact.
  DiscreteContactApproximation contact_approximation{
      DiscreteContactApproximation::kSimilar};

  // If true, the state is initialized to
  // RobotModel::RobotStateWithOneContactStiction().
  // If false, the state is initialized to RobotModel::RobotZeroState().
  bool state_in_contact{true};

  // Specifies the geometric modeling of contact. Even if `with_contact_geometry
  // = false`, MultibodyPlant calls specific proximity queries depending on the
  // contact model. To unit test these different code paths, this config
  // provides both `contact_model` and `with_contact_geometry` separately.
  ContactModel contact_model{ContactModel::kHydroelasticWithFallback};
};

// Helper class to create an interesting SapContactProblem with problem data
// a function of the parameters of differentiation, in this case the initial
// state. This allows to exercise the entire numeric pipeline with gradients
// propagating through complex terms such as the mass matrix, contact Jacobians
// and even contact data.
//
// In particular, we load the model of an IIWA7 arm and a free floating plate.
// Having a robot and a free floating body helps to stress test the sparsity
// treatment within the solver.
template <typename T>
class RobotModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotModel);

  // Creates an empty model.
  RobotModel() = default;

  // This constructor is provided to make a RobotModel<double>. Use the scalar
  // conversion methods provided by this class to obtain models on other scalar
  // types.
  explicit RobotModel(const RobotModelConfig& config,
                      bool add_visualization = false)
    requires std::is_same_v<T, double>
  {  // NOLINT(whitespace/braces)
    systems::DiagramBuilder<double> builder;
    auto items = AddMultibodyPlantSceneGraph(&builder, kTimeStep_);
    plant_ = &items.plant;

    Parser parser(plant_);
    robot_model_instance_ = parser.AddModelsFromUrl(
        "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf")
                                [0];
    parser.AddModelsFromUrl("package://drake_models/dishes/plate_8in.sdf");

    // Weld the robot's base to the world.
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetBodyByName("iiwa_link_0").body_frame(),
                       math::RigidTransformd::Identity());

    // Ground geometry.
    geometry::ProximityProperties proximity_properties;
    geometry::AddContactMaterial(kHcDissipation_, kStiffness_,
                                 CoulombFriction<double>(kMu_, kMu_),
                                 &proximity_properties);
    proximity_properties.AddProperty(geometry::internal::kMaterialGroup,
                                     geometry::internal::kRelaxationTime,
                                     kRelaxationTime_ / 2);
    plant_->RegisterCollisionGeometry(
        plant_->world_body(),
        math::RigidTransformd(Eigen::Vector3d(0.0, 0.0, -0.05)),
        geometry::Box(2.0, 2.0, 0.1), "ground_collision", proximity_properties);

    // Add simple contact geometry at the end effector.
    plant_->RegisterCollisionGeometry(
        plant_->GetBodyByName("iiwa_link_7"),
        math::RigidTransformd(Eigen::Vector3d(0.0, 0.0, 0.07)),
        geometry::Sphere(0.05), "iiwa_link_7_collision", proximity_properties);

    plant_->set_contact_model(config.contact_model);
    plant_->set_discrete_contact_approximation(config.contact_approximation);
    plant_->Finalize();

    // Remove proximity roles if geometry is not requested.
    if (!config.with_contact_geometry) {
      const auto& inspector = items.scene_graph.model_inspector();
      for (const auto id :
           inspector.GetAllGeometryIds(geometry::Role::kProximity)) {
        items.scene_graph.RemoveRole(*plant_->get_source_id(), id,
                                     geometry::Role::kProximity);
      }
    }

    // Make and add a manager so that we have access to it and its driver.
    if (plant_->get_discrete_contact_solver() == DiscreteContactSolver::kSap) {
      auto owned_contact_manager = std::make_unique<
          multibody::internal::CompliantContactManager<double>>();
      manager_ = owned_contact_manager.get();
      plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));
      driver_ = &multibody::internal::CompliantContactManagerTester::sap_driver(
          *manager_);
    }

    // We add a visualizer for visual inspection of the model in this test.
    if (add_visualization) {
      visualization::AddDefaultVisualization(&builder);
    }

    diagram_ = builder.Build();

    // Create context.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, context_.get());
    SetPlateInitialState();
    if (config.state_in_contact) {
      SetRobotState(RobotStateWithOneContactStiction());
    } else {
      SetRobotState(RobotZeroState());
    }

    // Fix input ports.
    const VectorX<double> tau =
        VectorX<double>::Zero(plant_->num_actuated_dofs());
    plant_->get_actuation_input_port().FixValue(plant_context_, tau);
  }

  const MultibodyPlant<T>& plant() const { return *plant_; }
  const systems::Diagram<T>& diagram() const { return *diagram_; }
  const systems::Context<T>& context() const { return *context_; }

  int num_velocities() const { return plant_->num_velocities(); }

  template <typename U>
  std::unique_ptr<RobotModel<U>> ToScalarType() const {
    auto converted_model = std::make_unique<RobotModel<U>>();

    // Scalar-convert the model.
    converted_model->diagram_ = dynamic_pointer_cast<systems::Diagram<U>>(
        diagram_->template ToScalarType<U>());
    converted_model->plant_ = const_cast<MultibodyPlant<U>*>(
        &converted_model->diagram_
             ->template GetDowncastSubsystemByName<MultibodyPlant<U>>(
                 plant_->get_name()));

    // Make and add a manager so that we have access to it and its driver.
    if (plant_->get_discrete_contact_solver() == DiscreteContactSolver::kSap &&
        !std::is_same_v<U, symbolic::Expression>) {
      auto owned_contact_manager_ad =
          std::make_unique<multibody::internal::CompliantContactManager<U>>();
      converted_model->manager_ = owned_contact_manager_ad.get();
      converted_model->plant_->SetDiscreteUpdateManager(
          std::move(owned_contact_manager_ad));
      converted_model->driver_ =
          &multibody::internal::CompliantContactManagerTester::sap_driver(
              *converted_model->manager_);
    }

    // Create context.
    converted_model->context_ =
        converted_model->diagram_->CreateDefaultContext();
    converted_model->context_->SetTimeStateAndParametersFrom(*context_);
    converted_model->plant_context_ =
        &converted_model->diagram_->GetMutableSubsystemContext(
            *converted_model->plant_, converted_model->context_.get());

    // Fix input ports.
    const VectorX<U> tau_ad = VectorX<U>::Zero(plant_->num_actuated_dofs());
    converted_model->plant_->get_actuation_input_port().FixValue(
        converted_model->plant_context_, tau_ad);

    return converted_model;
  }

  std::unique_ptr<RobotModel<AutoDiffXd>> ToAutoDiffXd() const {
    return ToScalarType<AutoDiffXd>();
  }

  void ForcedPublish() { diagram_->ForcedPublish(*context_); }

  // Helper to set the full state of the model, including robot and plate.
  void SetState(const VectorX<T>& x) {
    plant_->SetPositionsAndVelocities(plant_context_, x);
  }

  // Helper to set the state of the robot only.
  void SetRobotState(const VectorX<T>& x) {
    plant_->SetPositionsAndVelocities(plant_context_, robot_model_instance_, x);
  }

  // Returns a vector with the full state of the model.
  VectorX<T> GetState() const {
    return plant_->GetPositionsAndVelocities(*plant_context_);
  }

  // Helper that uses the underlying SapDriver to evaluate the SapContactProblem
  // at x0. Not available when T = symbolic::Expression.
  const multibody::contact_solvers::internal::SapContactProblem<T>&
  EvalContactProblem(const VectorX<T>& x0)
    requires(!std::is_same_v<T, symbolic::Expression>)
  {  // NOLINT(whitespace/braces)
    DRAKE_DEMAND(plant_->get_discrete_contact_solver() ==
                 DiscreteContactSolver::kSap);
    SetState(x0);
    const auto& problem_cache =
        drake::multibody::internal::SapDriverTest::EvalContactProblemCache(
            *driver_, *plant_context_);
    // There are no locked dofs. Sanity check.
    DRAKE_DEMAND(problem_cache.sap_problem_locked == nullptr);
    return *problem_cache.sap_problem;
  }

  // Makes a state in which the robot's end effector touches the ground.
  // Velocities are zero.
  static Eigen::VectorXd RobotStateWithOneContactStiction() {
    return (VectorX<double>(14) << 0, 1.17, 0, -1.33, 0, 0.58, 0,  // q
            0, 0, 0, 0, 0, 0, 0                                    // v
            )
        .finished();
  }

  // Makes a state in which the robot's end effector touches the ground.
  // Velocities are non-zero.
  static Eigen::VectorXd RobotStateWithOneOneContactSliding() {
    return (VectorX<double>(14) << 0, 1.17, 0, -1.33, 0, 0.58, 0,  // q
            0, -0.1, 0, -0.2, 0, 0, 0                              // v
            )
        .finished();
  }

  // Returns the robot's "zero state".
  static Eigen::VectorXd RobotZeroState() { return VectorX<double>::Zero(14); }

 private:
  // Friendship to give ToScalarType() access to private members.
  template <typename U>
  friend class RobotModel;

  // We set the initial state of the plate to be away from the robot and away
  // from the ground so that there  is no contact. This is to stress test the
  // sparsity treatment withing the SAP solver, which removes non-participating
  // DOFs from the problem, and solves a reduced problem instead.
  void SetPlateInitialState() {
    plant_->SetFreeBodyPose(plant_context_, plant_->GetBodyByName("plate_8in"),
                            math::RigidTransform<T>{Vector3<T>(0.5, 0.5, 0.5)});
  }

  std::unique_ptr<systems::Diagram<T>> diagram_;
  MultibodyPlant<T>* plant_{nullptr};
  std::unique_ptr<systems::Context<T>> context_;
  systems::Context<T>* plant_context_{nullptr};
  // N.B. manager_ and driver_ will be available for testing only when using the
  // SAP solver for T != symbolic::Expression.
  multibody::internal::CompliantContactManager<T>* manager_{nullptr};
  const multibody::internal::SapDriver<T>* driver_{nullptr};
  ModelInstanceIndex robot_model_instance_;

  // Parameters of the problem.
  const double kTimeStep_{0.001};      // Discrete time step of the plant.
  const double kStiffness_{1.0e4};     // In N/m.
  const double kHcDissipation_{0.2};   // In s/m.
  const double kMu_{0.5};              // Coefficient of friction.
  const double kRelaxationTime_{0.1};  // In s.
};

// Function that can be used to manually inspect the model used for testing. To
// see the model, run the test as a command-line executable, instead of as a
// bazel test.
void VisualizeRobotModel();

}  // namespace test
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::test::RobotModel)
