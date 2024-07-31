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
namespace test {

// Options for RobotModel.
struct RobotModelConfig {
  enum class ContactConfig {
    // The model is loaded with no geometry, in an arbitrary state.
    // In particular, the state from RobotModel::RobotZeroState().
    kNoGeometry,
    // Full model with geometry, in a state with no contact.
    // In particular, the state from RobotModel::RobotZeroState().
    kNoContactState,
    // Full model with geometry, in a state with contact.
    // In particular, the state from
    // RobotModel::RobotStateWithOneContactStiction().
    kInContactState,
  };

  // Model specifics related to the geometry and contact state.
  ContactConfig contact_configuration{ContactConfig::kInContactState};

  // Specifies the discrete approximation of contact.
  DiscreteContactApproximation contact_approximation{
      DiscreteContactApproximation::kSimilar};

  // Specifies the geometric modeling of contact.
  ContactModel contact_model{ContactModel::kHydroelasticWithFallback};
};

// Provides a string description of a robot model configuration.
// It can be used to provide the suffix for gtest parameters.
std::ostream& operator<<(std::ostream& out, const RobotModelConfig& c);

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
    requires std::is_same_v<T, double>;

  const MultibodyPlant<T>& plant() const { return *plant_; }
  const systems::Diagram<T>& diagram() const { return *diagram_; }
  const systems::Context<T>& context() const { return *context_; }

  int num_velocities() const { return plant_->num_velocities(); }

  template <typename U>
  std::unique_ptr<RobotModel<U>> ToScalarType() const;

  std::unique_ptr<RobotModel<AutoDiffXd>> ToAutoDiffXd() const;

  // Helper to invoke ForcedPublish() on the underlying system at the current
  // state stored by this model.
  void ForcedPublish();

  // Helper to set the full state of the model, including robot and plate.
  void SetState(const VectorX<T>& x);

  // Helper to set the state of the robot only.
  void SetRobotState(const VectorX<T>& x);

  // Returns a vector with the full state of the model.
  VectorX<T> GetState() const;

  // Helper that uses the underlying SapDriver to evaluate the SapContactProblem
  // at x0. Not available when T = symbolic::Expression.
  const multibody::contact_solvers::internal::SapContactProblem<T>&
  EvalContactProblem(const VectorX<T>& x0)
    requires(!std::is_same_v<T, symbolic::Expression>);

  // Makes a state in which the robot's end effector touches the ground.
  // Velocities are zero.
  static Eigen::VectorXd RobotStateWithOneContactStiction();

  // Makes a state in which the robot's end effector touches the ground.
  // Velocities are non-zero.
  static Eigen::VectorXd RobotStateWithOneOneContactSliding();

  // Returns the robot's "zero state".
  static Eigen::VectorXd RobotZeroState();

 private:
  // Friendship to give ToScalarType() access to private members.
  template <typename U>
  friend class RobotModel;

  // We set the initial state of the plate to be away from the robot and away
  // from the ground so that there  is no contact. This is to stress test the
  // sparsity treatment withing the SAP solver, which removes non-participating
  // DOFs from the problem, and solves a reduced problem instead.
  void SetPlateInitialState();

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
  const double kTimeStep{0.001};      // Discrete time step of the plant.
  const double kStiffness{1.0e4};     // In N/m.
  const double kHcDissipation{0.2};   // In s/m.
  const double kMu{0.5};              // Coefficient of friction.
  const double kRelaxationTime{0.1};  // In s.
};

// Publishes a default robot model for manual inspection via visualization. To
// see the model, run the test as a command-line executable, instead of as a
// bazel test.
void VisualizeRobotModel();

}  // namespace test
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::test::RobotModel);
