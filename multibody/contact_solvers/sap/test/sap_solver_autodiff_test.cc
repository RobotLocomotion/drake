#include <chrono>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/visualization/visualization_config_functions.h"

using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using drake::multibody::internal::CompliantContactManager;
using drake::multibody::internal::CompliantContactManagerTester;
using drake::multibody::internal::SapDriver;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {

namespace internal {
// Friend helper class to access SapDriver private internals for testing.
class SapDriverTest {
 public:
  template <typename T>
  static const ContactProblemCache<T>& EvalContactProblemCache(
      const SapDriver<T>& driver, const Context<T>& context) {
    return driver.EvalContactProblemCache(context);
  }
};
}  // namespace internal

namespace contact_solvers {
namespace internal {

// Tests that SolveWithGuess returns the free motion velocities and zero
// impulses if the constraints are empty.
GTEST_TEST(SapAutoDiffTest, ProblemWithNoConstraints) {
  constexpr double kTimeStep = 0.001;
  constexpr int kNumDofs = 5;
  constexpr int kNumCliques = 1;
  std::vector<MatrixX<AutoDiffXd>> A(kNumCliques);
  A[0] = MatrixX<AutoDiffXd>::Identity(kNumDofs, kNumDofs);
  VectorX<AutoDiffXd> v_star =
      VectorX<AutoDiffXd>::LinSpaced(kNumDofs, 0.0, 1.0);
  SapSolver<AutoDiffXd> sap;

  // The case without constraints.
  SapContactProblem<AutoDiffXd> contact_problem_without_constraint(
      kTimeStep, std::move(A), v_star);
  // Use NaN as the guess as it shouldn't be used at all in the early exit where
  // there's no constraint.
  const VectorX<AutoDiffXd> v_guess =
      VectorX<AutoDiffXd>::Constant(kNumDofs, NAN);
  SapSolverResults<AutoDiffXd> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(contact_problem_without_constraint, v_guess, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);
  EXPECT_EQ(result.v, v_star);
  EXPECT_EQ(result.j, VectorX<AutoDiffXd>::Zero(kNumDofs));
}

// Helper class to create an interesting SapContactProblem with problem data
// function of the parameters of differentiation, in this case the initial
// state. This allows to exercise the entire numeric pipeline with gradients
// propagating through complex terms such as the mass matrix, contact Jacobians
// and even contact data.
//
// In particular, we load the model of an IIWA7 arm, to be set into a
// configuration where the end effector contacts the ground.
template <typename T>
class RobotModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotModel);

  // Creates an empty model.
  RobotModel() = default;

  RobotModel(DiscreteContactApproximation contact_approximation,
             bool add_visualization = false) {
    DiagramBuilder<double> builder;
    auto items = AddMultibodyPlantSceneGraph(&builder, kTimeStep_);
    plant_ = &items.plant;

    Parser(plant_).AddModelsFromUrl(
        "package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf");

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
        plant_->world_body(), RigidTransformd(Vector3d(0.0, 0.0, -0.05)),
        geometry::Box(2.0, 2.0, 0.1), "ground_collision", proximity_properties);

    // Add simple contact geometry at the end effector.
    plant_->RegisterCollisionGeometry(
        plant_->GetBodyByName("iiwa_link_7"),
        RigidTransformd(Vector3d(0.0, 0.0, 0.07)), geometry::Sphere(0.05),
        "iiwa_link_7_collision", proximity_properties);

    plant_->set_discrete_contact_approximation(contact_approximation);
    plant_->Finalize();

    // Make and add a manager so that we have access to it and its driver.
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));
    driver_ = &CompliantContactManagerTester::sap_driver(*manager_);

    // We add visualizer for visual inspection of the model in this test.
    if (add_visualization) {
      // visualization::AddDefaultVisualization(&builder);
      visualization::VisualizationConfig vis_config;
      visualization::ApplyVisualizationConfig(vis_config, &builder);
    }

    diagram_ = builder.Build();

    // Create context.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, context_.get());

    // Fix input ports.
    const VectorX<double> tau =
        VectorX<double>::Zero(plant_->num_actuated_dofs());
    plant_->get_actuation_input_port().FixValue(plant_context_, tau);
  }

  const MultibodyPlant<T>& plant() const { return *plant_; }
  const Diagram<T>& diagram() const { return *diagram_; }

  int num_velocities() const { return plant_->num_velocities(); }

  std::unique_ptr<RobotModel<AutoDiffXd>> ToAutoDiffXd() const {
    auto model_ad = std::make_unique<RobotModel<AutoDiffXd>>();

    // Scalar-convert the model.
    model_ad->diagram_ =
        dynamic_pointer_cast<Diagram<AutoDiffXd>>(diagram_->ToAutoDiffXd());
    model_ad->plant_ = const_cast<MultibodyPlant<AutoDiffXd>*>(
        &model_ad->diagram_
             ->GetDowncastSubsystemByName<MultibodyPlant<AutoDiffXd>>(
                 plant_->get_name()));

    // Make and add a manager so that we have access to it and its driver.
    auto owned_contact_manager_ad =
        std::make_unique<CompliantContactManager<AutoDiffXd>>();
    model_ad->manager_ = owned_contact_manager_ad.get();
    model_ad->plant_->SetDiscreteUpdateManager(
        std::move(owned_contact_manager_ad));
    model_ad->driver_ =
        &CompliantContactManagerTester::sap_driver(*model_ad->manager_);

    // Create context.
    model_ad->context_ = model_ad->diagram_->CreateDefaultContext();
    model_ad->context_->SetTimeStateAndParametersFrom(*context_);
    model_ad->plant_context_ = &model_ad->diagram_->GetMutableSubsystemContext(
        *model_ad->plant_, model_ad->context_.get());

    // Fix input ports.
    const VectorX<AutoDiffXd> tau_ad =
        VectorX<AutoDiffXd>::Zero(plant_->num_actuated_dofs());
    model_ad->plant_->get_actuation_input_port().FixValue(
        model_ad->plant_context_, tau_ad);

    return model_ad;
  }

  void ForcedPublish() { diagram_->ForcedPublish(*context_); }

  void SetState(const VectorX<T>& x) {
    plant_->SetPositionsAndVelocities(plant_context_, x);
  }

  // Helper that uses the underlying SapDriver to evaluate the SapContactProblem
  // at x0.
  const SapContactProblem<T>& EvalContactProblem(const VectorX<T>& x0) {
    SetState(x0);
    const auto& problem_cache =
        drake::multibody::internal::SapDriverTest::EvalContactProblemCache(
            *driver_, *plant_context_);
    // There are no locked dofs. Sanity check.
    DRAKE_DEMAND(problem_cache.sap_problem_locked == nullptr);
    return *problem_cache.sap_problem;
  }

 private:
  // Friendship to give ToAutoDiffXd() access to private members.
  template <typename U>
  friend class RobotModel;

  std::unique_ptr<Diagram<T>> diagram_;
  MultibodyPlant<T>* plant_{nullptr};
  std::unique_ptr<Context<T>> context_;
  Context<T>* plant_context_{nullptr};
  CompliantContactManager<T>* manager_{nullptr};
  const SapDriver<T>* driver_{nullptr};

  // Parameters of the problem.
  const double kTimeStep_{0.001};   // Discrete time step of the plant.
  const double kStiffness_{1.0e4};  // In N/m.
  const double kHydroelasticModulus_{250.0};  // In Pa.
  const double kHcDissipation_{0.2};          // In s/m.
  const double kGroundThickness_{0.1};        // In m.
  const double kMu_{0.5};                     // Coefficient of friction.
  const double kRelaxationTime_{0.1};         // In s.
};

// Test that can be used to manually inspect the model used for testing. To see
// the model, run the test as a command-line executable, instead of as a bazel
// test.
GTEST_TEST(RobotModel, Visualize) {
  RobotModel<double> model(DiscreteContactApproximation::kSimilar,
                           true /* add viz */);
  // Set contact configuration.
  const VectorX<double> x0 =
      (VectorX<double>(14) << 0, 1.17, 0, -1.33, 0, 0.58, 0,  // q
       0, 0, 0, 0, 0, 0, 0                                    // v
       )
          .finished();
  model.SetState(x0);

  model.ForcedPublish();
  common::MaybePauseForUser();
}

constexpr double kEps = std::numeric_limits<double>::epsilon();

// Helper to compare the value and gradients of x against value_expected and
// gradient_expected.
void ValidateValueAndGradients(const VectorX<AutoDiffXd>& x,
                               const VectorX<double>& value_expected,
                               const MatrixX<double>& gradient_expected,
                               const double tolerance = kEps) {
  const VectorXd value = math::ExtractValue(x);
  const auto gradient = math::ExtractGradient(x);
  EXPECT_TRUE(CompareMatrices(value, value_expected, tolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(gradient, gradient_expected, tolerance,
                              MatrixCompareType::relative));
}

struct SapSolverIiwaRobotTestTestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  // Initial state for RobotModel.
  VectorX<double> x0;
  // SAP solver relative tolerance.
  double solver_rel_tolerance;
  // Expected error against gradients computed numerically. This expected error
  // will need to include errors in the numerical computation itself.
  double expected_error;
  // Perturbation used to compute numerical gradients.
  double perturbation;
  DiscreteContactApproximation contact_approximation{
      DiscreteContactApproximation::kSap};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out,
                         const SapSolverIiwaRobotTestTestConfig& c) {
  out << c.description;
  return out;
}

// Test fixture to setup a RobotModel on bout T = double and T = AutoDiffXd, so
// that we can compute gradients numerically as a reference solution to test
// gradients through SAP.
class SapSolverIiwaRobotTestTest
    : public ::testing::TestWithParam<SapSolverIiwaRobotTestTestConfig> {
 public:
  void SetUp() override {
    const SapSolverIiwaRobotTestTestConfig& config = GetParam();
    model_ = std::make_unique<RobotModel<double>>(config.contact_approximation);
    model_ad_ = model_->ToAutoDiffXd();
  }

  template <typename T>
  const RobotModel<T>& get_model() const {
    if constexpr (std::is_same_v<T, double>) {
      return *model_;
    } else {
      return *model_ad_;
    }
  }

  template <typename T>
  RobotModel<T>& get_mutable_model() {
    if constexpr (std::is_same_v<T, double>) {
      return *model_;
    } else {
      return *model_ad_;
    }
  }

  // Solves the SAP problem for velocities v, starting from the initial state
  // x0.
  template <typename T>
  void CalcNextStepVelocities(const VectorX<T>& x0, VectorX<T>* v) {
    RobotModel<T>& model = get_mutable_model<T>();
    model.SetState(x0);
    const VectorX<T> v0 = x0.tail(model.num_velocities());
    const SapContactProblem<T>& problem = model.EvalContactProblem(x0);
    *v = SolveContactProblem(problem, v0);
  }

  // Helper to setup a SapSolver to solve the given SapContactProblem. v0 is the
  // initial guess.
  template <typename T>
  VectorX<T> SolveContactProblem(const SapContactProblem<T>& problem,
                                 const VectorX<T>& v0) const {
    const SapSolverIiwaRobotTestTestConfig& config = GetParam();

    // For a very pricise control of the solver accuracy, we set all tolerances
    // to zero except rel_tolerance. Therefore accuracy will be controlled
    // solely by this parameter.
    contact_solvers::internal::SapSolverParameters sap_parameters;
    sap_parameters.abs_tolerance = 0.0;
    sap_parameters.rel_tolerance = config.solver_rel_tolerance;
    sap_parameters.cost_abs_tolerance = 0.0;
    sap_parameters.cost_rel_tolerance = 0.0;
    sap_parameters.linear_solver_type =
        contact_solvers::internal::SapHessianFactorizationType::kDense;

    SapSolver<T> sap;
    sap.set_parameters(sap_parameters);

    SapSolverResults<T> results;
    const SapSolverStatus status = sap.SolveWithGuess(problem, v0, &results);
    const bool success = status == SapSolverStatus::kSuccess;
    EXPECT_TRUE(success);
    return results.v;
  }

 protected:
  std::unique_ptr<RobotModel<double>> model_;
  std::unique_ptr<RobotModel<AutoDiffXd>> model_ad_;
};

// Setup test cases using point and hydroelastic contact.
std::vector<SapSolverIiwaRobotTestTestConfig>
MakeNumericalGradientsTestCases() {
  return std::vector<SapSolverIiwaRobotTestTestConfig>{
      {
          .description = "NoContactSimilar",
          .x0 = VectorX<double>::Zero(14),
          .solver_rel_tolerance = 1.0e-13,
          .expected_error = 4.0e-11,
          .perturbation = kEps,
          .contact_approximation = DiscreteContactApproximation::kSimilar,
      },
      {
          .description = "OneContactStictionSimilar",
          .x0 = (VectorX<double>(14) << 0, 1.17, 0, -1.33, 0, 0.58, 0,  // q
                 0, 0, 0, 0, 0, 0, 0                                    // v
                 )
                    .finished(),
          .solver_rel_tolerance = 1.0e-13,
          .expected_error = 2.0e-8,
          .perturbation = kEps,
          .contact_approximation = DiscreteContactApproximation::kSimilar,
      },
      {
          .description = "OneContactSlipSimilar",
          .x0 = (VectorX<double>(14) << 0, 1.17, 0, -1.33, 0, 0.58, 0,  // q
                 0, -0.1, 0, -0.2, 0, 0, 0                              // v
                 )
                    .finished(),
          .solver_rel_tolerance = 1.0e-13,
          .expected_error = 4.0e-9,
          .perturbation = kEps,
          .contact_approximation = DiscreteContactApproximation::kSimilar,
      },
  };
}

INSTANTIATE_TEST_SUITE_P(SapDriverGradientsTest, SapSolverIiwaRobotTestTest,
                         testing::ValuesIn(MakeNumericalGradientsTestCases()),
                         testing::PrintToStringParamName());

TEST_P(SapSolverIiwaRobotTestTest, CompareNumericalGradients) {
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  const SapSolverIiwaRobotTestTestConfig& config = GetParam();

  const VectorX<double> x0 = config.x0;
  VectorX<double> v(7);

  auto start = high_resolution_clock::now();
  CalcNextStepVelocities(x0, &v);
  auto duration =
      duration_cast<microseconds>(high_resolution_clock::now() - start);
  drake::log()->info("Discrete Update<double>: {}s", duration.count() / 1e6);

  std::function<void(const VectorX<double>& x, VectorX<double>*)> next_state =
      [&](const VectorX<double>& X, VectorX<double>* V) -> void {
    CalcNextStepVelocities(X, V);
  };

  start = high_resolution_clock::now();
  MatrixX<double> dv_dx0 = math::ComputeNumericalGradient(
      next_state, x0,
      math::NumericalGradientOption(math::NumericalGradientMethod::kCentral,
                                    config.perturbation));
  duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
  drake::log()->info("Finite Differences: {}s", duration.count() / 1e6);

  const VectorX<AutoDiffXd> x0_ad = math::InitializeAutoDiff(x0);
  VectorX<AutoDiffXd> v_ad(7);

  start = high_resolution_clock::now();
  CalcNextStepVelocities(x0_ad, &v_ad);
  duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
  drake::log()->info("Discrete Update<AutoDiffXd>: {}s",
                     duration.count() / 1e6);

  ValidateValueAndGradients(v_ad, v, dv_dx0, config.expected_error);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
