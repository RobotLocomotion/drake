#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/common/text_logging.h"
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
#include "drake/multibody/test_utilities/robot_model.h"
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
using drake::multibody::test::RobotModel;
using drake::multibody::test::RobotModelConfig;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
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

// Test that can be used to manually inspect the model used for testing. To see
// the model, run the test as a command-line executable, instead of as a bazel
// test.
// To show the all proximity properties in Meshcat (even if they don't have
// illustration), check the box "proximity" under "drake" in Meshcat's controls
// panel.
GTEST_TEST(RobotModel, Visualize) {
  test::VisualizeRobotModel();
}

constexpr double kEps = std::numeric_limits<double>::epsilon();

// Helper to compare the value and gradients of x against value_expected and
// gradient_expected.
void ValidateValueAndGradients(const VectorX<AutoDiffXd>& x,
                               const VectorX<double>& value_expected,
                               const MatrixX<double>& gradient_expected,
                               const double value_tolerance,
                               const double gradient_tolerance) {
  const VectorXd value = math::ExtractValue(x);
  const MatrixXd gradient = math::ExtractGradient(x);
  EXPECT_TRUE(CompareMatrices(value, value_expected, value_tolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(gradient, gradient_expected, gradient_tolerance,
                              MatrixCompareType::relative));
}

struct SapSolverIiwaRobotTestConfig {
  // This is a gtest test suffix; no underscores or spaces.
  std::string description;
  // Initial state of the robot in RobotModel.
  VectorX<double> robot_x0;
  // Whether the AutoDiffXd problem has gradients or not.
  bool with_gradients{true};
  // SAP solver relative tolerance.
  double solver_rel_tolerance{1.0e-13};
  // Expected error against gradients computed numerically. This expected error
  // will need to include errors in the numerical computation itself.
  double expected_error{0.0};
  // Perturbation used to compute numerical gradients.
  double perturbation{kEps};
  DiscreteContactApproximation contact_approximation{
      DiscreteContactApproximation::kSimilar};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out,
                         const SapSolverIiwaRobotTestConfig& c) {
  out << c.description;
  return out;
}

// Test fixture to setup a RobotModel on T = double and T = AutoDiffXd, so that
// we can compute gradients numerically as a reference solution to test
// gradients through SAP.
class SapSolverIiwaRobotTest
    : public ::testing::TestWithParam<SapSolverIiwaRobotTestConfig> {
 public:
  void SetUp() override {
    const SapSolverIiwaRobotTestConfig& config = GetParam();
    RobotModelConfig robot_config{.contact_approximation =
                                      config.contact_approximation};
    model_ = std::make_unique<RobotModel<double>>(robot_config);
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
    const SapSolverIiwaRobotTestConfig& config = GetParam();

    // For a very precise control of the solver accuracy, we set all tolerances
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
std::vector<SapSolverIiwaRobotTestConfig> MakeNumericalGradientsTestCases() {
  return std::vector<SapSolverIiwaRobotTestConfig>{
      {
          .description = "NoContact",
          .robot_x0 = VectorX<double>::Zero(14),
          .solver_rel_tolerance = 1.0e-13,
          .expected_error = 4.0e-11,
      },
      {
          .description = "OneContactStiction",
          .robot_x0 = RobotModel<double>::RobotStateWithOneContactStiction(),
          .with_gradients = true,
          .solver_rel_tolerance = 1.0e-13,
          .expected_error = 2.0e-8,
          .perturbation = kEps,
          .contact_approximation = DiscreteContactApproximation::kSimilar,
      },
      {
          .description = "OneContactSlip",
          .robot_x0 = RobotModel<double>::RobotStateWithOneOneContactSliding(),
          .solver_rel_tolerance = 1.0e-13,
          .expected_error = 4.0e-9,
      },
      {
          .description = "OneContactStictionNoGradients",
          .robot_x0 = RobotModel<double>::RobotStateWithOneContactStiction(),
          .with_gradients = false,
          .solver_rel_tolerance = 1.0e-13,
      },
  };
}

INSTANTIATE_TEST_SUITE_P(SapDriverGradientsTest, SapSolverIiwaRobotTest,
                         testing::ValuesIn(MakeNumericalGradientsTestCases()),
                         testing::PrintToStringParamName());

TEST_P(SapSolverIiwaRobotTest, CompareNumericalGradients) {
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  const SapSolverIiwaRobotTestConfig& config = GetParam();

  RobotModel<double>& model = get_mutable_model<double>();
  model.SetRobotState(config.robot_x0);
  const VectorX<double> x0 = model.GetState();
  VectorX<double> v(model.num_velocities());

  auto start = high_resolution_clock::now();
  CalcNextStepVelocities(x0, &v);
  auto duration =
      duration_cast<microseconds>(high_resolution_clock::now() - start);
  drake::log()->info("Discrete Update<double>: {} s", duration.count() / 1e6);

  const VectorX<AutoDiffXd> x0_ad = config.with_gradients
                                        ? math::InitializeAutoDiff(x0)
                                        : VectorX<AutoDiffXd>(x0);
  VectorX<AutoDiffXd> v_ad(v.size());

  start = high_resolution_clock::now();
  CalcNextStepVelocities(x0_ad, &v_ad);
  duration = duration_cast<microseconds>(high_resolution_clock::now() - start);
  drake::log()->info("Discrete Update<AutoDiffXd>: {} s",
                     duration.count() / 1e6);

  // Values must be within machine epsilon.
  const VectorXd v_value = math::ExtractValue(v_ad);
  EXPECT_TRUE(CompareMatrices(v_value, v, kEps, MatrixCompareType::relative));

  const MatrixXd v_gradient = math::ExtractGradient(v_ad);
  // No need to compute expensive numerical gradients if we know they are
  // zero.
  if (config.with_gradients) {
    std::function<void(const VectorX<double>& x, VectorX<double>*)>
        velocity_update = [&](const VectorX<double>& initial_state,
                              VectorX<double>* velocities) -> void {
      CalcNextStepVelocities(initial_state, velocities);
    };

    start = high_resolution_clock::now();
    const MatrixX<double> dv_dx0 = math::ComputeNumericalGradient(
        velocity_update, x0,
        math::NumericalGradientOption(math::NumericalGradientMethod::kCentral,
                                      config.perturbation));
    duration =
        duration_cast<microseconds>(high_resolution_clock::now() - start);
    drake::log()->info("Finite Differences: {} s", duration.count() / 1e6);

    EXPECT_TRUE(CompareMatrices(v_gradient, dv_dx0, config.expected_error,
                                MatrixCompareType::relative));
  } else {
    EXPECT_EQ(v_gradient.size(), 0);
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
