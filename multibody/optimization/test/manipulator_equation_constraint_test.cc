#include "drake/multibody/optimization/manipulator_equation_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
namespace {
const double kEps = std::numeric_limits<double>::epsilon();
class TwoFreeSpheresTest : public ::testing::Test {
 public:
  TwoFreeSpheresTest() {
    std::vector<test::SphereSpecification> spheres;
    spheres.emplace_back(0.1, 1E3, CoulombFriction<double>(0.8, 0.7));
    spheres.emplace_back(0.2, 1E3, CoulombFriction<double>(0.9, 0.7));
    spheres_ = std::make_unique<test::FreeSpheresAndBoxes<AutoDiffXd>>(
        spheres, std::vector<test::BoxSpecification>() /* no boxes */,
        CoulombFriction<double>(1, 0.8));
    spheres_double_ = std::make_unique<test::FreeSpheresAndBoxes<double>>(
        spheres_->spheres(), spheres_->boxes(), spheres_->ground_friction());
    const auto& query_port = spheres_->plant().get_geometry_query_input_port();
    const auto& query_object =
        query_port.Eval<geometry::QueryObject<AutoDiffXd>>(
            spheres_->plant_context());
    const std::set<std::pair<geometry::GeometryId, geometry::GeometryId>>
        collision_candidate_pairs =
            query_object.inspector().GetCollisionCandidates();
    const auto& plant = spheres_->plant();
    v_vars_ = prog_.NewContinuousVariables(plant.num_velocities(), "v");
    q_next_vars_ =
        prog_.NewContinuousVariables(plant.num_positions(), "q_next");
    v_next_vars_ =
        prog_.NewContinuousVariables(plant.num_velocities(), "v_next");
    int geometry_pair_count = 0;
    for (const auto& geometry_pair : collision_candidate_pairs) {
      auto wrench_evaluator =
          std::make_shared<ContactWrenchFromForceInWorldFrameEvaluator>(
              &plant, spheres_->get_mutable_plant_context(),
              SortedPair<geometry::GeometryId>(geometry_pair.first,
                                               geometry_pair.second));
      auto lambda_i = prog_.NewContinuousVariables(
          wrench_evaluator->num_lambda(),
          "lambda" + std::to_string(geometry_pair_count));
      contact_wrench_evaluators_and_lambda_.push_back(
          std::make_pair(wrench_evaluator, lambda_i));
      geometry_pair_count++;
    }
    dt_var_ = prog_.NewContinuousVariables(1, "dt")(0);
  }

  void CheckManipulatorEquationsConstraintEval(
      const solvers::Binding<ManipulatorEquationConstraint>&
          manipulator_equation_binding,
      const Eigen::Ref<const AutoDiffVecXd>& v_autodiff,
      const Eigen::Ref<const AutoDiffVecXd>& q_next_autodiff,
      const Eigen::Ref<const AutoDiffVecXd>& v_next_autodiff,
      const Eigen::Ref<const AutoDiffVecXd>& lambda_autodiff,
      const Eigen::Ref<const AutoDiffVecXd>& dt_autodiff) {
    // Manually evaluates the manipulator equation constraint
    // (Bu[n+1]  + ∑ᵢ Jᵢ(q[n+1])ᵀFᵢ_AB_W(λᵢ[n+1]) + g(q[n+1])
    //  - C(q[n+1], v[n+1])) * dt
    //  - M(q[n+1])(v[n+1] - v[n])
    // where i indicates the ith contact.
    // Notice that since the spheres are un-actuated, we don't compute the term
    // B * u. Moreover the bias term C is exactly zero.
    spheres_->plant().SetPositions(spheres_->get_mutable_plant_context(),
                                   q_next_autodiff);
    spheres_->plant().SetVelocities(spheres_->get_mutable_plant_context(),
                                    v_next_autodiff);
    const AutoDiffVecXd g_autodiff =
        spheres_->plant().CalcGravityGeneralizedForces(
            *(spheres_->get_mutable_plant_context()));
    Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, 1> C_bias_autodiff(
        spheres_->plant().num_velocities(), 1);
    spheres_->plant().CalcBiasTerm(*spheres_->get_mutable_plant_context(),
                                   &C_bias_autodiff);

    AutoDiffVecXd y_autodiff_expected = g_autodiff - C_bias_autodiff;

    const auto& query_port = spheres_->plant().get_geometry_query_input_port();
    const auto& query_object =
        query_port.Eval<geometry::QueryObject<AutoDiffXd>>(
            spheres_->plant_context());
    const std::vector<geometry::SignedDistancePair<AutoDiffXd>>
        signed_distance_pairs =
            query_object.ComputeSignedDistancePairwiseClosestPoints();
    EXPECT_EQ(signed_distance_pairs.size(), 3);
    const geometry::SceneGraphInspector<AutoDiffXd>& inspector =
        query_object.inspector();
    for (const auto& signed_distance_pair : signed_distance_pairs) {
      const geometry::FrameId frame_A_id =
          inspector.GetFrameId(signed_distance_pair.id_A);
      const geometry::FrameId frame_B_id =
          inspector.GetFrameId(signed_distance_pair.id_B);
      const Frame<AutoDiffXd>& frameA =
          spheres_->plant().GetBodyFromFrameId(frame_A_id)->body_frame();
      const Frame<AutoDiffXd>& frameB =
          spheres_->plant().GetBodyFromFrameId(frame_B_id)->body_frame();
      Eigen::Matrix<AutoDiffXd, 6, Eigen::Dynamic> Jv_V_WCa(6, 12);
      Eigen::Matrix<AutoDiffXd, 6, Eigen::Dynamic> Jv_V_WCb(6, 12);
      spheres_->plant().CalcJacobianSpatialVelocity(
          spheres_->plant_context(), JacobianWrtVariable::kV, frameA,
          signed_distance_pair.p_ACa, spheres_->plant().world_frame(),
          spheres_->plant().world_frame(), &Jv_V_WCa);
      spheres_->plant().CalcJacobianSpatialVelocity(
          spheres_->plant_context(), JacobianWrtVariable::kV, frameB,
          signed_distance_pair.p_BCb, spheres_->plant().world_frame(),
          spheres_->plant().world_frame(), &Jv_V_WCb);

      AutoDiffVecXd F_AB_W(6);

      auto lambda_indices_in_all_lambda =
          manipulator_equation_binding.evaluator()
              ->contact_pair_to_wrench_evaluator()
              .at(SortedPair<geometry::GeometryId>(signed_distance_pair.id_A,
                                                   signed_distance_pair.id_B))
              .lambda_indices_in_all_lambda;
      for (int i = 0; i < 3; ++i) {
        F_AB_W(i) = 0 * lambda_autodiff(lambda_indices_in_all_lambda[0]);
      }
      F_AB_W(3) = lambda_autodiff(lambda_indices_in_all_lambda[0]);
      F_AB_W(4) = lambda_autodiff(lambda_indices_in_all_lambda[1]);
      F_AB_W(5) = lambda_autodiff(lambda_indices_in_all_lambda[2]);
      y_autodiff_expected +=
          Jv_V_WCa.transpose() * -F_AB_W + Jv_V_WCb.transpose() * F_AB_W;
    }

    Eigen::Matrix<AutoDiffXd, Eigen::Dynamic, Eigen::Dynamic> M_mass_autodiff(
        spheres_->plant().num_velocities(), spheres_->plant().num_velocities());
    spheres_->plant().CalcMassMatrixViaInverseDynamics(
        *spheres_->get_mutable_plant_context(), &M_mass_autodiff);

    y_autodiff_expected = y_autodiff_expected * dt_autodiff -
                          M_mass_autodiff * (v_next_autodiff - v_autodiff);

    // 48 = 24 for velocities, 14 for positions, 9 for lambda, 1 for dt
    AutoDiffVecXd x_autodiff(48);
    x_autodiff << v_autodiff, q_next_autodiff, v_next_autodiff, lambda_autodiff,
        dt_autodiff;
    AutoDiffVecXd y_autodiff;
    manipulator_equation_binding.evaluator()->Eval(x_autodiff, &y_autodiff);
    EXPECT_TRUE(CompareMatrices(
        math::ExtractValue(y_autodiff),
        math::ExtractValue(y_autodiff_expected), 100 * kEps));
    EXPECT_TRUE(CompareMatrices(
        math::ExtractGradient(y_autodiff),
        math::ExtractGradient(y_autodiff_expected),
        100 * kEps));

    // Use a std::function instead of auto eval_fun as a lambda. This is
    // explained in ComputeNumericalGradient.
    std::function<void(const Eigen::Ref<const Eigen::VectorXd>&,
                       Eigen::VectorXd*)>
        eval_fun = [&manipulator_equation_binding](
                       const Eigen::Ref<const Eigen::VectorXd>& x,
                       Eigen::VectorXd* y) {
          manipulator_equation_binding.evaluator()->Eval(x, y);
        };

    const auto J = math::ComputeNumericalGradient(
        eval_fun, math::ExtractValue(x_autodiff));
    EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff), J, 1e-5));
  }

 protected:
  std::unique_ptr<test::FreeSpheresAndBoxes<AutoDiffXd>> spheres_;
  std::unique_ptr<test::FreeSpheresAndBoxes<double>> spheres_double_;
  solvers::MathematicalProgram prog_;
  VectorX<symbolic::Variable> v_vars_;
  VectorX<symbolic::Variable> q_next_vars_;
  VectorX<symbolic::Variable> v_next_vars_;
  VectorX<symbolic::Variable> u_next_vars_{0};
  std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                        VectorX<symbolic::Variable>>>
      contact_wrench_evaluators_and_lambda_;
  symbolic::Variable dt_var_;
};

TEST_F(TwoFreeSpheresTest, Construction) {
  // Test the static method MakeBinding.
  const auto& plant = spheres_->plant();
  const auto manipulator_equation_binding =
      ManipulatorEquationConstraint::MakeBinding(
          &plant, spheres_->get_mutable_plant_context(),
          contact_wrench_evaluators_and_lambda_, v_vars_, q_next_vars_,
          v_next_vars_, u_next_vars_, dt_var_);
  // Test the size of ManipulatorEquationConstraint
  EXPECT_EQ(
      manipulator_equation_binding.evaluator()->num_vars(),
      48 /* 24 for velocities, 14 for positions, 9 for lambda, 1 for dt */);
  EXPECT_EQ(manipulator_equation_binding.evaluator()->num_constraints(),
            plant.num_velocities());
  EXPECT_TRUE(
      CompareMatrices(manipulator_equation_binding.evaluator()->lower_bound(),
                      Eigen::VectorXd::Zero(12)));
  EXPECT_TRUE(
      CompareMatrices(manipulator_equation_binding.evaluator()->upper_bound(),
                      Eigen::VectorXd::Zero(12)));
  // Now check if each contact wrench evaluator is bound to the correct lambda.
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda_) {
    const auto& lambda_indices =
        manipulator_equation_binding.evaluator()
            ->contact_pair_to_wrench_evaluator()
            .at(contact_wrench_evaluator_and_lambda.first->geometry_id_pair())
            .lambda_indices_in_all_lambda;
    // Check if lambda_indices_in_all_lambda is correct.
    EXPECT_EQ(lambda_indices.size(),
              contact_wrench_evaluator_and_lambda.second.rows());
    for (int i = 0; i < static_cast<int>(lambda_indices.size()); ++i) {
      EXPECT_EQ(manipulator_equation_binding.variables()(
                    v_vars_.rows() + q_next_vars_.rows() + v_next_vars_.rows() +
                    u_next_vars_.rows() + lambda_indices[i]),
                contact_wrench_evaluator_and_lambda.second.coeff(i));
    }
  }
}

TEST_F(TwoFreeSpheresTest, Eval) {
  // Test Eval method of ManipulatorEquationConstraint.
  const auto& plant = spheres_->plant();
  const auto manipulator_equation_binding =
      ManipulatorEquationConstraint::MakeBinding(
          &plant, spheres_->get_mutable_plant_context(),
          contact_wrench_evaluators_and_lambda_, v_vars_, q_next_vars_,
          v_next_vars_, u_next_vars_, dt_var_);
  math::RigidTransform<double> X_WS0, X_WS1;
  // Set the sphere pose X_WS0 and X_WS1 arbitrarily.
  X_WS0.set(math::RotationMatrix<double>(Eigen::AngleAxisd(
                M_PI_4, Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3))),
            Eigen::Vector3d(0.1, 0.2, 0.3));
  X_WS1.set(math::RotationMatrix<double>(Eigen::AngleAxisd(
                -0.1, Eigen::Vector3d(0.1, 0.2, 0.3).normalized())),
            Eigen::Vector3d(0.15, 0.25, 0.2));

  // Set the velocities to arbitrary values.
  Eigen::VectorXd v_val(12), v_next_val(12);
  v_val << .01, .02, .03, .04, .05, .06, .07, .08, .09, .1, .11, .12;
  v_next_val << v_val * 2;

  Eigen::VectorXd q_next_val(14);
  q_next_val.head<4>() = X_WS0.rotation().ToQuaternionAsVector4();
  q_next_val.segment<3>(4) = X_WS0.translation();
  q_next_val.segment<4>(7) = X_WS1.rotation().ToQuaternionAsVector4();
  q_next_val.tail<3>() = X_WS1.translation();

  Eigen::VectorXd lambda_val(9);
  // Test lambda = 0.
  lambda_val.setZero();
  double dt_val = 0.1;
  // 48 = 24 for velocities, 14 for positions, 9 for lambda, 1 for dt
  Eigen::VectorXd x_val(48);
  x_val << v_val, q_next_val, v_next_val, lambda_val, dt_val;

  auto x_autodiff = math::InitializeAutoDiff(x_val);

  CheckManipulatorEquationsConstraintEval(
      manipulator_equation_binding, x_autodiff.head<12>(),
      x_autodiff.segment<14>(12), x_autodiff.segment<12>(12 + 14),
      x_autodiff.segment<9>(12 + 14 + 12), x_autodiff.tail<1>());

  // Test lambda != 0
  lambda_val << 2, 3, 4, 5, 6, 7, 8, 9, 10;
  x_val << v_val, q_next_val, v_next_val, lambda_val, dt_val;
  x_autodiff = math::InitializeAutoDiff(x_val);
  CheckManipulatorEquationsConstraintEval(
      manipulator_equation_binding, x_autodiff.head<12>(),
      x_autodiff.segment<14>(12), x_autodiff.segment<12>(12 + 14),
      x_autodiff.segment<9>(12 + 14 + 12), x_autodiff.tail<1>());

  // Solves the optimization problem.
  // Note that we don't impose the complementarity constraint signed_distance *
  // contact_force = 0, or the friction cone constraint.
  // First set the spheres' positions, such that they are both on the ground,
  // and touching each other.
  X_WS0.set_translation(Eigen::Vector3d(0, 0, spheres_->spheres()[0].radius));
  X_WS1.set_translation(
      Eigen::Vector3d(std::sqrt(4 * spheres_->spheres()[0].radius *
                                spheres_->spheres()[1].radius),
                      0, spheres_->spheres()[1].radius));
  q_next_val.head<4>() << 1, 0, 0, 0;
  q_next_val.segment<3>(4) = X_WS0.translation();
  q_next_val.segment<4>(7) << 1, 0, 0, 0;
  q_next_val.tail<3>() = X_WS1.translation();

  prog_.AddConstraint(manipulator_equation_binding);

  Eigen::VectorXd x_init(prog_.num_vars());
  x_init.setZero();
  prog_.SetDecisionVariableValueInVector(v_vars_, v_val, &x_init);
  prog_.SetDecisionVariableValueInVector(q_next_vars_, q_next_val, &x_init);
  prog_.SetDecisionVariableValueInVector(v_next_vars_, v_next_val, &x_init);
  prog_.SetDecisionVariableValueInVector(
      contact_wrench_evaluators_and_lambda_[1].second,
      Eigen::Vector3d(0, 0, -spheres_->spheres()[0].inertia.get_mass() * 9.81),
      &x_init);
  prog_.SetDecisionVariableValueInVector(
      contact_wrench_evaluators_and_lambda_[2].second,
      Eigen::Vector3d(0, 0, -spheres_->spheres()[1].inertia.get_mass() * 9.81),
      &x_init);
  prog_.SetDecisionVariableValueInVector(dt_var_, dt_val, &x_init);

  auto result = solvers::Solve(prog_, x_init);
  EXPECT_TRUE(result.is_success());

  // Given the solution, now manually check that the system satisfies the
  // manipulator equation constraint.
  // First, reset the plant context to match the solution vars.
  spheres_double_->plant().SetPositions(
      spheres_double_->get_mutable_plant_context(),
      result.GetSolution(q_next_vars_));
  spheres_double_->plant().SetVelocities(
      spheres_double_->get_mutable_plant_context(),
      result.GetSolution(v_next_vars_));
  X_WS0 = spheres_double_->plant().GetFreeBodyPose(
      spheres_double_->plant_context(),
      spheres_double_->plant().GetBodyByName("sphere0"));
  X_WS1 = spheres_double_->plant().GetFreeBodyPose(
      spheres_double_->plant_context(),
      spheres_double_->plant().GetBodyByName("sphere1"));

  // Compute the total wrench applied on each sphere, expressed in the world
  // frame.
  Vector6<double> sphere0_total_wrench, sphere1_total_wrench;
  const double gravity = 9.81;
  sphere0_total_wrench.tail<3>() << 0, 0,
      -spheres_double_->spheres()[0].inertia.get_mass() * gravity;
  sphere0_total_wrench.head<3>() =
      X_WS0.translation().cross(sphere0_total_wrench.tail<3>());
  sphere1_total_wrench.tail<3>() << 0, 0,
      -spheres_double_->spheres()[1].inertia.get_mass() * gravity;
  sphere1_total_wrench.head<3>() =
      X_WS1.translation().cross(sphere1_total_wrench.tail<3>());
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda_) {
    const Eigen::Vector3d lambda_sol =
        result.GetSolution(contact_wrench_evaluator_and_lambda.second);
    if (contact_wrench_evaluator_and_lambda.first->geometry_id_pair() ==
        SortedPair<geometry::GeometryId>(spheres_->sphere_geometry_ids()[0],
                                         spheres_->sphere_geometry_ids()[1])) {
      // Contact between two spheres.
      // Compute the witness points on sphere 0 and sphere 1.
      const Eigen::Vector3d p_WC0 =
          X_WS0.translation() +
          (X_WS1.translation() - X_WS0.translation()).normalized() *
              spheres_double_->spheres()[0].radius;
      const Eigen::Vector3d p_WC1 =
          X_WS1.translation() +
          (X_WS0.translation() - X_WS1.translation()).normalized() *
              spheres_double_->spheres()[1].radius;
      sphere0_total_wrench.head<3>() += p_WC0.cross(-lambda_sol);
      sphere1_total_wrench.head<3>() += p_WC1.cross(lambda_sol);
      sphere0_total_wrench.tail<3>() += -lambda_sol;
      sphere1_total_wrench.tail<3>() += lambda_sol;
    } else if (contact_wrench_evaluator_and_lambda.first->geometry_id_pair() ==
               SortedPair<geometry::GeometryId>(
                   spheres_->sphere_geometry_ids()[0],
                   spheres_->ground_geometry_id())) {
      // Contact between the ground and sphere 0.
      // Compute the witness point on sphere 0
      const Eigen::Vector3d p_WC0 =
          X_WS0.translation() -
          Eigen::Vector3d(0, 0, spheres_double_->spheres()[0].radius);
      sphere0_total_wrench.head<3>() += p_WC0.cross(-lambda_sol);
      sphere0_total_wrench.tail<3>() += -lambda_sol;
    } else if (contact_wrench_evaluator_and_lambda.first->geometry_id_pair() ==
               SortedPair<geometry::GeometryId>(
                   spheres_->sphere_geometry_ids()[1],
                   spheres_->ground_geometry_id())) {
      // contact between the ground and sphere 1.
      // Compute the witness point on sphere 1
      const Eigen::Vector3d p_WC1 =
          X_WS1.translation() -
          Eigen::Vector3d(0, 0, spheres_double_->spheres()[1].radius);
      sphere1_total_wrench.head<3>() += p_WC1.cross(-lambda_sol);
      sphere1_total_wrench.tail<3>() += -lambda_sol;
    } else {
      throw std::runtime_error("Unknown contact geometry pairs.");
    }
  }

  Eigen::Matrix<double, Eigen::Dynamic, 1> C_bias(
      spheres_double_->plant().num_velocities(), 1);
  spheres_double_->plant().CalcBiasTerm(
      *spheres_double_->get_mutable_plant_context(), &C_bias);
  EXPECT_TRUE(CompareMatrices(C_bias, Eigen::VectorXd(12).setZero(), 1e-12));

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M_mass(
      spheres_double_->plant().num_velocities(),
      spheres_double_->plant().num_velocities());
  spheres_double_->plant().CalcMassMatrixViaInverseDynamics(
      *spheres_double_->get_mutable_plant_context(), &M_mass);
  auto v_sol = result.GetSolution(v_vars_);
  auto v_next_sol = result.GetSolution(v_next_vars_);
  auto dt_sol = result.GetSolution(dt_var_);

  // Compute the inertial forces (i.e., the left hand side of the multibody
  // dynamics equation), expressed in the world frame.
  Eigen::VectorXd lhs = M_mass * (v_next_sol - v_sol) / dt_sol;
  lhs.head<3>() = lhs.head<3>() + X_WS0.translation().cross(lhs.segment<3>(3));
  lhs.segment<3>(6) =
      lhs.segment<3>(6) + X_WS1.translation().cross(lhs.tail<3>());

  // The solver's default tolerance is 1E-6 (with normalization). The
  // unnormalized tolerance is about 1E-5.
  const double tol = 1E-5;
  EXPECT_TRUE(CompareMatrices(sphere0_total_wrench, lhs.head<6>(), tol));
  EXPECT_TRUE(CompareMatrices(sphere1_total_wrench, lhs.tail<6>(), tol));
}
}  // namespace
}  // namespace multibody
}  // namespace drake
