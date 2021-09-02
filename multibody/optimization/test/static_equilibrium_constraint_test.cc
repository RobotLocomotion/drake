#include "drake/multibody/optimization/static_equilibrium_constraint.h"

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
    q_vars_ = prog_.NewContinuousVariables(plant.num_positions(), "q");
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
  }

  void CheckStaticEquilibriumConstraintEval(
      const solvers::Binding<StaticEquilibriumConstraint>&
          static_equilibrium_binding,
      const Eigen::Ref<const AutoDiffVecXd>& q_autodiff,
      const Eigen::Ref<const AutoDiffVecXd> lambda_autodiff) {
    // Manually evaluates the static equilibrium constraint g(q) + J'*lambda
    // Notice that since the spheres are un-actuated, we don't compute the term
    // B * u.
    spheres_->plant().SetPositions(spheres_->get_mutable_plant_context(),
                                   q_autodiff);
    const AutoDiffVecXd g_autodiff =
        spheres_->plant().CalcGravityGeneralizedForces(
            *(spheres_->get_mutable_plant_context()));

    AutoDiffVecXd y_autodiff_expected = g_autodiff;

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
          static_equilibrium_binding.evaluator()
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

    AutoDiffVecXd x_autodiff(23);
    x_autodiff << q_autodiff, lambda_autodiff;
    AutoDiffVecXd y_autodiff;
    static_equilibrium_binding.evaluator()->Eval(x_autodiff, &y_autodiff);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(y_autodiff),
                                math::ExtractValue(y_autodiff_expected),
                                100 * kEps));
    EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff),
                                math::ExtractGradient(y_autodiff_expected),
                                100 * kEps));

    // Use a std::function instead of auto eval_fun as a lambda. This is
    // explained in ComputeNumericalGradient.
    std::function<void(const Eigen::Ref<const Eigen::VectorXd>&,
                       Eigen::VectorXd*)>
        eval_fun = [&static_equilibrium_binding](
                       const Eigen::Ref<const Eigen::VectorXd>& x,
                       Eigen::VectorXd* y) {
          static_equilibrium_binding.evaluator()->Eval(x, y);
        };

    const auto J = math::ComputeNumericalGradient(
        eval_fun, math::ExtractValue(x_autodiff));
    EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff), J, 1e-5));
  }

 protected:
  std::unique_ptr<test::FreeSpheresAndBoxes<AutoDiffXd>> spheres_;
  std::unique_ptr<test::FreeSpheresAndBoxes<double>> spheres_double_;
  solvers::MathematicalProgram prog_;
  VectorX<symbolic::Variable> q_vars_;
  VectorX<symbolic::Variable> u_vars_{0};
  std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                        VectorX<symbolic::Variable>>>
      contact_wrench_evaluators_and_lambda_;
};

TEST_F(TwoFreeSpheresTest, Construction) {
  // Test the static method MakeBinding.
  const auto& plant = spheres_->plant();
  const auto static_equilibrium_binding =
      StaticEquilibriumConstraint::MakeBinding(
          &plant, spheres_->get_mutable_plant_context(),
          contact_wrench_evaluators_and_lambda_, q_vars_, u_vars_);
  // Test the size of StaticEquilibriumConstraint
  EXPECT_EQ(static_equilibrium_binding.evaluator()->num_vars(),
            23 /* 14 for position, 9 for lambda */);
  EXPECT_EQ(static_equilibrium_binding.evaluator()->num_constraints(),
            plant.num_velocities());
  EXPECT_TRUE(
      CompareMatrices(static_equilibrium_binding.evaluator()->lower_bound(),
                      Eigen::VectorXd::Zero(12)));
  EXPECT_TRUE(
      CompareMatrices(static_equilibrium_binding.evaluator()->upper_bound(),
                      Eigen::VectorXd::Zero(12)));
  // Now check if each contact wrench evaluator is bound to the correct lambda.
  for (const auto& contact_wrench_evaluator_and_lambda :
       contact_wrench_evaluators_and_lambda_) {
    const auto& lambda_indices =
        static_equilibrium_binding.evaluator()
            ->contact_pair_to_wrench_evaluator()
            .at(contact_wrench_evaluator_and_lambda.first->geometry_id_pair())
            .lambda_indices_in_all_lambda;
    // Check if lambda_indices_in_all_lambda is correct.
    EXPECT_EQ(lambda_indices.size(),
              contact_wrench_evaluator_and_lambda.second.rows());
    for (int i = 0; i < static_cast<int>(lambda_indices.size()); ++i) {
      EXPECT_EQ(static_equilibrium_binding.variables()(
                    q_vars_.rows() + u_vars_.rows() + lambda_indices[i]),
                contact_wrench_evaluator_and_lambda.second.coeff(i));
    }
  }
}

TEST_F(TwoFreeSpheresTest, Eval) {
  // Test Eval method of StaticEquilibriumConstraint.
  const auto& plant = spheres_->plant();
  const auto static_equilibrium_binding =
      StaticEquilibriumConstraint::MakeBinding(
          &plant, spheres_->get_mutable_plant_context(),
          contact_wrench_evaluators_and_lambda_, q_vars_, u_vars_);
  math::RigidTransform<double> X_WS0, X_WS1;
  // Set the sphere pose X_WS0 and X_WS1 arbitrarily.
  X_WS0.set(math::RotationMatrix<double>(Eigen::AngleAxisd(
                M_PI_4, Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3))),
            Eigen::Vector3d(0.1, 0.2, 0.3));
  X_WS1.set(math::RotationMatrix<double>(Eigen::AngleAxisd(
                -0.1, Eigen::Vector3d(0.1, 0.2, 0.3).normalized())),
            Eigen::Vector3d(0.15, 0.25, 0.2));

  Eigen::VectorXd q_val(14);
  q_val.head<4>() = X_WS0.rotation().ToQuaternionAsVector4();
  q_val.segment<3>(4) = X_WS0.translation();
  q_val.segment<4>(7) = X_WS1.rotation().ToQuaternionAsVector4();
  q_val.tail<3>() = X_WS1.translation();
  Eigen::VectorXd lambda_val(9);
  // Test lambda = 0.
  lambda_val.setZero();
  Eigen::VectorXd x_val(23);
  x_val << q_val, lambda_val;
  auto x_autodiff = math::InitializeAutoDiff(x_val);
  CheckStaticEquilibriumConstraintEval(
      static_equilibrium_binding, x_autodiff.head<14>(), x_autodiff.tail<9>());
  // Test lambda != 0
  lambda_val << 2, 3, 4, 5, 6, 7, 8, 9, 10;
  x_val << q_val, lambda_val;
  x_autodiff = math::InitializeAutoDiff(x_val);
  CheckStaticEquilibriumConstraintEval(
      static_equilibrium_binding, x_autodiff.head<14>(), x_autodiff.tail<9>());

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
  q_val.head<4>() << 1, 0, 0, 0;
  q_val.segment<3>(4) = X_WS0.translation();
  q_val.segment<4>(7) << 1, 0, 0, 0;
  q_val.tail<3>() = X_WS1.translation();

  prog_.AddConstraint(static_equilibrium_binding);

  Eigen::VectorXd x_init(prog_.num_vars());
  x_init.setZero();
  prog_.SetDecisionVariableValueInVector(q_vars_, q_val, &x_init);
  prog_.SetDecisionVariableValueInVector(
      contact_wrench_evaluators_and_lambda_[1].second,
      Eigen::Vector3d(0, 0, -spheres_->spheres()[0].inertia.get_mass() * 9.81),
      &x_init);
  prog_.SetDecisionVariableValueInVector(
      contact_wrench_evaluators_and_lambda_[2].second,
      Eigen::Vector3d(0, 0, -spheres_->spheres()[1].inertia.get_mass() * 9.81),
      &x_init);

  auto result = solvers::Solve(prog_, x_init);
  EXPECT_TRUE(result.is_success());

  // Given the solution, now manually check if the system is in static
  // equilibrium.
  spheres_double_->plant().SetPositions(
      spheres_double_->get_mutable_plant_context(),
      result.GetSolution(q_vars_));

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
      // contact between two spheres.
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
      // contact between the ground and the sphere 0.
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
      // contact between the ground and the sphere 1.
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
  // The solver's default tolerance is 1E-6 (with normalization). The
  // unnormalized tolerance is about 1E-5.
  const double tol = 1E-5;
  EXPECT_TRUE(
      CompareMatrices(sphere0_total_wrench, Vector6<double>::Zero(), tol));
  EXPECT_TRUE(
      CompareMatrices(sphere1_total_wrench, Vector6<double>::Zero(), tol));
}
}  // namespace
}  // namespace multibody
}  // namespace drake
