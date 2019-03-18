#include "drake/multibody/optimization/static_equilibrium_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
const double kEps = std::numeric_limits<double>::epsilon();
class TwoFreeSpheresTest : public ::testing::Test {
 public:
  TwoFreeSpheresTest() {
    std::vector<SphereSpecification> spheres;
    spheres.emplace_back(0.1, 1E3, 0.8, 0.7);
    spheres.emplace_back(0.2, 1E3, 0.9, 0.7);
    spheres_ = std::make_unique<FreeSpheresAndBoxes<AutoDiffXd>>(
        spheres, std::vector<BoxSpecification>() /* no boxes */,
        CoulombFriction<double>(1, 0.8));
    spheres_double_ = std::make_unique<FreeSpheresAndBoxes<double>>(
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
    for (const auto& geometry_pair : collision_candidate_pairs) {
      auto wrench_evaluator =
          std::make_shared<ContactWrenchFromForceInWorldFrameEvaluator>(
              &plant, spheres_->get_mutable_plant_context(), geometry_pair);
      auto lambda_i = prog_.NewContinuousVariables(
          wrench_evaluator->num_lambda(), "lambda");
      contact_wrench_evaluators_and_lambda_.push_back(
          std::make_pair(wrench_evaluator, lambda_i));
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
    const std::vector<geometry::SignedDistancePair<double>>
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
          signed_distance_pair.p_ACa.cast<AutoDiffXd>(),
          spheres_->plant().world_frame(), spheres_->plant().world_frame(),
          &Jv_V_WCa);
      spheres_->plant().CalcJacobianSpatialVelocity(
          spheres_->plant_context(), JacobianWrtVariable::kV, frameB,
          signed_distance_pair.p_BCb.cast<AutoDiffXd>(),
          spheres_->plant().world_frame(), spheres_->plant().world_frame(),
          &Jv_V_WCb);

      AutoDiffVecXd F_AB_W(6);

      auto lambda_indices_in_all_lambda =
          static_equilibrium_binding.evaluator()
              ->contact_pair_to_wrench_evaluator()
              .at(std::make_pair(signed_distance_pair.id_A,
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
    EXPECT_TRUE(CompareMatrices(
        math::autoDiffToValueMatrix(y_autodiff),
        math::autoDiffToValueMatrix(y_autodiff_expected), 100 * kEps));
    EXPECT_TRUE(CompareMatrices(
        math::autoDiffToGradientMatrix(y_autodiff),
        math::autoDiffToGradientMatrix(y_autodiff_expected), 100 * kEps));
  }

 protected:
  std::unique_ptr<FreeSpheresAndBoxes<AutoDiffXd>> spheres_;
  std::unique_ptr<FreeSpheresAndBoxes<double>> spheres_double_;
  std::unique_ptr<solvers::Binding<StaticEquilibriumConstraint>>
      static_equilibrium_binding_;
  solvers::MathematicalProgram prog_;
  VectorX<symbolic::Variable> q_vars_;
  VectorX<symbolic::Variable> u_vars_{0};
  std::vector<std::pair<std::shared_ptr<ContactWrenchEvaluator>,
                        VectorX<symbolic::Variable>>>
      contact_wrench_evaluators_and_lambda_;
};

TEST_F(TwoFreeSpheresTest, Constructor) {
  const auto& plant = spheres_->plant();
  const auto static_equilibrium_binding = CreateStaticEquilibriumConstraint(
      &plant, spheres_->get_mutable_plant_context(),
      contact_wrench_evaluators_and_lambda_, q_vars_, u_vars_);
  // Test constructor of StaticEquilibriumConstraint
  EXPECT_EQ(static_equilibrium_binding.evaluator()->num_vars(),
            23 /* 14 for position, 9 for lambda */);
  EXPECT_EQ(static_equilibrium_binding.evaluator()->num_constraints(), 12);
  EXPECT_TRUE(
      CompareMatrices(static_equilibrium_binding.evaluator()->lower_bound(),
                      Eigen::VectorXd::Zero(12)));
  EXPECT_TRUE(
      CompareMatrices(static_equilibrium_binding.evaluator()->upper_bound(),
                      Eigen::VectorXd::Zero(12)));
}

TEST_F(TwoFreeSpheresTest, Eval) {
  const auto& plant = spheres_->plant();
  const auto static_equilibrium_binding = CreateStaticEquilibriumConstraint(
      &plant, spheres_->get_mutable_plant_context(),
      contact_wrench_evaluators_and_lambda_, q_vars_, u_vars_);
  math::RigidTransform<double> X_WS1, X_WS2;
  // Set the sphere pose X_WS1 and X_WS2 arbitrarily.
  X_WS1.set(math::RotationMatrix<double>(Eigen::AngleAxisd(
                M_PI_4, Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3))),
            Eigen::Vector3d(0.1, 0.2, 0.3));
  X_WS2.set(math::RotationMatrix<double>(Eigen::AngleAxisd(
                -0.1, Eigen::Vector3d(0.1, 0.2, 0.3).normalized())),
            Eigen::Vector3d(0.15, 0.25, 0.2));

  Eigen::VectorXd q_val(14);
  q_val.head<4>() = X_WS1.rotation().ToQuaternionAsVector4();
  q_val.segment<3>(4) = X_WS1.translation();
  q_val.segment<4>(7) = X_WS2.rotation().ToQuaternionAsVector4();
  q_val.tail<3>() = X_WS2.translation();
  Eigen::VectorXd lambda_val(9);
  // Test lambda = 0.
  lambda_val.setZero();
  Eigen::VectorXd x_val(23);
  x_val << q_val, lambda_val;
  auto x_autodiff = math::initializeAutoDiff(x_val);
  CheckStaticEquilibriumConstraintEval(
      static_equilibrium_binding, x_autodiff.head<14>(), x_autodiff.tail<9>());
  // Test lambda != 0
  lambda_val << 2, 3, 4, 5, 6, 7, 8, 9, 10;
  x_val << q_val, lambda_val;
  x_autodiff = math::initializeAutoDiff(x_val);
  CheckStaticEquilibriumConstraintEval(
      static_equilibrium_binding, x_autodiff.head<14>(), x_autodiff.tail<9>());

  // Solves the optimization problem
  // Note that we don't impose the complementarity constraint signed_distance *
  // contact_force = 0, or the friction cone constraint.
  // First set the spheres position, such that they are both on the ground, and
  // touching each other.
  X_WS1.set_translation(Eigen::Vector3d(0, 0, spheres_->spheres()[0].radius));
  X_WS2.set_translation(
      Eigen::Vector3d(std::sqrt(4 * spheres_->spheres()[0].radius *
                                spheres_->spheres()[1].radius),
                      0, spheres_->spheres()[1].radius));
  q_val.segment<3>(4) = X_WS1.translation();
  q_val.tail<3>() = X_WS2.translation();

  prog_.AddBoundingBoxConstraint(q_val, q_val, q_vars_);
  prog_.AddConstraint(static_equilibrium_binding);

  Eigen::VectorXd x_init(prog_.num_vars());
  prog_.SetDecisionVariableValueInVector(q_vars_, q_val, &x_init);

  auto result = solvers::Solve(prog_, x_init);
  EXPECT_TRUE(result.is_success());

  // TODO(hongkai.dai): Now check if the system is in static equilibrium.
  spheres_double_->plant().SetPositions(
      spheres_double_->get_mutable_plant_context(),
      result.GetSolution(q_vars_));
}
}  // namespace multibody
}  // namespace drake
