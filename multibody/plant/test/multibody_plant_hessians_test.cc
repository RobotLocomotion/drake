#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/kuka_iiwa_model_tests.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/curvilinear_joint.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rpy_floating_joint.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

using test::KukaIiwaModelTests;

namespace {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using math::RollPitchYawd;
using systems::Context;
using Matrix6Xd = Matrix6X<double>;

// The largest disagreement with autodiff observed over the cases below is
// about 6 ε.
constexpr double kTolerance = 64 * std::numeric_limits<double>::epsilon();

// Computes H𝑠_V_ABp_E by automatic differentiation of
// CalcJacobianSpatialVelocity() on an AutoDiffXd copy of `plant`, at the
// positions stored in `context`. The i-th derivative is seeded with q̇ = eᵢ for
// 𝑠 = q̇ and with q̇ = N(q)⋅eᵢ for 𝑠 = v, so that element i of the result is
// Dᵢ(J𝑠_V_ABp_E), the derivative of J𝑠_V_ABp_E along 𝑠 = eᵢ.
std::vector<Matrix6Xd> CalcHessianViaAutoDiff(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    JacobianWrtVariable with_respect_to, const Frame<double>& frame_B,
    const Vector3d& p_BoBp_B, const Frame<double>& frame_A,
    const Frame<double>& frame_E) {
  const int nq = plant.num_positions();
  const int n = with_respect_to == JacobianWrtVariable::kQDot
                    ? nq
                    : plant.num_velocities();
  const MatrixXd seed = with_respect_to == JacobianWrtVariable::kQDot
                            ? MatrixXd(MatrixXd::Identity(nq, nq))
                            : MatrixXd(plant.MakeVelocityToQDotMap(context));

  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  std::unique_ptr<Context<AutoDiffXd>> context_ad =
      plant_ad->CreateDefaultContext();
  context_ad->SetTimeStateAndParametersFrom(context);
  plant_ad->SetPositions(
      context_ad.get(),
      math::InitializeAutoDiff(plant.GetPositions(context), seed));

  MatrixX<AutoDiffXd> J_ad(6, n);
  plant_ad->CalcJacobianSpatialVelocity(
      *context_ad, with_respect_to, plant_ad->get_frame(frame_B.index()),
      p_BoBp_B.cast<AutoDiffXd>(), plant_ad->get_frame(frame_A.index()),
      plant_ad->get_frame(frame_E.index()), &J_ad);

  // The gradient is (6⋅n) x n, with the entries of J in column-major order.
  const MatrixXd gradient = math::ExtractGradient(J_ad, n);
  std::vector<Matrix6Xd> H(n);
  for (int i = 0; i < n; ++i) {
    H[i] = Eigen::Map<const MatrixXd>(gradient.col(i).data(), 6, n);
  }
  return H;
}

// Computes H𝑠_V_ABp_E with CalcHessianSpatialVelocity(), checks it against
// CalcHessianViaAutoDiff() and returns it.
std::vector<Matrix6Xd> CheckHessian(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    JacobianWrtVariable with_respect_to, const Frame<double>& frame_B,
    const Vector3d& p_BoBp_B, const Frame<double>& frame_A,
    const Frame<double>& frame_E, double tolerance = kTolerance) {
  const int n = with_respect_to == JacobianWrtVariable::kQDot
                    ? plant.num_positions()
                    : plant.num_velocities();
  std::vector<Matrix6Xd> H;
  plant.CalcHessianSpatialVelocity(context, with_respect_to, frame_B, p_BoBp_B,
                                   frame_A, frame_E, &H);
  const std::vector<Matrix6Xd> H_expected = CalcHessianViaAutoDiff(
      plant, context, with_respect_to, frame_B, p_BoBp_B, frame_A, frame_E);
  EXPECT_EQ(ssize(H), n);
  for (int i = 0; i < n; ++i) {
    EXPECT_EQ(H[i].rows(), 6);
    EXPECT_EQ(H[i].cols(), n);
    EXPECT_TRUE(CompareMatrices(H[i], H_expected[i], tolerance,
                                MatrixCompareType::relative))
        << "with_respect_to = " << static_cast<int>(with_respect_to)
        << ", frame_B = " << frame_B.name() << ", frame_A = " << frame_A.name()
        << ", frame_E = " << frame_E.name() << ", slice " << i;
  }
  return H;
}

const Vector3d kArbitraryPoint(0.1, -0.05, 0.02);

// The iiwa of this fixture floats on a quaternion joint. Frame H is a
// FixedOffsetFrame on the end effector.
TEST_F(KukaIiwaModelTests, FloatingIiwaWrtV) {
  SetArbitraryConfigurationAndMotion();
  const Frame<double>& W = plant_->world_frame();
  const Frame<double>& L0 = plant_->GetFrameByName("iiwa_link_0");
  const Frame<double>& L3 = plant_->GetFrameByName("iiwa_link_3");
  const Frame<double>& L5 = plant_->GetFrameByName("iiwa_link_5");
  for (const auto& [frame_A, frame_E] :
       std::vector<std::pair<const Frame<double>*, const Frame<double>*>>{
           {&W, &W},
           {&L3, &W},
           {&L3, &L5},
           {&L0, frame_H_},
           {&W, frame_H_},
           {&W, &L3}}) {
    CheckHessian(*plant_, *context_, JacobianWrtVariable::kV, *frame_H_,
                 kArbitraryPoint, *frame_A, *frame_E);
  }

  // A non-unit quaternion does not change the derivative along v.
  SetArbitraryConfigurationAndMotion(/* unit_quaternion = */ false);
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kV, *frame_H_,
               kArbitraryPoint, L3, L5);
}

// With 𝑠 = q̇, the floating joint is only a problem when it is on the path
// between frame_A and frame_B.
TEST_F(KukaIiwaModelTests, FloatingIiwaWrtQDot) {
  SetArbitraryConfigurationAndMotion();
  const Frame<double>& W = plant_->world_frame();
  const Frame<double>& L0 = plant_->GetFrameByName("iiwa_link_0");
  const Frame<double>& L3 = plant_->GetFrameByName("iiwa_link_3");
  std::vector<Matrix6Xd> H;
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kQDot,
                                         *frame_H_, kArbitraryPoint, W, W, &H),
      "CalcHessianSpatialVelocity\\(\\): with_respect_to = "
      "JacobianWrtVariable::kQDot is not yet supported when the kinematic "
      "path between frame_A 'world' and frame_B 'H' contains a "
      "quaternion_floating joint \\('.*'\\) whose generalized velocities are "
      "not the time derivatives of its generalized positions. Use "
      "JacobianWrtVariable::kV.");
  // Likewise when the floating joint is on frame_A's side of the path.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kQDot,
                                         W, kArbitraryPoint, *frame_H_, W, &H),
      ".*between frame_A 'H' and frame_B 'world'.*");
  // With both frames on the arm, the floating joint's 7 positions still get
  // slices (they move A and B together) but no columns.
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kQDot, *frame_H_,
               kArbitraryPoint, L0, W);
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kQDot, *frame_H_,
               kArbitraryPoint, L3, *frame_H_);
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kQDot, L0,
               kArbitraryPoint, *frame_H_, W);
  // Also with a non-unit quaternion, whose N⁺ differs from a unit one's.
  SetArbitraryConfigurationAndMotion(/* unit_quaternion = */ false);
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kQDot, *frame_H_,
               kArbitraryPoint, L0, W);
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kQDot, *frame_H_,
               kArbitraryPoint, L3, *frame_H_);
}

// The documented recipe for ∂(Jv_V_ABp_E)/∂qₖ from the 𝑠 = v result.
TEST_F(KukaIiwaModelTests, QGradientFromVResult) {
  SetArbitraryConfigurationAndMotion();
  const int nq = plant_->num_positions();
  const int nv = plant_->num_velocities();
  const Frame<double>& W = plant_->world_frame();
  const Frame<double>& L3 = plant_->GetFrameByName("iiwa_link_3");
  const int quaternion_start =
      plant_->GetBodyByName("iiwa_link_0").floating_positions_start();
  const MatrixXd Nplus(plant_->MakeQDotToVelocityMap(*context_));
  context_autodiff_->SetTimeStateAndParametersFrom(*context_);
  plant_autodiff_->SetPositions(
      context_autodiff_.get(),
      math::InitializeAutoDiff(plant_->GetPositions(*context_)));

  // With A = link 3 and E = H, the floating base moves A, B, and E together,
  // so the quaternion gradient is zero; with A = E = W it is not.
  for (const auto& [frame_A, frame_E] :
       std::vector<std::pair<const Frame<double>*, const Frame<double>*>>{
           {&L3, frame_H_}, {&W, &W}}) {
    std::vector<Matrix6Xd> Hv;
    plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                       *frame_H_, kArbitraryPoint, *frame_A,
                                       *frame_E, &Hv);

    // ∂(Jv_V_ABp_E)/∂qₖ by automatic differentiation.
    MatrixX<AutoDiffXd> Jv_autodiff(6, nv);
    plant_autodiff_->CalcJacobianSpatialVelocity(
        *context_autodiff_, JacobianWrtVariable::kV,
        plant_autodiff_->get_frame(frame_H_->index()),
        kArbitraryPoint.cast<AutoDiffXd>(),
        plant_autodiff_->get_frame(frame_A->index()),
        plant_autodiff_->get_frame(frame_E->index()), &Jv_autodiff);
    const MatrixXd gradient = math::ExtractGradient(Jv_autodiff, nq);
    if (frame_A == &W) {
      EXPECT_GT(gradient.middleCols(quaternion_start, 4).norm(), 0.1);
    }
    for (int k = 0; k < nq; ++k) {
      Matrix6Xd dJv_dqk = Matrix6Xd::Zero(6, nv);
      for (int i = 0; i < nv; ++i) dJv_dqk += Hv[i] * Nplus(i, k);
      EXPECT_TRUE(CompareMatrices(
          dJv_dqk, Eigen::Map<const MatrixXd>(gradient.col(k).data(), 6, nv),
          kTolerance, MatrixCompareType::relative));
    }
  }
}

// The result is consistent with the bias spatial acceleration, which is the
// time derivative of J𝑠_V_ABp_E (taken in frame A, so E = A here) times v.
TEST_F(KukaIiwaModelTests, BiasAccelerationConsistency) {
  SetArbitraryConfigurationAndMotion();
  const Frame<double>& W = plant_->world_frame();
  const Frame<double>& L3 = plant_->GetFrameByName("iiwa_link_3");
  const VectorXd v = plant_->GetVelocities(*context_);
  for (const Frame<double>* frame_A :
       std::vector<const Frame<double>*>{&W, &L3}) {
    std::vector<Matrix6Xd> H;
    plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                       *frame_H_, kArbitraryPoint, *frame_A,
                                       *frame_A, &H);
    Vector6<double> AvBias_AHp_A = Vector6<double>::Zero();
    for (int i = 0; i < ssize(H); ++i) AvBias_AHp_A += v[i] * (H[i] * v);
    const SpatialAcceleration<double> AvBias_AHp_A_expected =
        plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                            *frame_H_, kArbitraryPoint,
                                            *frame_A, *frame_A);
    EXPECT_TRUE(CompareMatrices(AvBias_AHp_A,
                                AvBias_AHp_A_expected.get_coeffs(),
                                8 * kTolerance, MatrixCompareType::relative));
  }
}

TEST_F(KukaIiwaModelTests, AutoDiffScalarType) {
  SetArbitraryConfigurationAndMotion();
  const Frame<double>& L3 = plant_->GetFrameByName("iiwa_link_3");
  std::vector<Matrix6Xd> H;
  plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                     *frame_H_, kArbitraryPoint, L3, *frame_H_,
                                     &H);

  context_autodiff_->SetTimeStateAndParametersFrom(*context_);
  std::vector<Matrix6X<AutoDiffXd>> H_autodiff;
  plant_autodiff_->CalcHessianSpatialVelocity(
      *context_autodiff_, JacobianWrtVariable::kV,
      plant_autodiff_->get_frame(frame_H_->index()),
      kArbitraryPoint.cast<AutoDiffXd>(),
      plant_autodiff_->get_frame(L3.index()),
      plant_autodiff_->get_frame(frame_H_->index()), &H_autodiff);
  ASSERT_EQ(ssize(H_autodiff), ssize(H));
  for (int i = 0; i < ssize(H); ++i) {
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(H_autodiff[i]), H[i],
                                kTolerance, MatrixCompareType::relative));
  }
}

// An iiwa welded to the world, plus a free body in its own tree so that the
// plant has more positions (14) than velocities (13).
class WeldedIiwaTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    free_body_ = &plant_->AddRigidBody("free_body",
                                       SpatialInertia<double>::MakeUnitary());
    Parser(plant_.get())
        .AddModelsFromUrl(
            "package://drake_models/iiwa_description/sdf/"
            "iiwa14_no_collision.sdf");
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetFrameByName("iiwa_link_0"));
    frame_B_ = &plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "B", plant_->GetBodyByName("iiwa_link_7"),
        RigidTransformd(RollPitchYawd(0.3, -0.2, 0.5),
                        Vector3d(0.1, 0.05, -0.08))));
    frame_Wo_ = &plant_->AddFrame(std::make_unique<FixedOffsetFrame<double>>(
        "Wo", plant_->world_frame(),
        RigidTransformd(RollPitchYawd(0.4, 0.1, -0.7), Vector3d(1, 2, 3))));
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();

    plant_->SetFreeBodyPose(context_.get(), *free_body_,
                            RigidTransformd(RollPitchYawd(0.1, 0.2, 0.3),
                                            Vector3d(0.5, -0.4, 0.7)));
    const VectorXd angles =
        (VectorXd(7) << 0.3, -0.7, 1.1, -0.9, 0.5, 1.3, -0.4).finished();
    for (int i = 0; i < 7; ++i) {
      plant_
          ->GetJointByName<RevoluteJoint>("iiwa_joint_" + std::to_string(i + 1))
          .set_angle(context_.get(), angles[i]);
    }
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
  const RigidBody<double>* free_body_{};
  const Frame<double>* frame_B_{};
  const Frame<double>* frame_Wo_{};
};

TEST_F(WeldedIiwaTest, AgainstAutoDiff) {
  EXPECT_EQ(plant_->num_positions(), 14);
  EXPECT_EQ(plant_->num_velocities(), 13);
  const Frame<double>& W = plant_->world_frame();
  const Frame<double>& L3 = plant_->GetFrameByName("iiwa_link_3");
  const Frame<double>& L5 = plant_->GetFrameByName("iiwa_link_5");
  for (const JacobianWrtVariable with_respect_to :
       {JacobianWrtVariable::kQDot, JacobianWrtVariable::kV}) {
    for (const auto& [frame_A, frame_E] :
         std::vector<std::pair<const Frame<double>*, const Frame<double>*>>{
             {&W, &W},
             {&L3, &W},
             {&L3, &L5},
             {&L3, frame_B_},
             {frame_B_, &W},
             {&W, frame_Wo_},
             {frame_Wo_, &L5}}) {
      const std::vector<Matrix6Xd> H =
          CheckHessian(*plant_, *context_, with_respect_to, *frame_B_,
                       kArbitraryPoint, *frame_A, *frame_E);
      // The free body is on no path, so its slices and columns are zero.
      const bool is_wrt_qdot = with_respect_to == JacobianWrtVariable::kQDot;
      const int free_start = is_wrt_qdot
                                 ? free_body_->floating_positions_start()
                                 : free_body_->floating_velocities_start_in_v();
      const int n_free = is_wrt_qdot ? 7 : 6;
      for (int i = 0; i < ssize(H); ++i) {
        EXPECT_TRUE(H[i].middleCols(free_start, n_free).isZero(0));
        if (free_start <= i && i < free_start + n_free) {
          EXPECT_TRUE(H[i].isZero(0));
        }
      }
    }
  }
}

// With A = E = W and q̇ = v, the translational part is the Hessian of the
// position of Bp, which is symmetric in the two derivative indices.
TEST_F(WeldedIiwaTest, TranslationalSymmetry) {
  const Frame<double>& W = plant_->world_frame();
  std::vector<Matrix6Xd> H;
  plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                     *frame_B_, kArbitraryPoint, W, W, &H);
  for (int i = 0; i < ssize(H); ++i) {
    for (int j = 0; j < ssize(H); ++j) {
      EXPECT_TRUE(CompareMatrices(H[i].col(j).tail<3>(), H[j].col(i).tail<3>(),
                                  kTolerance, MatrixCompareType::relative));
    }
  }
}

// With 𝑠 = q̇, the free body's quaternion joint is rejected only when it is on
// the path between frame_A and frame_B.
TEST_F(WeldedIiwaTest, WrtQDotWithFreeBody) {
  const Frame<double>& W = plant_->world_frame();
  const Frame<double>& F = free_body_->body_frame();
  std::vector<Matrix6Xd> H;
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kQDot,
                                         *frame_B_, kArbitraryPoint, F, W, &H),
      ".*between frame_A 'free_body' and frame_B 'B' contains a "
      "quaternion_floating joint.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kQDot,
                                         F, kArbitraryPoint, *frame_B_, W, &H),
      ".*not yet supported.*");
  // As the expressed-in frame, the free body is fine.
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kQDot, *frame_B_,
               kArbitraryPoint, W, F);
  // And so is everything with 𝑠 = v.
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kV, *frame_B_,
               kArbitraryPoint, F, W);
  CheckHessian(*plant_, *context_, JacobianWrtVariable::kV, F, kArbitraryPoint,
               *frame_B_, F);
}

TEST_F(WeldedIiwaTest, ZeroResults) {
  const Frame<double>& W = plant_->world_frame();
  const Frame<double>& L3 = plant_->GetFrameByName("iiwa_link_3");
  for (const JacobianWrtVariable with_respect_to :
       {JacobianWrtVariable::kQDot, JacobianWrtVariable::kV}) {
    const int n = with_respect_to == JacobianWrtVariable::kQDot
                      ? plant_->num_positions()
                      : plant_->num_velocities();
    // Frames A and B on the world or welded to it (E may move; it only
    // re-expresses the zero result).
    for (const Frame<double>* frame_B :
         {&W, frame_Wo_, &plant_->GetFrameByName("iiwa_link_0")}) {
      for (const Frame<double>* frame_A : {&W, frame_Wo_}) {
        std::vector<Matrix6Xd> H;
        plant_->CalcHessianSpatialVelocity(*context_, with_respect_to, *frame_B,
                                           kArbitraryPoint, *frame_A, L3, &H);
        ASSERT_EQ(ssize(H), n);
        for (const Matrix6Xd& slice : H) {
          EXPECT_EQ(slice.cols(), n);
          EXPECT_TRUE(slice.isZero(0));
        }
      }
    }
    // Frame A is frame B.
    std::vector<Matrix6Xd> H;
    plant_->CalcHessianSpatialVelocity(*context_, with_respect_to, *frame_B_,
                                       kArbitraryPoint, *frame_B_, L3, &H);
    ASSERT_EQ(ssize(H), n);
    for (const Matrix6Xd& slice : H) EXPECT_TRUE(slice.isZero(0));
  }
}

TEST_F(WeldedIiwaTest, OutputHandling) {
  const Frame<double>& W = plant_->world_frame();
  const Frame<double>& L3 = plant_->GetFrameByName("iiwa_link_3");
  DRAKE_EXPECT_THROWS_MESSAGE(plant_->CalcHessianSpatialVelocity(
                                  *context_, JacobianWrtVariable::kV, *frame_B_,
                                  kArbitraryPoint, L3, W, nullptr),
                              ".*Hs_V_ABp_E != nullptr.*");
  MultibodyPlant<double> other_plant(0.0);
  other_plant.Finalize();
  std::vector<Matrix6Xd> H;
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->CalcHessianSpatialVelocity(*other_plant.CreateDefaultContext(),
                                         JacobianWrtVariable::kV, *frame_B_,
                                         kArbitraryPoint, L3, W, &H),
      ".*was passed the Context of a system[\\s\\S]*");

  std::vector<Matrix6Xd> H_expected;
  plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                     *frame_B_, kArbitraryPoint, L3, W,
                                     &H_expected);
  // A wrongly sized output is resized.
  H.assign(3, Matrix6Xd::Ones(6, 2));
  plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                     *frame_B_, kArbitraryPoint, L3, W, &H);
  ASSERT_EQ(ssize(H), ssize(H_expected));
  for (int i = 0; i < ssize(H); ++i) EXPECT_EQ(H[i], H_expected[i]);
  // A reused output is fully overwritten.
  plant_->GetJointByName<RevoluteJoint>("iiwa_joint_4")
      .set_angle(context_.get(), 0.2);
  plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                     *frame_B_, kArbitraryPoint, L3, W,
                                     &H_expected);
  plant_->CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                     *frame_B_, kArbitraryPoint, L3, W, &H);
  for (int i = 0; i < ssize(H); ++i) EXPECT_EQ(H[i], H_expected[i]);
}

// A plant with every joint type except a quaternion floating joint (covered by
// the floating iiwa tests) on the paths between the test frames:
//
//   world ─J0─ base ─J1─ b1 ─J2─ b2 ─J3─ b3 ─J4─ b4 ─J5─ b5 ─J6─ b6 [B]
//     │                            └─J7─ c1 ─J8─ c2 [A]
//     │           └─J9─ e1 [E]
//     └─J10─ d1 [D]
//
// where J0 is an rpy floating joint (or a weld), J1 prismatic, J2 planar,
// J3 universal, J4 screw, J5 revolute, J6 a ball joint (or a revolute joint),
// J7 revolute, J8 prismatic, J9 curvilinear and J10 revolute. J2, J3 and J6
// connect to offset frames on their parent and child bodies, so that X_PF and
// X_MB are not identity. B, A, E and D are offset frames. With
// `weld_in_path`, a body b2w is welded to b2 and carries J3's parent frame.
struct BranchingTree {
  std::unique_ptr<MultibodyPlant<double>> plant;
  std::unique_ptr<Context<double>> context;
  const Frame<double>* frame_B{};
  const Frame<double>* frame_A{};
  const Frame<double>* frame_E{};
  const Frame<double>* frame_D{};
};

BranchingTree MakeBranchingTree(bool floating_base, bool weld_in_path = false,
                                bool fuse_welded_links = false) {
  BranchingTree tree;
  tree.plant = std::make_unique<MultibodyPlant<double>>(0.0);
  MultibodyPlant<double>& plant = *tree.plant;
  plant.SetFuseWeldedLinks(fuse_welded_links);
  const SpatialInertia<double> M = SpatialInertia<double>::MakeUnitary();
  auto add_body = [&](const std::string& name) -> const RigidBody<double>& {
    return plant.AddRigidBody(name, M);
  };
  auto add_offset_frame =
      [&](const std::string& name, const RigidBody<double>& body,
          const RigidTransformd& X_BF) -> const Frame<double>& {
    return plant.AddFrame(
        std::make_unique<FixedOffsetFrame<double>>(name, body, X_BF));
  };
  const RigidTransformd X1(RollPitchYawd(0.2, -0.3, 0.4),
                           Vector3d(0.1, 0.2, 0.3));
  const RigidTransformd X2(RollPitchYawd(-0.5, 0.1, 0.2),
                           Vector3d(-0.2, 0.1, 0.4));
  const RigidTransformd X3(RollPitchYawd(0.3, 0.3, -0.6),
                           Vector3d(0.3, -0.1, 0.2));

  const RigidBody<double>& base = add_body("base");
  if (floating_base) {
    plant.AddJoint<RpyFloatingJoint>("J0", plant.world_body(), X1, base, X2);
  } else {
    plant.AddJoint<WeldJoint>("J0", plant.world_body(), X1, base, X2, X3);
  }
  const RigidBody<double>& b1 = add_body("b1");
  plant.AddJoint<PrismaticJoint>("J1", base, std::nullopt, b1, std::nullopt,
                                 Vector3d(0, 0, 1));
  const RigidBody<double>& b2 = add_body("b2");
  plant.AddJoint(std::make_unique<PlanarJoint<double>>(
      "J2", add_offset_frame("F2", b1, X1), add_offset_frame("M2", b2, X2),
      Vector3d::Zero()));
  const RigidBody<double>& b3 = add_body("b3");
  const RigidBody<double>* parent_of_J3 = &b2;
  if (weld_in_path) {
    const RigidBody<double>& b2w = add_body("b2w");
    plant.AddJoint<WeldJoint>("J2w", b2, X3, b2w, std::nullopt, X1);
    parent_of_J3 = &b2w;
  }
  plant.AddJoint(std::make_unique<UniversalJoint<double>>(
      "J3", add_offset_frame("F3", *parent_of_J3, X3),
      add_offset_frame("M3", b3, X1)));
  const RigidBody<double>& b4 = add_body("b4");
  plant.AddJoint<ScrewJoint>("J4", b3, std::nullopt, b4, std::nullopt,
                             Vector3d(0, 1, 0), /* screw_pitch = */ 0.1,
                             /* damping = */ 0.0);
  const RigidBody<double>& b5 = add_body("b5");
  plant.AddJoint<RevoluteJoint>("J5", b4, X2, b5, std::nullopt,
                                Vector3d(1, 0, 0));
  const RigidBody<double>& b6 = add_body("b6");
  if (floating_base) {
    plant.AddJoint(std::make_unique<BallRpyJoint<double>>(
        "J6", add_offset_frame("F6", b5, X2), add_offset_frame("M6", b6, X3)));
  } else {
    plant.AddJoint(std::make_unique<RevoluteJoint<double>>(
        "J6", add_offset_frame("F6", b5, X2), add_offset_frame("M6", b6, X3),
        Vector3d(0, 0, 1)));
  }
  const RigidBody<double>& c1 = add_body("c1");
  plant.AddJoint<RevoluteJoint>("J7", b2, X3, c1, X1, Vector3d(0, 1, 0));
  const RigidBody<double>& c2 = add_body("c2");
  plant.AddJoint<PrismaticJoint>("J8", c1, std::nullopt, c2, X2,
                                 Vector3d(1, 0, 0));
  const RigidBody<double>& e1 = add_body("e1");
  // A "stadium" curve: two straight segments of length 1 joined by two
  // semicircles of radius 1.
  const trajectories::PiecewiseConstantCurvatureTrajectory<double> curve(
      std::vector<double>{0, M_PI, 1 + M_PI, 1 + 2 * M_PI, 2 + 2 * M_PI},
      std::vector<double>{1, 0, 1, 0}, Vector3d(1, -std::sqrt(2.0), 1),
      Vector3d(1, std::sqrt(2.0), 1), Vector3d(0.5, 0.25, 0.125),
      /* is_periodic = */ true);
  plant.AddJoint<CurvilinearJoint>("J9", base, X3, e1, std::nullopt, curve);
  const RigidBody<double>& d1 = add_body("d1");
  plant.AddJoint<RevoluteJoint>("J10", plant.world_body(), X2, d1, X3,
                                Vector3d(0, 0, 1));

  tree.frame_B = &add_offset_frame("B", b6, X1);
  tree.frame_A = &add_offset_frame("A", c2, X2);
  tree.frame_E = &add_offset_frame("E", e1, X3);
  tree.frame_D = &add_offset_frame("D", d1, X1);
  plant.Finalize();
  tree.context = plant.CreateDefaultContext();

  // An arbitrary configuration, with the rpy pitch angles well away from ±π/2.
  Context<double>* context = tree.context.get();
  if (floating_base) {
    plant.GetJointByName("J0").SetPositions(
        context,
        (Vector6<double>() << 0.3, -0.4, 0.5, 0.7, -0.2, 0.9).finished());
    plant.GetJointByName("J6").SetPositions(context, Vector3d(0.4, 0.3, -0.5));
  } else {
    plant.GetJointByName("J6").SetPositions(context, Vector1d(0.6));
  }
  plant.GetJointByName("J1").SetPositions(context, Vector1d(0.35));
  plant.GetJointByName("J2").SetPositions(context, Vector3d(0.2, -0.3, 0.7));
  plant.GetJointByName("J3").SetPositions(context, Vector2d(0.8, -0.6));
  plant.GetJointByName("J4").SetPositions(context, Vector1d(1.1));
  plant.GetJointByName("J5").SetPositions(context, Vector1d(-0.9));
  plant.GetJointByName("J7").SetPositions(context, Vector1d(0.45));
  plant.GetJointByName("J8").SetPositions(context, Vector1d(-0.25));
  plant.GetJointByName("J9").SetPositions(context, Vector1d(0.4 * M_PI));
  plant.GetJointByName("J10").SetPositions(context, Vector1d(0.55));
  return tree;
}

GTEST_TEST(BranchingTreeTest, FloatingBaseWrtV) {
  const BranchingTree tree = MakeBranchingTree(/* floating_base = */ true);
  const MultibodyPlant<double>& plant = *tree.plant;
  EXPECT_FALSE(plant.IsVelocityEqualToQDot());
  const Frame<double>& W = plant.world_frame();
  const Frame<double>& base = plant.GetFrameByName("base");
  const Frame<double>& b2 = plant.GetFrameByName("b2");
  const Frame<double>& c1 = plant.GetFrameByName("c1");
  for (const auto& [frame_A, frame_E] :
       std::vector<std::pair<const Frame<double>*, const Frame<double>*>>{
           {&W, &W},
           {&W, tree.frame_E},
           {tree.frame_A, &W},
           {tree.frame_A, tree.frame_E},
           {tree.frame_A, tree.frame_B},
           {&b2, tree.frame_A},
           {&base, &b2},
           {&c1, tree.frame_A},
           {tree.frame_E, tree.frame_A},
           {tree.frame_D, tree.frame_E}}) {
    CheckHessian(plant, *tree.context, JacobianWrtVariable::kV, *tree.frame_B,
                 kArbitraryPoint, *frame_A, *frame_E);
    // Also with the roles of A and B swapped.
    CheckHessian(plant, *tree.context, JacobianWrtVariable::kV, *frame_A,
                 kArbitraryPoint, *tree.frame_B, *frame_E);
  }
  // With 𝑠 = q̇, the ball joint between A and B is rejected, as is the
  // floating joint when it is between A and B ...
  std::vector<Matrix6Xd> H;
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CalcHessianSpatialVelocity(
          *tree.context, JacobianWrtVariable::kQDot, *tree.frame_B,
          kArbitraryPoint, *tree.frame_A, W, &H),
      ".*contains a ball_rpy joint \\('J6'\\).*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CalcHessianSpatialVelocity(
          *tree.context, JacobianWrtVariable::kQDot, *tree.frame_E,
          kArbitraryPoint, *tree.frame_D, W, &H),
      ".*contains a rpy_floating joint \\('J0'\\).*");
  // ... but not when it is upstream of both.
  CheckHessian(plant, *tree.context, JacobianWrtVariable::kQDot, *tree.frame_A,
               kArbitraryPoint, b2, *tree.frame_E);
}

GTEST_TEST(BranchingTreeTest, WeldInPathAndFusedLinks) {
  for (const bool floating_base : {true, false}) {
    for (const bool fuse_welded_links : {false, true}) {
      const BranchingTree tree = MakeBranchingTree(
          floating_base, /* weld_in_path = */ true, fuse_welded_links);
      const MultibodyPlant<double>& plant = *tree.plant;
      const Frame<double>& W = plant.world_frame();
      const Frame<double>& b4 = plant.GetFrameByName("b4");
      for (const JacobianWrtVariable with_respect_to :
           {JacobianWrtVariable::kQDot, JacobianWrtVariable::kV}) {
        // With a floating base the ball joint at B rules out 𝑠 = q̇.
        if (floating_base && with_respect_to == JacobianWrtVariable::kQDot) {
          continue;
        }
        for (const auto& [frame_A, frame_E] :
             std::vector<std::pair<const Frame<double>*, const Frame<double>*>>{
                 {&W, &W},
                 {tree.frame_A, tree.frame_E},
                 {&b4, tree.frame_B},
                 {tree.frame_D, &b4}}) {
          CheckHessian(plant, *tree.context, with_respect_to, *tree.frame_B,
                       kArbitraryPoint, *frame_A, *frame_E);
        }
      }
    }
  }
}

// A roll-pitch-yaw joint at gimbal lock has no N(q), but the Hessian along v
// is still well defined. The bias acceleration serves as the oracle, since the
// autodiff one needs N(q) too.
GTEST_TEST(BranchingTreeTest, GimbalLock) {
  const BranchingTree tree = MakeBranchingTree(/* floating_base = */ true);
  const MultibodyPlant<double>& plant = *tree.plant;
  Context<double>& context = *tree.context;
  plant.GetJointByName("J6").SetPositions(&context,
                                          Vector3d(0.4, M_PI / 2, -0.5));
  plant.SetVelocities(&context,
                      VectorXd::LinSpaced(plant.num_velocities(), -0.7, 0.9));
  const VectorXd v = plant.GetVelocities(context);
  std::vector<Matrix6Xd> H;
  EXPECT_NO_THROW(plant.CalcHessianSpatialVelocity(
      context, JacobianWrtVariable::kV, *tree.frame_B, kArbitraryPoint,
      *tree.frame_A, *tree.frame_A, &H));
  Vector6<double> AvBias = Vector6<double>::Zero();
  for (int i = 0; i < ssize(H); ++i) AvBias += v[i] * (H[i] * v);
  const SpatialAcceleration<double> AvBias_expected =
      plant.CalcBiasSpatialAcceleration(context, JacobianWrtVariable::kV,
                                        *tree.frame_B, kArbitraryPoint,
                                        *tree.frame_A, *tree.frame_A);
  EXPECT_TRUE(CompareMatrices(AvBias, AvBias_expected.get_coeffs(),
                              8 * kTolerance, MatrixCompareType::relative));

  // Likewise with AutoDiffXd positions that carry derivatives.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant);
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff =
      plant_autodiff->CreateDefaultContext();
  context_autodiff->SetTimeStateAndParametersFrom(context);
  plant_autodiff->SetPositions(
      context_autodiff.get(),
      math::InitializeAutoDiff(plant.GetPositions(context)));
  std::vector<Matrix6X<AutoDiffXd>> H_autodiff;
  EXPECT_NO_THROW(plant_autodiff->CalcHessianSpatialVelocity(
      *context_autodiff, JacobianWrtVariable::kV,
      plant_autodiff->get_frame(tree.frame_B->index()),
      kArbitraryPoint.cast<AutoDiffXd>(),
      plant_autodiff->get_frame(tree.frame_A->index()),
      plant_autodiff->get_frame(tree.frame_A->index()), &H_autodiff));
  ASSERT_EQ(ssize(H_autodiff), ssize(H));
  for (int i = 0; i < ssize(H); ++i) {
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(H_autodiff[i]), H[i],
                                kTolerance, MatrixCompareType::relative));
  }
}

// With free symbolic roll-pitch-yaw angles, N(q) cannot be checked for gimbal
// lock, so the Hessian along v must not need it.
GTEST_TEST(BranchingTreeTest, SymbolicScalarType) {
  const BranchingTree tree = MakeBranchingTree(/* floating_base = */ true);
  const MultibodyPlant<double>& plant = *tree.plant;
  std::unique_ptr<MultibodyPlant<symbolic::Expression>> plant_symbolic =
      systems::System<double>::ToSymbolic(plant);
  std::unique_ptr<Context<symbolic::Expression>> context_symbolic =
      plant_symbolic->CreateDefaultContext();
  context_symbolic->SetTimeStateAndParametersFrom(*tree.context);
  symbolic::Environment env;
  for (const char* name : {"J0", "J6"}) {
    const Joint<double>& joint = plant.GetJointByName(name);
    const VectorXd q = joint.GetPositions(*tree.context);
    VectorX<symbolic::Expression> q_symbolic(q.size());
    for (int i = 0; i < q.size(); ++i) {
      const symbolic::Variable q_i(std::string(name) + "_q" +
                                   std::to_string(i));
      env.insert(q_i, q[i]);
      q_symbolic[i] = q_i;
    }
    plant_symbolic->GetJointByName(name).SetPositions(context_symbolic.get(),
                                                      q_symbolic);
  }
  // J6 is between B and A; J0 is between E and D.
  for (const auto& [frame_B, frame_A] :
       std::vector<std::pair<const Frame<double>*, const Frame<double>*>>{
           {tree.frame_B, tree.frame_A}, {tree.frame_E, tree.frame_D}}) {
    std::vector<Matrix6Xd> H;
    plant.CalcHessianSpatialVelocity(*tree.context, JacobianWrtVariable::kV,
                                     *frame_B, kArbitraryPoint, *frame_A,
                                     plant.world_frame(), &H);
    std::vector<Matrix6X<symbolic::Expression>> H_symbolic;
    plant_symbolic->CalcHessianSpatialVelocity(
        *context_symbolic, JacobianWrtVariable::kV,
        plant_symbolic->get_frame(frame_B->index()),
        kArbitraryPoint.cast<symbolic::Expression>(),
        plant_symbolic->get_frame(frame_A->index()),
        plant_symbolic->world_frame(), &H_symbolic);
    ASSERT_EQ(ssize(H_symbolic), ssize(H));
    for (int i = 0; i < ssize(H); ++i) {
      EXPECT_TRUE(CompareMatrices(symbolic::Evaluate(H_symbolic[i], env), H[i],
                                  kTolerance, MatrixCompareType::relative));
    }
  }
}

GTEST_TEST(BranchingTreeTest, AutoDiffScalarType) {
  const BranchingTree tree = MakeBranchingTree(/* floating_base = */ true);
  const MultibodyPlant<double>& plant = *tree.plant;
  std::vector<Matrix6Xd> H;
  plant.CalcHessianSpatialVelocity(*tree.context, JacobianWrtVariable::kV,
                                   *tree.frame_B, kArbitraryPoint,
                                   *tree.frame_E, *tree.frame_A, &H);

  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant);
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff =
      plant_autodiff->CreateDefaultContext();
  context_autodiff->SetTimeStateAndParametersFrom(*tree.context);
  std::vector<Matrix6X<AutoDiffXd>> H_autodiff;
  plant_autodiff->CalcHessianSpatialVelocity(
      *context_autodiff, JacobianWrtVariable::kV,
      plant_autodiff->get_frame(tree.frame_B->index()),
      kArbitraryPoint.cast<AutoDiffXd>(),
      plant_autodiff->get_frame(tree.frame_E->index()),
      plant_autodiff->get_frame(tree.frame_A->index()), &H_autodiff);
  ASSERT_EQ(ssize(H_autodiff), ssize(H));
  for (int i = 0; i < ssize(H); ++i) {
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(H_autodiff[i]), H[i],
                                kTolerance, MatrixCompareType::relative));
  }
}

GTEST_TEST(BranchingTreeTest, WeldedBaseWrtQDotAndV) {
  const BranchingTree tree = MakeBranchingTree(/* floating_base = */ false);
  const MultibodyPlant<double>& plant = *tree.plant;
  EXPECT_TRUE(plant.IsVelocityEqualToQDot());
  const Frame<double>& W = plant.world_frame();
  const Frame<double>& b2 = plant.GetFrameByName("b2");
  const Frame<double>& c1 = plant.GetFrameByName("c1");
  for (const JacobianWrtVariable with_respect_to :
       {JacobianWrtVariable::kQDot, JacobianWrtVariable::kV}) {
    for (const auto& [frame_A, frame_E] :
         std::vector<std::pair<const Frame<double>*, const Frame<double>*>>{
             {&W, &W},
             {&W, tree.frame_E},
             {tree.frame_A, &W},
             {tree.frame_A, tree.frame_E},
             {tree.frame_A, tree.frame_B},
             {&b2, tree.frame_A},
             {&c1, tree.frame_A},
             {tree.frame_E, tree.frame_A},
             {tree.frame_D, &W},
             {tree.frame_D, tree.frame_E}}) {
      CheckHessian(plant, *tree.context, with_respect_to, *tree.frame_B,
                   kArbitraryPoint, *frame_A, *frame_E);
      CheckHessian(plant, *tree.context, with_respect_to, *frame_A,
                   kArbitraryPoint, *tree.frame_B, *frame_E);
    }
  }
}

GTEST_TEST(EmptyPlantTest, NoPositions) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>& body =
      plant.AddRigidBody("body", SpatialInertia<double>::MakeUnitary());
  plant.WeldFrames(plant.world_frame(), body.body_frame(),
                   RigidTransformd(Vector3d(1, 2, 3)));
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  for (const JacobianWrtVariable with_respect_to :
       {JacobianWrtVariable::kQDot, JacobianWrtVariable::kV}) {
    std::vector<Matrix6Xd> H(2);
    plant.CalcHessianSpatialVelocity(
        *context, with_respect_to, body.body_frame(), kArbitraryPoint,
        plant.world_frame(), body.body_frame(), &H);
    EXPECT_TRUE(H.empty());
  }
}

// A plant with only the world body, so B, A and E are all the world frame.
GTEST_TEST(EmptyPlantTest, WorldOnly) {
  MultibodyPlant<double> plant(0.0);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  const Frame<double>& world = plant.world_frame();
  for (const JacobianWrtVariable with_respect_to :
       {JacobianWrtVariable::kQDot, JacobianWrtVariable::kV}) {
    std::vector<Matrix6Xd> H(2);
    plant.CalcHessianSpatialVelocity(*context, with_respect_to, world,
                                     kArbitraryPoint, world, world, &H);
    EXPECT_TRUE(H.empty());
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
