#include "drake/multibody/rational/rational_forward_kinematics.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic/rational_function.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/rational/test/rational_forward_kinematics_test_utilities.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"

namespace drake {
namespace multibody {
namespace {
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::BodyIndex;
using multibody::ModelInstanceIndex;
using symbolic::Polynomial;
using symbolic::RationalFunction;

void CheckBodyKinematics(const RationalForwardKinematics& dut,
                         const Eigen::Ref<const Eigen::VectorXd>& q_val,
                         const Eigen::Ref<const Eigen::VectorXd>& q_star_val,
                         const Eigen::Ref<const Eigen::VectorXd>& s_val,
                         BodyIndex expressed_body_index) {
  DRAKE_DEMAND(s_val.rows() == dut.s().rows());
  auto context = dut.plant().CreateDefaultContext();

  dut.plant().SetPositions(context.get(), q_val);

  std::vector<math::RigidTransformd> X_WB_expected;

  const internal::MultibodyTree<double>& tree =
      internal::GetInternalTree(dut.plant());
  tree.CalcAllBodyPosesInWorld(*context, &X_WB_expected);

  symbolic::Environment env;
  for (int i = 0; i < s_val.rows(); ++i) {
    env.insert(dut.s()(i), s_val(i));
  }

  const double kTol{1E-12};
  for (int i = 0; i < dut.plant().num_bodies(); ++i) {
    const math::RigidTransformd X_AB_expected =
        X_WB_expected[expressed_body_index].inverse() * X_WB_expected[i];
    // Now check CalcBodyPoseAsMultilinearPolynomial, namely to compute a
    // single body pose.
    const RationalForwardKinematics::Pose<symbolic::Polynomial> pose_poly_i =
        dut.CalcBodyPoseAsMultilinearPolynomial(q_star_val, BodyIndex(i),
                                                expressed_body_index);
    Eigen::Matrix3d R_AB_i;
    Eigen::Vector3d p_AB_i;
    for (int m = 0; m < 3; ++m) {
      for (int n = 0; n < 3; ++n) {
        const symbolic::RationalFunction R_AB_i_mn_rational =
            dut.ConvertMultilinearPolynomialToRationalFunction(
                pose_poly_i.rotation(m, n));
        R_AB_i(m, n) = R_AB_i_mn_rational.Evaluate(env);
      }
      const symbolic::RationalFunction p_AB_i_m_rational =
          dut.ConvertMultilinearPolynomialToRationalFunction(
              pose_poly_i.position(m));
      p_AB_i(m) = p_AB_i_m_rational.Evaluate(env);
    }
    EXPECT_TRUE(
        CompareMatrices(R_AB_i, X_AB_expected.rotation().matrix(), kTol));
    EXPECT_TRUE(CompareMatrices(p_AB_i, X_AB_expected.translation(), kTol));
  }
}

TEST_F(FinalizedIiwaTest, CalcBodyPoses) {
  RationalForwardKinematics dut(iiwa_.get());
  constexpr int kNumJoints = 7;
  EXPECT_EQ(dut.s().rows(), kNumJoints);

  // q_val = 0 and q* = 0.
  CheckBodyKinematics(dut, Eigen::VectorXd::Zero(kNumJoints),
                      Eigen::VectorXd::Zero(kNumJoints),
                      Eigen::VectorXd::Zero(kNumJoints), world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckBodyKinematics(dut, Eigen::VectorXd::Zero(kNumJoints),
                        Eigen::VectorXd::Zero(kNumJoints),
                        Eigen::VectorXd::Zero(kNumJoints), iiwa_link_[i]);
  }

  // Non-zero q_val and zero q_star_val.
  Eigen::VectorXd q_val(kNumJoints);
  // arbitrary value
  q_val << 0.2, 0.3, 0.5, -0.1, 1.2, 2.3, -0.5;
  Eigen::VectorXd s_val = (q_val / 2).array().tan().matrix();
  CheckBodyKinematics(dut, q_val, Eigen::VectorXd::Zero(kNumJoints), s_val,
                      world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckBodyKinematics(dut, q_val, Eigen::VectorXd::Zero(kNumJoints), s_val,
                        iiwa_link_[i]);
  }

  // Non-zero q_val and non-zero q_star_val.
  Eigen::VectorXd q_star_val(kNumJoints);
  q_star_val << 1.2, -0.4, 0.3, -0.5, 0.4, 1, 0.2;
  s_val = ((q_val - q_star_val) / 2).array().tan().matrix();
  CheckBodyKinematics(dut, q_val, q_star_val, s_val, world_);
  // Compute the pose in the iiwa_link[i]'s frame.
  for (int i = 0; i < 8; ++i) {
    CheckBodyKinematics(dut, q_val, q_star_val, s_val, iiwa_link_[i]);
  }
}

GTEST_TEST(RationalForwardKinematicsTest, CalcBodyPosesForDualArmIiwa) {
  RigidTransformd X_WL{};
  RigidTransformd X_WR{
      RotationMatrixd(Eigen::AngleAxisd(0.2 * M_PI, Eigen::Vector3d::UnitZ())),
      {0.2, 0.8, 0.1}};
  ModelInstanceIndex left_iiwa_instance, right_iiwa_instance;
  auto iiwa_plant =
      ConstructDualArmIiwaPlant("iiwa14_no_collision.sdf", X_WL, X_WR,
                                &left_iiwa_instance, &right_iiwa_instance);

  RationalForwardKinematics dut(iiwa_plant.get());
  EXPECT_EQ(dut.s().size(), 14);

  Eigen::VectorXd q_star_val = Eigen::VectorXd::Zero(14);
  Eigen::VectorXd q_val = Eigen::VectorXd::Zero(14);

  auto calc_s_val = [&](const Eigen::VectorXd& q_left,
                        const Eigen::VectorXd& q_right,
                        const Eigen::VectorXd& q_left_star,
                        const Eigen::VectorXd& q_right_star) {
    auto context = iiwa_plant->CreateDefaultContext();
    iiwa_plant->SetPositions(context.get(), left_iiwa_instance, q_left);
    iiwa_plant->SetPositions(context.get(), right_iiwa_instance, q_right);
    q_val = iiwa_plant->GetPositions(*context);
    iiwa_plant->SetPositions(context.get(), left_iiwa_instance, q_left_star);
    iiwa_plant->SetPositions(context.get(), right_iiwa_instance, q_right_star);
    q_star_val = iiwa_plant->GetPositions(*context);

    Eigen::VectorXd s_val = dut.ComputeSValue(q_val, q_star_val);
    return s_val;
  };

  const BodyIndex world_index = iiwa_plant->world_body().index();
  Eigen::VectorXd s_val = Eigen::VectorXd::Zero(14);
  CheckBodyKinematics(dut, q_val, q_star_val, s_val, world_index);

  Eigen::VectorXd q_left(7);
  Eigen::VectorXd q_right(7);
  Eigen::VectorXd q_left_star(7);
  Eigen::VectorXd q_right_star(7);
  q_left_star.setZero();
  q_right_star.setZero();
  q_left << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  q_right << -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7;
  s_val = calc_s_val(q_left, q_right, q_left_star, q_right_star);
  CheckBodyKinematics(dut, q_val, q_star_val, s_val, world_index);

  q_left_star << -0.2, 1.2, 0.3, 0.4, -2.1, 2.2, 2.3;
  q_right_star << 0.1, 0.2, 0.5, 1.1, -0.3, -0.2, 2.1;
  s_val = calc_s_val(q_left, q_right, q_left_star, q_right_star);
  CheckBodyKinematics(dut, q_val, q_star_val, s_val, world_index);
}

// Construct a plant with weld, revolute and prismatic joints.
// The kinematic structure is
// revolute - body7 - weld - body8
//   |
// world - revolute - body0 - prismatic - body1 - weld - body2
//                       |
//                    revolute
//                       |
// body4 - prismatic - body3 - revolute - body5 - revolute - body6
class KinematicTreeTest : public testing::Test {
 public:
  KinematicTreeTest() : plant_{new MultibodyPlant<double>{0.}} {
    AddBodyWithJoint<RevoluteJoint>(
        "body0", world_index(), "joint0",
        math::RigidTransformd(math::RollPitchYawd(0.2, 0.1, 0.5),
                              Eigen::Vector3d(0.2, 0.5, 0.1)),
        math::RigidTransformd(math::RollPitchYawd(-0.5, 0.1, 0.4),
                              Eigen::Vector3d(0.1, 0.3, 0.2)),
        Eigen::Vector3d(0, 1 / std::sqrt(2), 1 / std::sqrt(2)));
    AddBodyWithJoint<PrismaticJoint>(
        "body1", body_indices_[0], "joint1",
        math::RigidTransformd(math::RollPitchYawd(0.1, 0.2, 0.3),
                              Eigen::Vector3d(0.2, 0.4, -0.1)),
        math::RigidTransformd(math::RollPitchYawd(-0.1, 0.5, 0.3),
                              Eigen::Vector3d(0.2, 0.5, -0.1)),
        Eigen::Vector3d(1.0 / 3, 2.0 / 3, -2. / 3));
    AddBodyWithJoint<WeldJoint>(
        "body2", body_indices_[1], "joint2",
        math::RigidTransformd(math::RollPitchYawd(0.2, -0.1, 0.5),
                              Eigen::Vector3d(0.4, -0.1, 0.2)),
        math::RigidTransformd(math::RollPitchYawd(-0.1, 0.5, 0.3),
                              Eigen::Vector3d(0.2, 0.4, 0.3)),
        math::RigidTransformd(math::RollPitchYawd(0.5, 0.3, -0.2),
                              Eigen::Vector3d(0.2, 0.1, 0.4)));
    AddBodyWithJoint<RevoluteJoint>(
        "body3", body_indices_[0], "joint3",
        math::RigidTransformd(math::RollPitchYawd(0.2, -0.5, 0.3),
                              Eigen::Vector3d(0.2, -0.1, 1.5)),
        math::RigidTransformd(math::RollPitchYawd(0.4, 0.2, 0.5),
                              Eigen::Vector3d(0.4, -0.2, 1.1)),
        Eigen::Vector3d::UnitX());
    AddBodyWithJoint<PrismaticJoint>(
        "body4", body_indices_[3], "joint4",
        math::RigidTransformd(math::RollPitchYawd(0.2, 0.1, 0.5),
                              Eigen::Vector3d(0.5, -0.3, 1.1)),
        math::RigidTransformd(math::RollPitchYawd(0.2, -0.1, 0.5),
                              Eigen::Vector3d(0, 0.1, -0.2)),
        Eigen::Vector3d::UnitY());
    AddBodyWithJoint<RevoluteJoint>(
        "body5", body_indices_[3], "joint5",
        math::RigidTransformd(math::RollPitchYawd(0.1, 0.4, 0.5),
                              Eigen::Vector3d(0.2, 0.4, 0.1)),
        math::RigidTransformd(math::RollPitchYawd(0.3, 0.1, -0.5),
                              Eigen::Vector3d(0.2, 0.5, -0.1)),
        Eigen::Vector3d::UnitX());
    AddBodyWithJoint<RevoluteJoint>(
        "body6", body_indices_[5], "joint6",
        math::RigidTransformd(math::RollPitchYawd(1.1, 0.4, -0.5),
                              Eigen::Vector3d(0.4, -0.4, 0.1)),
        math::RigidTransformd(math::RollPitchYawd(0.7, -0.1, -0.5),
                              Eigen::Vector3d(0.2, 0.5, -0.1)),
        Eigen::Vector3d::UnitY());
    AddBodyWithJoint<RevoluteJoint>(
        "body7", world_index(), "joint7",
        math::RigidTransformd(math::RollPitchYawd(0.1, -0.4, -0.5),
                              Eigen::Vector3d(0.2, -0.5, 0.1)),
        math::RigidTransformd(math::RollPitchYawd(0.1, -0.3, -0.5),
                              Eigen::Vector3d(0.2, 0.5, -1.1)),
        Eigen::Vector3d::UnitY());
    AddBodyWithJoint<WeldJoint>(
        "body8", body_indices_[7], "joint8",
        math::RigidTransformd(math::RollPitchYawd(0.2, -0.4, -0.5),
                              Eigen::Vector3d(0.2, 0.5, 0.1)),
        math::RigidTransformd(math::RollPitchYawd(0.3, -0.1, -0.5),
                              Eigen::Vector3d(0.2, 0.5, -1.1)),
        math::RigidTransformd(math::RollPitchYawd(0.1, -1.1, -0.3),
                              Eigen::Vector3d(0.2, 0.3, -1.2)));
    plant_->Finalize();
  }

 protected:
  template <template <typename> class JointType, typename... Args>
  void AddBodyWithJoint(const std::string& body_name,
                        const BodyIndex parent_index,
                        const std::string& joint_name,
                        const std::optional<math::RigidTransform<double>>& X_PF,
                        const std::optional<math::RigidTransform<double>>& X_BM,
                        Args&&... args) {
    const SpatialInertia<double> spatial_inertia(
        1, Eigen::Vector3d::Zero(),
        UnitInertia<double>(0.01, 0.01, 0.01, 0, 0, 0));
    body_indices_.push_back(
        plant_->AddRigidBody(body_name, spatial_inertia).index());
    plant_->AddJoint<JointType>(joint_name, plant_->get_body(parent_index),
                                X_PF, plant_->get_body(body_indices_.back()),
                                X_BM, std::forward<Args>(args)...);
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::vector<BodyIndex> body_indices_;
};

TEST_F(KinematicTreeTest, CalcBodyPoses) {
  RationalForwardKinematics dut(plant_.get());
  EXPECT_EQ(dut.s().rows(), plant_->num_positions());

  // q_val = 0 and q* = 0.
  CheckBodyKinematics(dut, Eigen::VectorXd::Zero(plant_->num_positions()),
                      Eigen::VectorXd::Zero(plant_->num_positions()),
                      Eigen::VectorXd::Zero(plant_->num_positions()),
                      world_index());
  // q_val = 0, q* = 0, express pose in body1.
  CheckBodyKinematics(dut, Eigen::VectorXd::Zero(plant_->num_positions()),
                      Eigen::VectorXd::Zero(plant_->num_positions()),
                      Eigen::VectorXd::Zero(plant_->num_positions()),
                      body_indices_[1]);

  // q_val != 0, q* != 0, express pose in body 2.
  Eigen::VectorXd q_val(plant_->num_positions());
  Eigen::VectorXd q_star_val(plant_->num_positions());
  for (int i = 0; i < q_val.rows(); ++i) {
    q_val(i) = 0.003 * i * (i + 1) - 0.1 * i;
    q_star_val(i) = 0.001 * i * (i - 1) + 0.2 * i;
  }
  Eigen::VectorXd s_val = dut.ComputeSValue(q_val, q_star_val);
  CheckBodyKinematics(dut, q_val, q_star_val, s_val, body_indices_[2]);
}
}  // namespace
}  // namespace multibody
}  // namespace drake
