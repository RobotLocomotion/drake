#include "drake/perception/estimators/dev/articulated_icp.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/multibody/rigid_body_plant/viewer_draw_translator.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/perception/estimators/dev/test/test_util.h"
#include "drake/solvers/mathematical_program.h"

using std::make_shared;
using std::pair;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;
using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Isometry3d;
using ::testing::TestWithParam;

namespace drake {
namespace perception {
namespace estimators {
namespace {

using solvers::MathematicalProgram;
using systems::BasicVector;
using systems::ViewerDrawTranslator;

// TODO(eric.cousineau): Move to proper utility.
class ArticulatedIcpVisualizer : public PointCloudVisualizer {
 public:
  explicit ArticulatedIcpVisualizer(const Scene* scene)
      : scene_(scene) {
    Init();
  }
  void Init() {
    const lcmt_viewer_load_robot load_msg(
        (multibody::CreateLoadRobotMessage<double>(scene_->tree())));
    vector<uint8_t> bytes(load_msg.getEncodedSize());
    load_msg.encode(bytes.data(), 0, bytes.size());
    lcm().Publish("DRAKE_VIEWER_LOAD_ROBOT", bytes.data(), bytes.size());
  }
  void PublishScene(const SceneState& scene_state) {
    const ViewerDrawTranslator draw_msg_tx(scene_->tree());
    drake::lcmt_viewer_draw draw_msg{};
    vector<uint8_t> bytes;
    const int num_q = scene_->tree().get_num_positions();
    const int num_v = scene_->tree().get_num_velocities();
    const int num_x = num_q + num_v;
    BasicVector<double> x(num_x);
    auto xb = x.get_mutable_value();
    xb.setZero();
    xb.head(num_q) = scene_state.q();
    draw_msg_tx.Serialize(0, x, &bytes);
    lcm().Publish("DRAKE_VIEWER_DRAW", bytes.data(), bytes.size());
  }

 private:
  const Scene* scene_;
};

class ArticulatedIcpTest : public TestWithParam<ObjectTestType> {
 protected:
  void SetUp() override {
    const ObjectTestType type = GetParam();
    ObjectTestSetup setup;
    GetObjectTestSetup(type, &setup);

    // Create a formulation for a simple floating-base target
    string file_path = FindResourceOrThrow(setup.urdf_file);
    // TODO(eric.cousineau): Use kQuaternion.
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame{nullptr};
    mutable_tree_ = new RigidBodyTreed();
    parsers::urdf::AddModelInstanceFromUrdfFile(file_path, floating_base_type,
                                                weld_frame, mutable_tree_);

    mutable_tree_->compile();
    tree_.reset(mutable_tree_);
    tree_cache_.reset(new KinematicsCached(tree_->CreateKinematicsCache()));

    const Vector3d obj_xyz = setup.X_WB.translation();
    // TODO(eric.cousineau): change X_WB.rotation() to X_WB.linear() once #7035
    // is resolved.
    const Vector3d obj_rpy = math::rotmat2rpy(setup.X_WB.rotation());
    const int nq = tree_->get_num_positions();

    // Perturbation for initializing local ICP.
    q_perturb_.resize(nq);

    // Set additional parameters
    switch (type) {
      case kSimpleCuboid: {
        zero_cost_tolerance_ = 1e-4;
        q_diff_norm_min_ = 0.01;
        q_perturb_ << 0.3, 0.25, 0.25, kPi / 8, kPi / 12, kPi / 10;
        break;
      }
      case kBlueFunnelScan: {
        // Due to mismatch between scan and measured point cloud.
        zero_cost_tolerance_ = 0.2;
        // Use smaller perturbation. Convergence is very sensitive to initial
        // condition.
        q_perturb_ << 0.03, 0.02, 0.02, kPi / 16, kPi / 24, kPi / 20;
        q_diff_norm_min_ = 0.25;
        break;
      }
    }

    ASSERT_EQ(6, nq);
    q_init_.resize(nq);
    q_init_ << obj_xyz, obj_rpy;

    // Add fixed camera frame.
    // NOTE: At present, the camera pose does not influence anything given
    // that it is fixed-base.
    Vector3d camera_xyz(-2, 0, 0.1);
    camera_xyz += obj_xyz;
    Vector3d camera_rpy(0, 0, 0);  // degrees
    auto* world_body = &mutable_tree_->world();
    shared_ptr<RigidBodyFramed> camera_frame;
    camera_frame.reset(
        new RigidBodyFramed("camera", world_body, camera_xyz,
                            camera_rpy * kPi / 180));
    mutable_tree_->addFrame(camera_frame);

    // Create the scene.
    scene_.reset(new Scene(tree_.get(), 0, camera_frame->get_frame_index()));

    // Compute the initial body correspondences.
    ComputeBodyCorrespondenceInfluences(*scene_, &influences_);

    // Get synthetic measured point cloud.
    points_ = setup.X_WB * setup.points_B;

    // Create visualizer.
    vis_.reset(new ArticulatedIcpVisualizer(scene_.get()));
  }

 protected:
  RigidBodyTreed* mutable_tree_;
  unique_ptr<KinematicsCached> tree_cache_;
  shared_ptr<const RigidBodyTreed> tree_;
  VectorXd q_init_;
  VectorXd q_perturb_;
  unique_ptr<Scene> scene_;
  ArticulatedBodyInfluences influences_;
  Matrix3Xd points_;
  double zero_cost_tolerance_{};
  double q_diff_norm_min_ = 0.01;
  unique_ptr<ArticulatedIcpVisualizer> vis_;
};

TEST_P(ArticulatedIcpTest, PositiveReturnsZeroCost) {
  // Start box at the given state, ensure that the cost returned is near zero.
  VectorXd q = q_init_;
  SceneState scene_state(scene_.get());
  scene_state.Update(q);

  // Get correspondences
  ArticulatedPointCorrespondences correspondence;
  ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
  // Compute error
  ArticulatedIcpErrorNormCost cost(scene_.get());
  ComputeCost(scene_state, correspondence, &cost);

  // The error should be near zero for points on the surface.
  EXPECT_NEAR(0, cost.cost(), zero_cost_tolerance_);
  // Ensure that we have the same cost from the linearized L2 norm version.
  ArticulatedIcpLinearizedNormCost lin_cost(scene_.get());
  ComputeCost(scene_state, correspondence, &lin_cost);
  EXPECT_EQ(cost.cost(), lin_cost.cost());
}

TEST_P(ArticulatedIcpTest, PositiveReturnsIncreasingCost) {
  // Start the object at the given state, ensure that the cost increases as we
  // translate away from the object given this simple scene (where the object
  // is non-adversarial).
  const bool publish_debug = false;
  double prev_cost = 0;
  SceneState scene_state(scene_.get());
  VectorXd q = q_init_;
  for (int i = 0; i < 5; ++i) {
    scene_state.Update(q);
    // Get correspondences
    ArticulatedPointCorrespondences correspondence;
    ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
    // Compute error
    ArticulatedIcpErrorNormCost cost(scene_.get());
    ComputeCost(scene_state, correspondence, &cost);
    // Ensure cost is increasing.
    EXPECT_GT(cost.cost(), prev_cost);
    prev_cost = cost.cost();
    // Ensure that we have the same cost from the linearized L2 norm version.
    ArticulatedIcpLinearizedNormCost lin_cost(scene_.get());
    ComputeCost(scene_state, correspondence, &lin_cost);
    EXPECT_EQ(cost.cost(), lin_cost.cost());

    // Translate the object 10cm along the x-axis after each iteration.
    q(0) += 0.1;

    if (publish_debug) {
      vis_->PublishScene(scene_state);
      vis_->PublishCloud(points_);
      drake::log()->info("i = {}", i);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

shared_ptr<solvers::QuadraticCost> MakeZeroQuadraticCost(int num_var) {
  return std::make_shared<solvers::QuadraticCost>(
      Eigen::MatrixXd::Zero(num_var, num_var),
      Eigen::VectorXd::Zero(num_var), 0);
}

shared_ptr<solvers::QuadraticCost> MakeConditioningCost(int num_var,
                                                        double value) {
  return std::make_shared<solvers::QuadraticCost>(
      value * Eigen::MatrixXd::Identity(num_var, num_var),
      Eigen::VectorXd::Zero(num_var), 0);
}

TEST_P(ArticulatedIcpTest, PositiveReturnsConvergenceTest) {
  // Test the number of iterations for the ICP to converge.
  const int num_q = tree_->get_num_positions();
  MathematicalProgram prog;
  const auto q_var = prog.NewContinuousVariables(num_q, "q");

  // Add cost for accumulating positive returns.
  auto qp_cost = MakeZeroQuadraticCost(num_q);
  prog.AddCost(qp_cost, q_var);
  // Add conditioning value for positive semi-definiteness.
  const double psd_cond = 1e-5;
  prog.AddCost(MakeConditioningCost(num_q, psd_cond), q_var);

  VectorXd q = q_init_ + q_perturb_;
  SceneState scene_state(scene_.get());

  const int iter_max = 15;
  const bool publish_debug = false;

  int iter = 0;
  ArticulatedPointCorrespondences correspondence;
  ArticulatedIcpLinearizedNormCost icp_cost(scene_.get());

  if (publish_debug) {
    SceneState debug_state(scene_.get());
    debug_state.Update(q);
    vis_->PublishScene(debug_state);
    vis_->PublishCloud(points_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  while (true) {
    // Update formulation.
    scene_state.Update(q);

    correspondence.clear();
    ComputeCorrespondences(scene_state, influences_, points_, &correspondence);
    ComputeCost(scene_state, correspondence, &icp_cost);
    icp_cost.UpdateCost(qp_cost.get());

    // Solve.
    prog.SetInitialGuess(q_var, q);
    auto result = prog.Solve();
    ASSERT_EQ(solvers::kSolutionFound, result);

    const double q_diff_norm = (q - q_init_).norm();
    if (q_diff_norm < q_diff_norm_min_) {
      break;
    }

    // Update initial guess.
    q = prog.GetSolution(q_var);
    ++iter;
    ASSERT_TRUE(iter < iter_max);

    if (publish_debug) {
      SceneState debug_state(scene_.get());
      debug_state.Update(q);
      vis_->PublishScene(debug_state);
      vis_->PublishCloud(points_);
      drake::log()->info("iter = {}, |q_diff| = {}", iter, q_diff_norm);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  drake::log()->info("Took {} iterations for acceptable convergence", iter);
}

// Instantiate parameterized test cases.
// TODO(eric.cousineau): Clearly parameterize failures for blue funnel case.
INSTANTIATE_TEST_CASE_P(test, ArticulatedIcpTest, ObjectTestTypes);

}  // namespace
}  // namespace estimators
}  // namespace perception
}  // namespace drake
