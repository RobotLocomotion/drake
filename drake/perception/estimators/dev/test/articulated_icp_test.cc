#include "drake/perception/estimators/dev/articulated_icp.h"

#include <bot_core/pointcloud_t.hpp>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmtypes/drake/lcmt_viewer_draw.hpp"
#include "drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/create_load_robot_message.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/solvers/mathematical_program.h"

using std::make_shared;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;
using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace perception {
namespace estimators {
namespace {

using solvers::MathematicalProgram;
using systems::BasicVector;
using systems::ViewerDrawTranslator;

const double kPi = M_PI;
const double kQDiffNormMin = 0.01;

/**
 * Simple interval class.
 */
struct Interval {
  Interval() {}
  Interval(double min_in, double max_in)
      : min(min_in), max(max_in) {
    DRAKE_DEMAND(min <= max);
  }
  double min{};
  double max{};
  inline bool IsInside(double i) const { return i >= min && i <= max; }
  inline double width() const { return max - min; }
};

struct Bounds {
  Bounds() {}
  Bounds(Interval x_in, Interval y_in, Interval z_in)
      : x(x_in), y(y_in), z(z_in) {}
  Interval x;
  Interval y;
  Interval z;
  inline bool IsInside(double xi, double yi, double zi) const {
    return x.IsInside(xi) && y.IsInside(yi) && z.IsInside(zi);
  }
};

struct IntervalIndex {
  int index;
  Interval interval;
};

struct PlaneIndices {
  IntervalIndex a;  // first plane coordinate
  IntervalIndex b;  // second plane coordinate
  IntervalIndex d;  // depth plane coordinate
};

Matrix2Xd Generate2DPlane(double space, Interval x, Interval y) {
  const int nc = floor(x.width() / space);
  const int nr = floor(y.width() / space);
  int i = 0;
  Matrix2Xd out(2, nc * nr);
  for (int c = 0; c < nc; c++) {
    for (int r = 0; r < nr; r++) {
      out.col(i) << c * space + x.min, r * space + y.min;
      i++;
    }
  }
  out.conservativeResize(Eigen::NoChange, i);
  return out;
}

Matrix3Xd Generate2DPlane(double space, PlaneIndices is) {
  // Generate single plane.
  Matrix2Xd p2d = Generate2DPlane(space, is.a.interval, is.b.interval);
  // Map to 3d for upper and lower bound
  const int n = p2d.cols();
  Matrix3Xd p3du(3, 2 * n);
  auto map_into = [&](auto&& xpr, double value) {
    xpr.row(is.a.index) = p2d.row(0);
    xpr.row(is.b.index) = p2d.row(1);
    xpr.row(is.d.index).setConstant(value);
  };
  map_into(p3du.leftCols(n), is.d.interval.min);
  map_into(p3du.rightCols(n), is.d.interval.max);
  return p3du;
}

Matrix3Xd GenerateBoxPointCloud(double space, Bounds box) {
  IntervalIndex ix = {0, box.x};
  IntervalIndex iy = {1, box.y};
  IntervalIndex iz = {2, box.z};
  // Generate for each face
  auto xy_z = Generate2DPlane(space, {ix, iy, iz});
  auto yz_x = Generate2DPlane(space, {iy, iz, ix});
  auto xz_y = Generate2DPlane(space, {ix, iz, iy});
  Matrix3Xd pts(3, xy_z.cols() + yz_x.cols() + xz_y.cols());
  pts << xy_z, yz_x, xz_y;
  return pts;
}

// TODO(eric.cousineau): Move to a proper LCM conversion type.
void PointCloudToLcm(const Matrix3Xd& pts_W, bot_core::pointcloud_t* pmessage) {
  bot_core::pointcloud_t& message = *pmessage;
  message.points.clear();
  message.frame_id = std::string(RigidBodyTreeConstants::kWorldName);
  message.n_channels = 0;
  message.n_points = pts_W.cols();
  message.points.resize(message.n_points);
  for (int i = 0; i < message.n_points; ++i) {
    Eigen::Vector3f pt_W = pts_W.col(i).cast<float>();
    message.points[i] = {pt_W(0), pt_W(1), pt_W(2)};
  }
  message.n_points = message.points.size();
}

// TODO(eric.cousineau): Move to proper utility.
class IcpVisualizer {
 public:
  explicit IcpVisualizer(const Scene* scene,
                         bool auto_init = true)
      : scene_(scene) {
    if (auto_init) {
      Init();
    }
  }
  void Init() {
    const lcmt_viewer_load_robot load_msg(
        (multibody::CreateLoadRobotMessage<double>(scene_->tree())));
    vector<uint8_t> bytes(load_msg.getEncodedSize());
    load_msg.encode(bytes.data(), 0, bytes.size());
    lcm_.Publish("DRAKE_VIEWER_LOAD_ROBOT", bytes.data(), bytes.size());
  }
  void PublishScene(const SceneState& scene_state) {
    const ViewerDrawTranslator draw_msg_tx(scene_->tree());
    drake::lcmt_viewer_draw draw_msg;
    vector<uint8_t> bytes;
    const int num_q = scene_->tree().get_num_positions();
    const int num_v = scene_->tree().get_num_velocities();
    const int num_x = num_q + num_v;
    BasicVector<double> x(num_x);
    auto xb = x.get_mutable_value();
    xb.setZero();
    xb.head(num_q) = scene_state.q();
    draw_msg_tx.Serialize(0, x, &bytes);
    lcm_.Publish("DRAKE_VIEWER_DRAW", bytes.data(), bytes.size());
  }
  void PublishCloud(const Matrix3Xd& points) {
    bot_core::pointcloud_t pt_msg;
    PointCloudToLcm(points, &pt_msg);
    vector<uint8_t> bytes(pt_msg.getEncodedSize());
    pt_msg.encode(bytes.data(), 0, bytes.size());
    lcm_.Publish("DRAKE_POINTCLOUD_RGBD", bytes.data(), bytes.size());
  }

 private:
  lcm::DrakeLcm lcm_;
  const Scene* scene_;
};

class ArticulatedIcpTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a formulation for a simple floating-base target
    string file_path = FindResourceOrThrow(
        "drake/perception/estimators/dev/simple_cuboid.urdf");
    // TODO(eric.cousineau): Use kQuaternion.
    auto floating_base_type = multibody::joints::kRollPitchYaw;
    shared_ptr<RigidBodyFramed> weld_frame{nullptr};
    mutable_tree_ = new RigidBodyTreed();
    parsers::urdf::AddModelInstanceFromUrdfFile(file_path, floating_base_type,
                                                weld_frame, mutable_tree_);

    mutable_tree_->compile();
    tree_.reset(mutable_tree_);
    tree_cache_.reset(new KinematicsCached(tree_->CreateKinematicsCache()));

    Vector3d obj_xyz(0, 0, 0.25);
    Vector3d obj_rpy(0, 0, 0);
    const int nq = tree_->get_num_positions();
    ASSERT_EQ(6, nq);
    q_init_.resize(nq);
    q_init_ << obj_xyz, obj_rpy;

    // Perturbation for initializing local ICP.
    q_perturb_.resize(nq);
    q_perturb_ << 0.3, 0.25, 0.25, kPi / 8, kPi / 12, kPi / 10;

    // Add fixed camera frame.
    // NOTE: At present, the camera pose does not influence anything given
    // that it is fixed-base.
    Vector3d camera_xyz(-2, 0, 0.1);
    camera_xyz += obj_xyz;
    Vector3d camera_rpy(0, 0, 0);  // degrees
    camera_rpy *= kPi / 180;
    auto* world_body = &mutable_tree_->world();
    shared_ptr<RigidBodyFramed> camera_frame;
    camera_frame.reset(
        new RigidBodyFramed("camera", world_body, camera_xyz, camera_rpy));
    mutable_tree_->addFrame(camera_frame);

    // Create the scene.
    scene_.reset(new Scene(tree_.get(), 0, camera_frame->get_frame_index()));

    // Compute the initial body correspondences.
    ComputeBodyCorrespondenceInfluences(*scene_, &influences_);

    // Generate simple point cloud corresponding to shape of box.
    const Bounds box(
        Interval(-0.03, 0.03),
        Interval(-0.03, 0.03),
        Interval(-0.1, 0.1));
    const double space = 0.02;
    // Transform to initial pose dictated by `q_init_`.
    Eigen::Isometry3d T_WB;
    T_WB.linear() << drake::math::rpy2rotmat(obj_rpy);
    T_WB.translation() << obj_xyz;
    points_ = T_WB * GenerateBoxPointCloud(space, box);

    // Create visualizer.
    vis_.reset(new IcpVisualizer(scene_.get()));
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
  unique_ptr<IcpVisualizer> vis_;
};

TEST_F(ArticulatedIcpTest, PositiveReturnsZeroCost) {
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

  double tol = 1e-10;
  // The error should be near zero for points on the surface.
  EXPECT_NEAR(0, cost.cost(), tol);
  // Ensure that we have the same cost from the linearized L2 norm version.
  ArticulatedIcpLinearizedNormCost lin_cost(scene_.get());
  ComputeCost(scene_state, correspondence, &lin_cost);
  EXPECT_EQ(cost.cost(), lin_cost.cost());
}

// TODO(eric.cousineau): Add SVD computation for known correspondences.
// Add unittest comparing ICP for a single body using SVD.

TEST_F(ArticulatedIcpTest, PositiveReturnsIncreasingCost) {
  // Start box at the given state, ensure that the cost increases as we
  // translate away from the object given this simple scene.
  double prev_cost = 0;
  SceneState scene_state(scene_.get());
  for (int i = 0; i < 5; ++i) {
    VectorXd q = q_init_;
    q(0) = 0.05 * i;
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

TEST_F(ArticulatedIcpTest, PositiveReturnsConvergenceTest) {
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

  int iter = 0;
  ArticulatedPointCorrespondences correspondence;
  ArticulatedIcpLinearizedNormCost icp_cost(scene_.get());

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
    if (q_diff_norm < kQDiffNormMin) {
      break;
    }

    // Update initial guess.
    q = prog.GetSolution(q_var);
    ++iter;
    ASSERT_TRUE(iter < iter_max);
  }
  drake::log()->info("Took {} iterations for acceptable convergence", iter);

  scene_state.Update(q);
  vis_->PublishScene(scene_state);
  vis_->PublishCloud(points_);
}

}  // namespace
}  // namespace estimators
}  // namespace perception
}  // namespace drake
