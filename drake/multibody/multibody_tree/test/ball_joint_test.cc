#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/benchmarks/cylinder_torque_free_analytical_solution/torque_free_cylinder_exact_solution.h"
#include "drake/multibody/multibody_tree/test/free_body_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace {

//const double kEpsilon = std::numeric_limits<double>::epsilon();

using benchmarks::cylinder_torque_free_analytical_solution::TorqueFreeCylinderExactSolution;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;
using std::vector;
using systems::Context;

#if 0
TorqueFreeCylinderExactSolution(const Quaterniond& quat_NB_initial,
                                const Vector3d& w_NB_B_initial,
                                const Vector3d& p_NoBcm_N_initial,
                                const Vector3d& v_NBcm_B_initial,
                                const Vector3d& gravity_N) {
#endif

GTEST_TEST(RollPitchYawTest, TimeDerivatives) {
  Vector3d w0_WB =
      Vector3d::UnitX() + Vector3d::UnitY() + Vector3d::UnitZ();
  Vector3d p0_WBcm = Vector3d::Zero();
  Vector3d v0_WBcm = Vector3d::Zero();
  Vector3d gravity_W = Vector3d::Zero();

  TorqueFreeCylinderExactSolution benchmark_(
      Quaterniond::Identity(), w0_WB,
      p0_WBcm, v0_WBcm, gravity_W);
}

#if 0
// Set of MultibodyTree tests for a double pendulum model.
// This double pendulum is similar to the acrobot model described in Section 3.1
// of the Underactuated Robotics notes available online at
// http://underactuated.csail.mit.edu/underactuated.html?chapter=3.
// The only difference is that this model has no actuation.
// This double pendulum is defined in the x-y plane with gravity acting in the
// negative y-axis direction.
// In this model the two links of the pendulum have the same length with their
// respective centers of mass located at the links' centroids.
//
// The schematic below shows the location and relationship of the frames defined
// by the model. A few comments:
//  - The pendulum moves in the x-y plane, with angles θ₁ and θ₂ defined
//    positive according to the right-hand-rule with the thumb aligned in the
//    z-direction.
//  - The body frames for each link are placed at their geometric center.
//  - The origin of the shoulder frames (Si and So) are coincident at all times.
//    So is aligned with Si for θ₁ = 0.
//  - The origin of the elbow frames (Ei and Eo) are coincident at all times.
//    Eo is aligned with Ei for θ₂ = 0.
//
//       y ^
//         | Si ≡ W World body frame.
//         +--> x  Shoulder inboard frame Si coincides with W.
//      X_SiSo(θ₁) Shoulder revolute mobilizer with generalized position θ₁.
//      +--+-----+
//      |  ^     |
//      |  | So  | Shoulder outboard frame So.
//      |  +-->  |
//      |        |
//      |  X_USo | Pose of So in U.
//      |        |
//      |  ^     |
//      |  | U   | Upper link body frame U.
//      |  +-->  |
//      |        |
//      |  X_UEi | Pose of Ei in U.
//      |        |
//      |  ^     |
//      |  | Ei  | Elbow inboard frame Ei.
//      |  +-->  |
//      +--------+
//      X_EiEo(θ₂) Elbow revolute mobilizer with generalized position θ₂.
//      +--+-----+
//      |  ^     |
//      |  |Eo/L | Elbow outboard frame Eo.
//      |  +-->  | Lower link's frame L is coincident with the elbow frame Eo.
//      |        |
//      |p_LoLcm | Position vector of the link's com measured from the link's
//      |        | frame origin Lo.
//      |  ^     |
//      |  | Lcm | Lower link's frame L shifted to its center of mass.
//      |  +-->  |
//      |        |
//      |        |
//      |        |
//      |        |
//      |        |
//      |        |
//      +--------+
class PendulumTests : public ::testing::Test {
 public:
  // Creates an "empty" MultibodyTree that only contains the "world" body and
  // world body frame.
  void SetUp() override {
    model_ = std::make_unique<MultibodyTree<double>>();

    // Retrieves the world body.
    world_body_ = &model_->get_world_body();
  }

  // Sets up the MultibodyTree model for a double pendulum. See this unit test's
  // class description for details.
  void CreatePendulumModel() {
    // Spatial inertia of the upper link about its frame U and expressed in U.
    Vector3d link1_com_U = Vector3d::Zero();  // U is at the link's COM.
    // Inertia for a thin rod with moment of inertia link1_Ic_ about the y axis.
    UnitInertia<double> G_U =
        UnitInertia<double>::StraightLine(link1_Ic_, Vector3d::UnitY());
    SpatialInertia<double> M_U(link1_mass_, link1_com_U, G_U);

    // Spatial inertia of the lower link about its frame L and expressed in L.
    Vector3d link2_com_L = Vector3d::Zero();  // L is at the link's COM.
    // Inertia for a thin rod with moment of inertia link2_Ic_ about the y axis.
    UnitInertia<double> G_Lcm =
        UnitInertia<double>::StraightLine(link2_Ic_, Vector3d::UnitY());
    // Spatial inertia about L's center of mass Lcm.
    SpatialInertia<double> M_Lcm(link2_mass_, link2_com_L, G_Lcm);
    // Since L's frame origin Lo is not at the the lower link's center of mass
    // Lcm, we must shift M_Lcm to obtain M_Lo.
    const Vector3d p_LoLcm(0.0, -half_link2_length_, 0.0);
    SpatialInertia<double> M_L = M_Lcm.Shift(-p_LoLcm);

    // Adds the upper and lower links of the pendulum.
    // Using: const BodyType& AddBody(std::unique_ptr<BodyType> body).
    upper_link_ =
        &model_->AddBody(make_unique<RigidBody<double>>(M_U));
    // Using: const BodyType<T>& AddBody(Args&&... args)
    lower_link_ = &model_->AddBody<RigidBody>(M_L);

    // The shoulder is the mobilizer that connects the world to the upper link.
    // Its inboard frame, Si, is the world frame. Its outboard frame, So, a
    // fixed offset frame on the upper link.
    shoulder_inboard_frame_ = &model_->get_world_frame();

    // The body frame of the upper link is U, and that of the lower link is L.
    // We will add a frame for the pendulum's shoulder. This will be the
    // shoulder's outboard frame So.
    // X_USo specifies the pose of the shoulder outboard frame So in the body
    // frame U of the upper link.
    // In this case the frame is created explicitly from the body frame of
    // upper_link.
    shoulder_outboard_frame_ =
        &model_->AddFrame<FixedOffsetFrame>(
            upper_link_->get_body_frame(), X_USo_);

    // The elbow is the mobilizer that connects upper and lower links.
    // Below we will create inboard and outboard frames associated with the
    // pendulum's elbow.
    // An inboard frame Ei is rigidly attached to the upper link. It is located
    // at y = -half_link_length_ in the frame of the upper link body.
    // X_UEi specifies the pose of the elbow inboard frame Ei in the body
    // frame U of the upper link.
    // In this case we create a frame using the FixedOffsetFrame::Create()
    // method taking a Body, i.e., creating a frame with a fixed offset from the
    // upper link body frame.
    elbow_inboard_frame_ =
        &model_->AddFrame<FixedOffsetFrame>(*upper_link_, X_UEi_);

    // To make this test a bit more interesting, we define the lower link's
    // frame L to be coincident with the elbow's outboard frame. Therefore,
    // Lo != Lcm.
    elbow_outboard_frame_ = &lower_link_->get_body_frame();

    // Adds the shoulder and elbow mobilizers of the pendulum.
    // Using:
    //  const Mobilizer& AddMobilizer(std::unique_ptr<MobilizerType> mobilizer).
    shoulder_mobilizer_ =
        &model_->AddMobilizer(
            make_unique<RevoluteMobilizer<double>>(
                *shoulder_inboard_frame_, *shoulder_outboard_frame_,
                Vector3d::UnitZ() /*revolute axis*/));
    // Using: const MobilizerType<T>& AddMobilizer(Args&&... args)
    elbow_mobilizer_ = &model_->AddMobilizer<RevoluteMobilizer>(
        *elbow_inboard_frame_, *elbow_outboard_frame_,
        Vector3d::UnitZ() /*revolute axis*/);

    // Add force element for a constant gravity.
    model_->AddForceElement<UniformGravityFieldElement>(
        Vector3d(0.0, -acceleration_of_gravity_, 0.0));
  }

  // Helper method to extract a pose from the position kinematics.
  // TODO(amcastro-tri):
  // Replace this by a method Body<T>::get_pose_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  template <typename T>
  const Isometry3<T>& get_body_pose_in_world(
      const PositionKinematicsCache<T>& pc,
      const Body<T>& body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return pc.get_X_WB(topology.get_body(body.get_index()).body_node);
  }

  // Helper method to extract spatial velocity from the velocity kinematics
  // cache.
  // TODO(amcastro-tri):
  // Replace this by a method
  // Body<T>::get_spatial_velocity_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  const SpatialVelocity<double>& get_body_spatial_velocity_in_world(
      const VelocityKinematicsCache<double>& vc,
      const Body<double>& body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return vc.get_V_WB(topology.get_body(body.get_index()).body_node);
  }

  // Helper method to extract spatial acceleration from the acceleration
  // kinematics cache.
  // TODO(amcastro-tri):
  // Replace this by a method
  // Body<T>::get_spatial_acceleration_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  const SpatialAcceleration<double>& get_body_spatial_acceleration_in_world(
      const AccelerationKinematicsCache<double>& ac,
      const Body<double>& body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return ac.get_A_WB(topology.get_body(body.get_index()).body_node);
  }

 protected:
  // For testing only so that we can retrieve/set (future to be) cache entries,
  // this method initializes the poses of each link in the position kinematics
  // cache.
  void SetPendulumPoses(PositionKinematicsCache<double>* pc) {
    pc->get_mutable_X_WB(BodyNodeIndex(1)) = X_WL_;
  }

  TestAxis plane_axis_;

  std::unique_ptr<MultibodyTree<double>> model_;
  const Body<double>* world_body_;
  // Bodies:
  const RigidBody<double>* upper_link_;
  const RigidBody<double>* lower_link_;
  // Frames:
  const BodyFrame<double>* shoulder_inboard_frame_;
  const FixedOffsetFrame<double>* shoulder_outboard_frame_;
  const FixedOffsetFrame<double>* elbow_inboard_frame_;
  const Frame<double>* elbow_outboard_frame_;
  // Mobilizers:
  const RevoluteMobilizer<double>* shoulder_mobilizer_;
  const RevoluteMobilizer<double>* elbow_mobilizer_;
  // Pendulum parameters:
  const double link1_length_ = 1.0;
  const double link1_mass_ = 1.0;
  // Unit inertia about an axis perpendicular to the rod for link1.
  const double link1_Ic_ = .083;
  const double link2_length_ = 2.0;
  const double link2_mass_ = 1.0;
  // Unit inertia about an axis perpendicular to the rod for link2.
  const double link2_Ic_ = .33;
  const double half_link1_length_ = link1_length_ / 2;
  const double half_link2_length_ = link2_length_ / 2;
  // Acceleration of gravity at Earth's surface.
  const double acceleration_of_gravity_ = 9.81;
  // Poses:
  // Desired pose of the lower link frame L in the world frame W.
  const Isometry3d X_WL_{Translation3d(0.0, -half_link1_length_, 0.0)};
  // Pose of the shoulder outboard frame So in the upper link frame U.
  const Isometry3d X_USo_{Translation3d(0.0, half_link1_length_, 0.0)};
  // Pose of the elbow inboard frame Ei in the upper link frame U.
  const Isometry3d X_UEi_{Translation3d(0.0, -half_link1_length_, 0.0)};
  // Pose of the elbow outboard frame Eo in the lower link frame L.
  const Isometry3d X_LEo_{Translation3d(0.0, half_link2_length_, 0.0)};
};

TEST_F(PendulumTests, CreateModelBasics) {
  // Initially there is only one body, the world.
  EXPECT_EQ(model_->get_num_bodies(), 1);
  // And there is only one frame, the world frame.
  EXPECT_EQ(model_->get_num_frames(), 1);

  CreatePendulumModel();

  // Verifies the number of multibody elements is correct.
  EXPECT_EQ(model_->get_num_bodies(), 3);
  EXPECT_EQ(model_->get_num_frames(), 5);
  EXPECT_EQ(model_->get_num_mobilizers(), 2);

  // Check that frames are associated with the correct bodies.
  EXPECT_EQ(
      shoulder_inboard_frame_->get_body().get_index(),
      world_body_->get_index());
  EXPECT_EQ(
      shoulder_outboard_frame_->get_body().get_index(),
      upper_link_->get_index());
  EXPECT_EQ(
      elbow_inboard_frame_->get_body().get_index(), upper_link_->get_index());
  EXPECT_EQ(
      elbow_outboard_frame_->get_body().get_index(), lower_link_->get_index());

  // Checks that mobilizers connect the right frames.
  EXPECT_EQ(shoulder_mobilizer_->get_inboard_frame().get_index(),
            world_body_->get_body_frame().get_index());
  EXPECT_EQ(shoulder_mobilizer_->get_outboard_frame().get_index(),
            shoulder_outboard_frame_->get_index());
  EXPECT_EQ(elbow_mobilizer_->get_inboard_frame().get_index(),
            elbow_inboard_frame_->get_index());
  EXPECT_EQ(elbow_mobilizer_->get_outboard_frame().get_index(),
            elbow_outboard_frame_->get_index());

  // Checks that mobilizers connect the right bodies.
  EXPECT_EQ(shoulder_mobilizer_->get_inboard_body().get_index(),
            world_body_->get_index());
  EXPECT_EQ(shoulder_mobilizer_->get_outboard_body().get_index(),
            upper_link_->get_index());
  EXPECT_EQ(elbow_mobilizer_->get_inboard_body().get_index(),
            upper_link_->get_index());
  EXPECT_EQ(elbow_mobilizer_->get_outboard_body().get_index(),
            lower_link_->get_index());

  // Checks we can retrieve the body associated with a frame.
  EXPECT_EQ(&shoulder_inboard_frame_->get_body(), world_body_);
  EXPECT_EQ(&shoulder_outboard_frame_->get_body(), upper_link_);
  EXPECT_EQ(&elbow_inboard_frame_->get_body(), upper_link_);
  EXPECT_EQ(&elbow_outboard_frame_->get_body(), lower_link_);

  // Checks we can request inboard/outboard bodies to a mobilizer.
  EXPECT_EQ(&shoulder_mobilizer_->get_inboard_body(), world_body_);
  EXPECT_EQ(&shoulder_mobilizer_->get_outboard_body(), upper_link_);
  EXPECT_EQ(&elbow_mobilizer_->get_inboard_body(), upper_link_);
  EXPECT_EQ(&elbow_mobilizer_->get_outboard_body(), lower_link_);

  // Request revolute mobilizers' axes.
  EXPECT_EQ(shoulder_mobilizer_->get_revolute_axis(), Vector3d::UnitZ());
  EXPECT_EQ(elbow_mobilizer_->get_revolute_axis(), Vector3d::UnitZ());
}
#endif
}  // namespace
}  // namespace multibody
}  // namespace drake
