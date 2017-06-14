// clang-format: off
#include "drake/multibody/multibody_tree/multibody_tree.h"
// clang-format: on

#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

using benchmarks::Acrobot;
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix4d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

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
//      X_SiSo(θ₂) Elbow revolute mobilizer with generalized position θ₂.
//      +--+-----+
//      |  ^     |
//      |  | Eo  | Elbow outboard frame Eo.
//      |  +-->  |
//      |        |
//      |  X_LEo | Pose of Eo in L.
//      |        |
//      |  ^     |
//      |  | L   | Lower link body frame L.
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
    // Creates a NaN SpatialInertia to instantiate the two RigidBody links of
    // the pendulum. Using a NaN spatial inertia is ok so far since we are still
    // not performing any numerical computations. This is only to test API.
    // M_Bo_B is the spatial inertia about the body frame's origin Bo and
    // expressed in the body frame B.
    SpatialInertia<double> M_Bo_B;

    // Adds the upper and lower links of the pendulum.
    // Using: const BodyType& AddBody(std::unique_ptr<BodyType> body).
    upper_link_ =
        &model_->AddBody(make_unique<RigidBody<double>>(M_Bo_B));
    // Using: const BodyType<T>& AddBody(Args&&... args)
    lower_link_ = &model_->AddBody<RigidBody>(M_Bo_B);

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
    // An outboard frame Eo is rigidly attached to the lower link. It is located
    // at y = +half_link_length_ in the frame of the lower link body.
    // X_UEi specifies the pose of the elbow inboard frame Ei in the body
    // frame U of the upper link.
    // X_LEo specifies the pose of the elbow outboard frame Eo in the body
    // frame L of the lower link.
    // In this case we create a frame using the FixedOffsetFrame::Create()
    // method taking a Body, i.e., creating a frame with a fixed offset from the
    // upper link body frame.
    elbow_inboard_frame_ =
        &model_->AddFrame<FixedOffsetFrame>(*upper_link_, X_UEi_);
    elbow_outboard_frame_ =
        &model_->AddFrame<FixedOffsetFrame>(*lower_link_, X_LEo_);

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
  }

  // Helper method to extract a pose from the position kinematics.
  // TODO(amcastro-tri):
  // Replace this by a method Body<T>::get_pose_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  const Isometry3d& get_body_pose_in_world(
      const PositionKinematicsCache<double>& pc,
      const Body<double>& body) const {
    const MultibodyTreeTopology& topology = model_->get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return pc.get_X_WB(topology.get_body(body.get_index()).body_node);
  }

 protected:
  // For testing only so that we can retrieve/set (future to be) cache entries,
  // this method initializes the poses of each link in the position kinematics
  // cache.
  void SetPendulumPoses(PositionKinematicsCache<double>* pc) {
    pc->get_mutable_X_WB(BodyNodeIndex(1)) = X_WL_;
  }

  std::unique_ptr<MultibodyTree<double>> model_;
  const Body<double>* world_body_;
  // Bodies:
  const RigidBody<double>* upper_link_;
  const RigidBody<double>* lower_link_;
  // Frames:
  const BodyFrame<double>* shoulder_inboard_frame_;
  const FixedOffsetFrame<double>* shoulder_outboard_frame_;
  const FixedOffsetFrame<double>* elbow_inboard_frame_;
  const FixedOffsetFrame<double>* elbow_outboard_frame_;
  // Mobilizers:
  const RevoluteMobilizer<double>* shoulder_mobilizer_;
  const RevoluteMobilizer<double>* elbow_mobilizer_;
  // Pendulum parameters:
  const double link_length_ = 1.0;
  const double half_link_length_ = link_length_ / 2;
  // Poses:
  // Desired pose of the lower link frame L in the world frame W.
  const Isometry3d X_WL_{Translation3d(0.0, -half_link_length_, 0.0)};
  // Pose of the shoulder outboard frame So in the upper link frame U.
  const Isometry3d X_USo_{Translation3d(0.0, half_link_length_, 0.0)};
  // Pose of the elbow inboard frame Ei in the upper link frame U.
  const Isometry3d X_UEi_{Translation3d(0.0, -half_link_length_, 0.0)};
  // Pose of the elbow outboard frame Eo in the lower link frame L.
  const Isometry3d X_LEo_{Translation3d(0.0, half_link_length_, 0.0)};
};

TEST_F(PendulumTests, CreateModelBasics) {
  // Initially there is only one body, the world.
  EXPECT_EQ(model_->get_num_bodies(), 1);
  // And there is only one frame, the world frame.
  EXPECT_EQ(model_->get_num_frames(), 1);

  CreatePendulumModel();

  // Verifies the number of multibody elements is correct.
  EXPECT_EQ(model_->get_num_bodies(), 3);
  EXPECT_EQ(model_->get_num_frames(), 6);
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

// Frame indexes are assigned by MultibodyTree. The number of frames
// equals the number of body frames (one per body) plus the number of
// additional frames added to the system (like FixedOffsetFrame objects).
// Frames are indexed in the order they are added to the MultibodyTree model.
// The order of the frames and their indexes is an implementation detail that
// users do not need to know about. Therefore this unit test would need to
// change in the future if we decide to change the "internal detail" on how we
// assign these indexes.
TEST_F(PendulumTests, Indexes) {
  CreatePendulumModel();
  EXPECT_EQ(shoulder_inboard_frame_->get_index(), FrameIndex(0));
  EXPECT_EQ(upper_link_->get_body_frame().get_index(), FrameIndex(1));
  EXPECT_EQ(lower_link_->get_body_frame().get_index(), FrameIndex(2));
  EXPECT_EQ(shoulder_outboard_frame_->get_index(), FrameIndex(3));
  EXPECT_EQ(elbow_inboard_frame_->get_index(), FrameIndex(4));
  EXPECT_EQ(elbow_outboard_frame_->get_index(), FrameIndex(5));
}

// Asserts that the Finalize() stage is successful and that re-finalization is
// not allowed.
TEST_F(PendulumTests, Finalize) {
  CreatePendulumModel();
  // Finalize() stage.
  EXPECT_FALSE(model_->topology_is_valid());  // Not valid before Finalize().
  EXPECT_NO_THROW(model_->Finalize());
  EXPECT_TRUE(model_->topology_is_valid());  // Valid after Finalize().

  // Asserts that no more multibody elements can be added after finalize.
  SpatialInertia<double> M_Bo_B;
  EXPECT_THROW(model_->AddBody<RigidBody>(M_Bo_B), std::logic_error);
  EXPECT_THROW(model_->AddFrame<FixedOffsetFrame>(*lower_link_, X_LEo_),
               std::logic_error);
  EXPECT_THROW(model_->AddMobilizer<RevoluteMobilizer>(
      *shoulder_inboard_frame_, *shoulder_outboard_frame_,
      Vector3d::UnitZ()), std::logic_error);

  // Asserts re-finalization is not allowed.
  EXPECT_THROW(model_->Finalize(), std::logic_error);
}

// This is an experiment with std::reference_wrapper to show that we can save
// bodies in an array of references.
TEST_F(PendulumTests, StdReferenceWrapperExperiment) {
  // Initially there is only one body, the world.
  EXPECT_EQ(model_->get_num_bodies(), 1);
  // And there is only one frame, the world frame.
  EXPECT_EQ(model_->get_num_frames(), 1);
  CreatePendulumModel();

  // Vector of references.
  std::vector<std::reference_wrapper<const Body<double>>> bodies;
  bodies.push_back(*world_body_);
  bodies.push_back(*upper_link_);
  bodies.push_back(*lower_link_);

  // Verify that vector "bodies" effectively holds valid references to the
  // actual body elements in the tree.
  // In addition, since these tests compare actual memory addresses, they
  // ensure that bodies were not copied instead.
  // Unfortunately we need the ugly get() method since operator.() is not
  // overloaded.
  EXPECT_EQ(&bodies[world_body_->get_index()].get(), world_body_);
  EXPECT_EQ(&bodies[upper_link_->get_index()].get(), upper_link_);
  EXPECT_EQ(&bodies[lower_link_->get_index()].get(), lower_link_);
}

TEST_F(PendulumTests, CreateContext) {
  CreatePendulumModel();

  // Verifies the number of multibody elements is correct. In this case:
  // - world_
  // - upper_link_
  // - lower_link_
  EXPECT_EQ(model_->get_num_bodies(), 3);

  // Verify we cannot create a Context until we have a valid topology.
  EXPECT_FALSE(model_->topology_is_valid());  // Not valid before Finalize().
  EXPECT_THROW(model_->CreateDefaultContext(), std::logic_error);

  // Finalize() stage.
  EXPECT_NO_THROW(model_->Finalize());
  EXPECT_TRUE(model_->topology_is_valid());  // Valid after Finalize().

  // Create Context.
  std::unique_ptr<Context<double>> context;
  EXPECT_NO_THROW(context = model_->CreateDefaultContext());

  // Tests MultibodyTreeContext accessors.
  auto mbt_context =
      dynamic_cast<MultibodyTreeContext<double>*>(context.get());
  ASSERT_TRUE(mbt_context != nullptr);

  // Verifies the correct number of generalized positions and velocities.
  EXPECT_EQ(mbt_context->get_positions().size(), 2);
  EXPECT_EQ(mbt_context->get_mutable_positions().size(), 2);
  EXPECT_EQ(mbt_context->get_velocities().size(), 2);
  EXPECT_EQ(mbt_context->get_mutable_velocities().size(), 2);

  // Verifies methods to retrieve fixed-sized segments of the state.
  EXPECT_EQ(mbt_context->get_state_segment<1>(1).size(), 1);
  EXPECT_EQ(mbt_context->get_mutable_state_segment<1>(1).size(), 1);

  // Set the poses of each body in the position kinematics cache to have an
  // arbitrary value that we can use for unit testing. In practice the poses in
  // the position kinematics will be the result of a position kinematics update
  // and will live in the context as a cache entry.
  PositionKinematicsCache<double> pc(model_->get_topology());
  SetPendulumPoses(&pc);

  // Retrieve body poses from position kinematics cache.
  const Isometry3d &X_WW = get_body_pose_in_world(pc, *world_body_);
  const Isometry3d &X_WLu = get_body_pose_in_world(pc, *upper_link_);

  // Asserts that the retrieved poses match with the ones specified by the unit
  // test method SetPendulumPoses().
  EXPECT_TRUE(X_WW.matrix().isApprox(Matrix4d::Identity()));
  EXPECT_TRUE(X_WLu.matrix().isApprox(X_WL_.matrix()));
}

// Unit test fixture to verify the correctness of MultibodyTree methods for
// computing kinematics. This fixture uses the reference solution provided by
// benchmarks::Acrobot.
class PendulumKinematicTests : public PendulumTests {
 public:
  void SetUp() override {
    PendulumTests::SetUp();
    CreatePendulumModel();
    model_->Finalize();
    context_ = model_->CreateDefaultContext();
    mbt_context_ =
        dynamic_cast<MultibodyTreeContext<double>*>(context_.get());
  }
 protected:
  std::unique_ptr<Context<double>> context_;
  MultibodyTreeContext<double>* mbt_context_;
  // Reference benchmark for verification.
  Acrobot<double> acrobot_benchmark_{Vector3d::UnitZ() /* Plane normal */,
                                     Vector3d::UnitY() /* Up vector */};
};

// Verify the correctness of method MultibodyTree::CalcPositionKinematicsCache()
// comparing the computed results the reference solution provided by
// benchmarks::Acrobot.
TEST_F(PendulumKinematicTests, CalcPositionKinematics) {
  // This is the minimum factor of the machine precision within which these
  // tests pass.
  const int kEpsilonFactor = 2;
  const double kEpsilon =
      kEpsilonFactor * std::numeric_limits<double>::epsilon();

  // By default CreateDefaultContext() sets mobilizer to their zero
  // configuration.
  EXPECT_EQ(shoulder_mobilizer_->get_angle(*context_), 0.0);
  EXPECT_EQ(elbow_mobilizer_->get_angle(*context_), 0.0);

  // Test mobilizer's setter/getters.
  shoulder_mobilizer_->set_angle(context_.get(), M_PI);
  EXPECT_EQ(shoulder_mobilizer_->get_angle(*context_), M_PI);
  shoulder_mobilizer_->set_zero_configuration(context_.get());
  EXPECT_EQ(shoulder_mobilizer_->get_angle(*context_), 0.0);

  PositionKinematicsCache<double> pc(model_->get_topology());

  const int num_angles = 50;
  const double kDeltaAngle = 2 * M_PI / (num_angles - 1.0);
  for (double ishoulder = 0; ishoulder < num_angles; ++ishoulder) {
    const double shoulder_angle = -M_PI + ishoulder * kDeltaAngle;
    for (double ielbow = 0; ielbow < num_angles; ++ielbow) {
      const double elbow_angle = -M_PI + ielbow * kDeltaAngle;

      shoulder_mobilizer_->set_angle(context_.get(), shoulder_angle);
      EXPECT_EQ(shoulder_mobilizer_->get_angle(*context_), shoulder_angle);
      elbow_mobilizer_->set_angle(context_.get(), elbow_angle);
      EXPECT_EQ(elbow_mobilizer_->get_angle(*context_), elbow_angle);

      // Verify this matches the corresponding entries in the context.
      EXPECT_NEAR(mbt_context_->get_positions()(0), shoulder_angle, kEpsilon);
      EXPECT_NEAR(mbt_context_->get_positions()(1), elbow_angle, kEpsilon);

      model_->CalcPositionKinematicsCache(*context_, &pc);

      // Indexes to the BodyNode objects associated with each mobilizer.
      const BodyNodeIndex shoulder_node =
          shoulder_mobilizer_->get_topology().body_node;
      const BodyNodeIndex elbow_node =
          elbow_mobilizer_->get_topology().body_node;

      // Expected poses of the outboard frames measured in the inboard frame.
      Isometry3d X_SiSo(AngleAxisd(shoulder_angle, Vector3d::UnitZ()));
      Isometry3d X_EiEo(AngleAxisd(elbow_angle, Vector3d::UnitZ()));

      // Verify the values in the position kinematics cache.
      EXPECT_TRUE(pc.get_X_FM(shoulder_node).matrix().isApprox(
          X_SiSo.matrix()));
      EXPECT_TRUE(pc.get_X_FM(elbow_node).matrix().isApprox(
          X_EiEo.matrix()));

      // Verify that both, const and mutable versions point to the same address.
      EXPECT_EQ(&pc.get_X_FM(shoulder_node),
                &pc.get_mutable_X_FM(shoulder_node));
      EXPECT_EQ(&pc.get_X_FM(elbow_node),
                &pc.get_mutable_X_FM(elbow_node));

      // Retrieve body poses from position kinematics cache.
      const Isometry3d& X_WW = get_body_pose_in_world(pc, *world_body_);
      const Isometry3d& X_WU = get_body_pose_in_world(pc, *upper_link_);
      const Isometry3d& X_WL = get_body_pose_in_world(pc, *lower_link_);

      const Isometry3d X_WU_expected =
          acrobot_benchmark_.CalcLink1PoseInWorldFrame(shoulder_angle);

      const Isometry3d X_WL_expected =
          acrobot_benchmark_.CalcLink2PoseInWorldFrame(shoulder_angle,
                                                       elbow_angle);

      // Asserts that the retrieved poses match with the ones specified by the
      // unit test method SetPendulumPoses().
      EXPECT_TRUE(X_WW.matrix().isApprox(Matrix4d::Identity(), kEpsilon));
      EXPECT_TRUE(X_WU.matrix().isApprox(X_WU_expected.matrix(), kEpsilon));
      EXPECT_TRUE(X_WL.matrix().isApprox(X_WL_expected.matrix(), kEpsilon));
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
