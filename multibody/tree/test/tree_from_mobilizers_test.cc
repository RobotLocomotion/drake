#include "drake/common/test_utilities/expect_no_throw.h"
// clang-format: off
#include "drake/multibody/tree/multibody_tree-inl.h"
// clang-format: on

#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_mobilizer.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

// Friend class for accessing Joint<T> protected/private internals.
class JointTester {
 public:
  JointTester() = delete;
  // For these tests we do know that a RevoluteJoint is implemented with a
  // RevoluteMobilizer.
  static const internal::RevoluteMobilizer<double>* get_mobilizer(
      const RevoluteJoint<double>& joint) {
    return joint.get_mobilizer();
  }
};

namespace internal {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

using benchmarks::Acrobot;
using Eigen::AngleAxisd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;
using std::vector;
using systems::Context;

// Set of MultibodyTree tests for a double pendulum model. The model is built
// directly using Mobilizer objects and therefore it is used to test some of the
// MultibodyTree internal details. Users would typically build their models
// using Joint objects instead.
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
//
class PendulumTests : public ::testing::Test {
 public:
  // SetUp() creates an "empty" MultibodyTree that only contains the
  // world body and world body frame.
  void SetUp() override {
    model_ = std::make_unique<MultibodyTree<double>>();
    world_body_ = &model_->world_body();
  }

  // Sets up the MultibodyTree model for a double pendulum. See this unit test's
  // class description for details. Note that this method does not finalize the
  // tree.
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
    // Since L's frame origin Lo is not at the lower link's center of mass Lcm,
    // we must shift M_Lcm to obtain M_Lo.
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
    shoulder_inboard_frame_ = &model_->world_frame();

    // The body frame of the upper link is U, and that of the lower link is L.
    // We will add a frame for the pendulum's shoulder. This will be the
    // shoulder's outboard frame So.
    // X_USo specifies the pose of the shoulder outboard frame So in the body
    // frame U of the upper link.
    // In this case the frame is created explicitly from the body frame of
    // upper_link.
    shoulder_outboard_frame_ =
        &model_->AddFrame<FixedOffsetFrame>(upper_link_->body_frame(), X_USo_);

    // Adds the shoulder and elbow mobilizers of the pendulum.
    // Using:
    //  const Mobilizer& AddMobilizer(std::unique_ptr<MobilizerType> mobilizer).
    shoulder_mobilizer_ =
        &model_->AddMobilizer(
            make_unique<RevoluteMobilizer<double>>(
                *shoulder_inboard_frame_, *shoulder_outboard_frame_,
                Vector3d::UnitZ() /*revolute axis*/));

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
    // To make this test a bit more interesting, we define the lower link's
    // frame L to be coincident with the elbow's outboard frame. Therefore,
    // Lo != Lcm.

    // Instead of creating an elbow inboard frame (Ei), an elbow outboard
    // frame (Eo) and connecting them with a Joint, we'll let the
    // MultibodyTree::AddJoint() method do that for us:
    elbow_joint_ = &model_->AddJoint<RevoluteJoint>(
        "ElbowJoint",
        *upper_link_, X_UEi_, /* Pose of Ei in U. */
        *lower_link_, {},     /* No pose provided, frame Eo IS frame L. */
        Vector3d::UnitZ()     /* revolute axis */);
    elbow_inboard_frame_ = &elbow_joint_->frame_on_parent();
    elbow_outboard_frame_ = &elbow_joint_->frame_on_child();
    EXPECT_EQ(elbow_joint_->name(), "ElbowJoint");

    // Assert that indeed the elbow joint's outboard frame IS the lower link
    // frame.
    ASSERT_EQ(elbow_outboard_frame_->index(),
              lower_link_->body_frame().index());

    // Add force element for a constant gravity pointing downwards, that is, in
    // the minus y-axis direction.
    model_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, -acceleration_of_gravity_, 0.0));
  }

  // Helper method to extract a pose from the position kinematics.
  // TODO(amcastro-tri):
  // Replace this by a method Body<T>::get_pose_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  template <typename T>
  static const RigidTransform<T> get_body_pose_in_world(
      const MultibodyTree<T>& tree,
      const PositionKinematicsCache<T>& pc,
      const Body<T>& body) {
    const MultibodyTreeTopology& topology = tree.get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    const BodyNodeIndex body_node_index =
        topology.get_body(body.index()).body_node;
    return RigidTransform<T>(pc.get_X_WB(body_node_index));
  }

  // Helper method to extract spatial velocity from the velocity kinematics
  // cache.
  // TODO(amcastro-tri):
  // Replace this by a method
  // Body<T>::get_spatial_velocity_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  static const SpatialVelocity<double>& get_body_spatial_velocity_in_world(
      const MultibodyTree<double>& tree,
      const VelocityKinematicsCache<double>& vc,
      const Body<double>& body) {
    const MultibodyTreeTopology& topology = tree.get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return vc.get_V_WB(topology.get_body(body.index()).body_node);
  }

  // Helper method to extract spatial acceleration from the acceleration
  // kinematics cache.
  // TODO(amcastro-tri):
  // Replace this by a method
  // Body<T>::get_spatial_acceleration_in_world(const Context<T>&)
  // when we can place cache entries in the context.
  static const SpatialAcceleration<double>&
  get_body_spatial_acceleration_in_world(
      const MultibodyTree<double>& tree,
      const AccelerationKinematicsCache<double>& ac, const Body<double>& body) {
    const MultibodyTreeTopology& topology = tree.get_topology();
    // Cache entries are accessed by BodyNodeIndex for fast traversals.
    return ac.get_A_WB(topology.get_body(body.index()).body_node);
  }

 protected:
  // For testing only so that we can retrieve/set (future to be) cache entries,
  // this method initializes the poses of each link in the position kinematics
  // cache.
  void SetPendulumPoses(PositionKinematicsCache<double>* pc) {
    pc->get_mutable_X_WB(BodyNodeIndex(1)) = X_WL_;
  }

  // Add elements to this model_ and then transfer the whole thing to
  // a MultibodyTreeSystem for execution.
  std::unique_ptr<MultibodyTree<double>> model_;
  const Body<double>* world_body_{nullptr};

  // Bodies:
  const RigidBody<double>* upper_link_{nullptr};
  const RigidBody<double>* lower_link_{nullptr};
  // Frames:
  const BodyFrame<double>* shoulder_inboard_frame_{nullptr};
  const FixedOffsetFrame<double>* shoulder_outboard_frame_{nullptr};
  const Frame<double>* elbow_inboard_frame_{nullptr};
  const Frame<double>* elbow_outboard_frame_{nullptr};
  // Mobilizers:
  const RevoluteMobilizer<double>* shoulder_mobilizer_{nullptr};
  const RevoluteMobilizer<double>* elbow_mobilizer_{nullptr};
  // Joints:
  const RevoluteJoint<double>* elbow_joint_{nullptr};
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
  const RigidTransformd X_WL_{Vector3d(0.0, -half_link1_length_, 0.0)};
  // Pose of the shoulder outboard frame So in the upper link frame U.
  const RigidTransformd X_USo_{Vector3d(0.0, half_link1_length_, 0.0)};
  // Pose of the elbow inboard frame Ei in the upper link frame U.
  const RigidTransformd X_UEi_{Vector3d(0.0, -half_link1_length_, 0.0)};
  // Pose of the elbow outboard frame Eo in the lower link frame L.
  const RigidTransformd X_LEo_{Vector3d(0.0, half_link2_length_, 0.0)};
};

TEST_F(PendulumTests, CreateModelBasics) {
  // Initially there is only one body, the world.
  EXPECT_EQ(model_->num_bodies(), 1);
  // And there is only one frame, the world frame.
  EXPECT_EQ(model_->num_frames(), 1);

  CreatePendulumModel();

  // Verifies the number of multibody elements is correct.
  EXPECT_EQ(model_->num_bodies(), 3);
  EXPECT_EQ(model_->num_frames(), 5);
  // Joint has no implementation before finalize.
  EXPECT_EQ(model_->num_mobilizers(), 1);

  // Check that frames are associated with the correct bodies.
  EXPECT_EQ(
      shoulder_inboard_frame_->body().index(),
      world_body_->index());
  EXPECT_EQ(
      shoulder_outboard_frame_->body().index(),
      upper_link_->index());

  // Checks that mobilizers connect the right frames.
  EXPECT_EQ(shoulder_mobilizer_->inboard_frame().index(),
            world_body_->body_frame().index());
  EXPECT_EQ(shoulder_mobilizer_->outboard_frame().index(),
            shoulder_outboard_frame_->index());

  // Checks that mobilizers connect the right bodies.
  EXPECT_EQ(shoulder_mobilizer_->inboard_body().index(),
            world_body_->index());
  EXPECT_EQ(shoulder_mobilizer_->outboard_body().index(),
            upper_link_->index());

  // Checks we can retrieve the body associated with a frame.
  EXPECT_EQ(&shoulder_inboard_frame_->body(), world_body_);
  EXPECT_EQ(&shoulder_outboard_frame_->body(), upper_link_);

  // Checks we can request inboard/outboard bodies to a mobilizer.
  EXPECT_EQ(&shoulder_mobilizer_->inboard_body(), world_body_);
  EXPECT_EQ(&shoulder_mobilizer_->outboard_body(), upper_link_);

  // Request revolute mobilizers' axes.
  EXPECT_EQ(shoulder_mobilizer_->revolute_axis(), Vector3d::UnitZ());

  // We need to Finalize() our model before testing the elbow mobilizer was
  // created correctly. Joint implementations are created at Finalize().
  DRAKE_ASSERT_NO_THROW(model_->Finalize());
  elbow_mobilizer_ = JointTester::get_mobilizer(*elbow_joint_);

  EXPECT_EQ(model_->num_mobilizers(), 2);
  // Check that frames are associated with the correct bodies.
  EXPECT_EQ(
      elbow_inboard_frame_->body().index(), upper_link_->index());
  EXPECT_EQ(
      elbow_outboard_frame_->body().index(), lower_link_->index());
  // Checks that mobilizers connect the right frames.
  EXPECT_EQ(elbow_mobilizer_->inboard_frame().index(),
            elbow_inboard_frame_->index());
  EXPECT_EQ(elbow_mobilizer_->outboard_frame().index(),
            elbow_outboard_frame_->index());
  // Checks that mobilizers connect the right bodies.
  EXPECT_EQ(elbow_mobilizer_->inboard_body().index(),
            upper_link_->index());
  EXPECT_EQ(elbow_mobilizer_->outboard_body().index(),
            lower_link_->index());
  // Checks we can retrieve the body associated with a frame.
  EXPECT_EQ(&elbow_inboard_frame_->body(), upper_link_);
  EXPECT_EQ(&elbow_outboard_frame_->body(), lower_link_);
  // Checks we can request inboard/outboard bodies to a mobilizer.
  EXPECT_EQ(&elbow_mobilizer_->inboard_body(), upper_link_);
  EXPECT_EQ(&elbow_mobilizer_->outboard_body(), lower_link_);
  // Request revolute mobilizers' axes.
  EXPECT_EQ(elbow_mobilizer_->revolute_axis(), Vector3d::UnitZ());
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
  EXPECT_EQ(shoulder_inboard_frame_->index(), FrameIndex(0));
  EXPECT_EQ(upper_link_->body_frame().index(), FrameIndex(1));
  EXPECT_EQ(lower_link_->body_frame().index(), FrameIndex(2));
  EXPECT_EQ(shoulder_outboard_frame_->index(), FrameIndex(3));
  EXPECT_EQ(elbow_inboard_frame_->index(), FrameIndex(4));
  EXPECT_EQ(elbow_outboard_frame_->index(), FrameIndex(2));
  // Verifies the elbow's outboard frame IS the lower link's frame.
  EXPECT_EQ(elbow_outboard_frame_->index(),
            lower_link_->body_frame().index());
}

// Asserts that the Finalize() stage is successful and that re-finalization is
// not allowed.
TEST_F(PendulumTests, Finalize) {
  CreatePendulumModel();
  // Finalize() stage.
  EXPECT_FALSE(model_->topology_is_valid());  // Not valid before Finalize().
  DRAKE_EXPECT_NO_THROW(model_->Finalize());
  EXPECT_TRUE(model_->topology_is_valid());  // Valid after Finalize().

  // Asserts that no more multibody elements can be added after finalize.
  SpatialInertia<double> M_Bo_B;
  EXPECT_THROW(model_->AddBody<RigidBody>(M_Bo_B), std::logic_error);
  EXPECT_THROW(
      model_->AddFrame<FixedOffsetFrame>(*lower_link_, X_LEo_),
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
  EXPECT_EQ(model_->num_bodies(), 1);
  // And there is only one frame, the world frame.
  EXPECT_EQ(model_->num_frames(), 1);
  CreatePendulumModel();

  // Vector of references.
  vector<std::reference_wrapper<const Body<double>>> bodies;
  bodies.push_back(*world_body_);
  bodies.push_back(*upper_link_);
  bodies.push_back(*lower_link_);

  // Verify that vector "bodies" effectively holds valid references to the
  // actual body elements in the tree.
  // In addition, since these tests compare actual memory addresses, they
  // ensure that bodies were not copied instead.
  // Unfortunately we need the ugly get() method since operator.() is not
  // overloaded.
  EXPECT_EQ(&bodies[world_body_->index()].get(), world_body_);
  EXPECT_EQ(&bodies[upper_link_->index()].get(), upper_link_);
  EXPECT_EQ(&bodies[lower_link_->index()].get(), lower_link_);
}

TEST_F(PendulumTests, CreateContext) {
  CreatePendulumModel();

  // Verifies the number of multibody elements is correct. In this case:
  // - world_
  // - upper_link_
  // - lower_link_
  EXPECT_EQ(model_->num_bodies(), 3);

  // Finalize() stage.
  DRAKE_EXPECT_NO_THROW(model_->Finalize());
  EXPECT_TRUE(model_->topology_is_valid());  // Valid after Finalize().

  // Create Context.
  MultibodyTreeSystem<double> system(std::move(model_));
  std::unique_ptr<Context<double>> context;
  DRAKE_EXPECT_NO_THROW(context = system.CreateDefaultContext());

  // Tests MultibodyTree state accessors.
  const auto& tree = GetInternalTree(system);

  // Verifies the correct number of generalized positions and velocities.
  EXPECT_EQ(tree.get_positions(*context).size(), 2);
  EXPECT_EQ(tree.GetMutablePositions(&*context).size(), 2);
  EXPECT_EQ(tree.get_velocities(*context).size(), 2);
  EXPECT_EQ(tree.GetMutableVelocities(&*context).size(), 2);

  // Verifies methods to retrieve fixed-sized segments of the state.
  EXPECT_EQ(tree.get_state_segment<1>(*context, 1).size(), 1);
  EXPECT_EQ(tree.GetMutableStateSegment<1>(&*context, 1).size(), 1);

  // Set the poses of each body in the position kinematics cache to have an
  // arbitrary value that we can use for unit testing. In practice the poses in
  // the position kinematics will be the result of a position kinematics update
  // and will live in the context as a cache entry.
  PositionKinematicsCache<double> pc(tree.get_topology());
  SetPendulumPoses(&pc);

  // Retrieve body poses from position kinematics cache.
  const RigidTransformd X_WW =
      get_body_pose_in_world(tree, pc, *world_body_);
  const RigidTransformd X_WLu =
      get_body_pose_in_world(tree, pc, *upper_link_);

  // Asserts that the retrieved poses match with the ones specified by the unit
  // test method SetPendulumPoses().
  EXPECT_TRUE(X_WW.GetAsMatrix4().isApprox(Matrix4d::Identity()));
  EXPECT_TRUE(X_WLu.GetAsMatrix34().isApprox(X_WL_.GetAsMatrix34()));
}

// Unit test fixture to verify the correctness of MultibodyTree methods for
// computing kinematics. This fixture uses the reference solution provided by
// benchmarks::Acrobot.
class PendulumKinematicTests : public PendulumTests {
 public:
  void SetUp() override {
    PendulumTests::SetUp();
    CreatePendulumModel();
    system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(model_));
    context_ = system_->CreateDefaultContext();

    // Only for testing, in this case we do know our Joint model IS a
    // RevoluteMobilizer.
    elbow_mobilizer_ = JointTester::get_mobilizer(*elbow_joint_);
  }

  /// Verifies that we can compute the mass matrix of the system using inverse
  /// dynamics.
  /// The result from inverse dynamics is the vector of generalized forces:
  ///   tau = M(q) * vdot + C(q, v) * v
  /// where q and v are the generalized positions and velocities, respectively.
  /// When v = 0 the Coriolis and gyroscopic forces term C(q, v) * v is zero.
  /// Therefore the i-th column of M(q) can be obtained performing inverse
  /// dynamics with an acceleration vector vdot = e_i, with e_i the i-th vector
  /// in the standard basis of ℝ²:
  ///   tau = Hi(q) = M(q) * e_i
  /// where Hi(q) is the i-th column in M(q).
  ///
  /// The solution is verified against the independent benchmark from
  /// drake::multibody::benchmarks::Acrobot.
  void VerifyMassMatrixViaInverseDynamics(
      double shoulder_angle, double elbow_angle) {
    shoulder_mobilizer_->set_angle(context_.get(), shoulder_angle);
    elbow_mobilizer_->set_angle(context_.get(), elbow_angle);

    Matrix2d H;
    tree().CalcMassMatrixViaInverseDynamics(*context_, &H);

    Matrix2d H_expected = acrobot_benchmark_.CalcMassMatrix(elbow_angle);
    EXPECT_TRUE(H.isApprox(H_expected, 5 * kEpsilon));
  }

  /// Verifies the results from MultibodyTree::CalcInverseDynamics() for a
  /// number of state configurations against the independently coded
  /// implementation in drake::multibody::benchmarks::Acrobot.
  void VerifyCoriolisTermViaInverseDynamics(
      double shoulder_angle, double elbow_angle) {
    const double kTolerance = 5 * kEpsilon;

    shoulder_mobilizer_->set_angle(context_.get(), shoulder_angle);
    elbow_mobilizer_->set_angle(context_.get(), elbow_angle);

    double shoulder_rate, elbow_rate;
    Vector2d C;
    Vector2d C_expected;

    // C(q, v) = 0 for v = 0.
    shoulder_rate = 0.0;
    elbow_rate = 0.0;
    shoulder_mobilizer_->set_angular_rate(context_.get(), shoulder_rate);
    elbow_mobilizer_->set_angular_rate(context_.get(), elbow_rate);
    tree().CalcBiasTerm(*context_, &C);
    C_expected = acrobot_benchmark_.CalcCoriolisVector(
            shoulder_angle, elbow_angle, shoulder_rate, elbow_rate);
    EXPECT_TRUE(CompareMatrices(
        C, C_expected, kTolerance, MatrixCompareType::relative));

    // First column of C(q, e_1) times e_1.
    shoulder_rate = 1.0;
    elbow_rate = 0.0;
    shoulder_mobilizer_->set_angular_rate(context_.get(), shoulder_rate);
    elbow_mobilizer_->set_angular_rate(context_.get(), elbow_rate);
    tree().CalcBiasTerm(*context_, &C);
    C_expected = acrobot_benchmark_.CalcCoriolisVector(
        shoulder_angle, elbow_angle, shoulder_rate, elbow_rate);
    EXPECT_TRUE(CompareMatrices(
        C, C_expected, kTolerance, MatrixCompareType::relative));

    // Second column of C(q, e_2) times e_2.
    shoulder_rate = 0.0;
    elbow_rate = 1.0;
    shoulder_mobilizer_->set_angular_rate(context_.get(), shoulder_rate);
    elbow_mobilizer_->set_angular_rate(context_.get(), elbow_rate);
    tree().CalcBiasTerm(*context_, &C);
    C_expected = acrobot_benchmark_.CalcCoriolisVector(
        shoulder_angle, elbow_angle, shoulder_rate, elbow_rate);
    EXPECT_TRUE(CompareMatrices(
        C, C_expected, kTolerance, MatrixCompareType::relative));

    // Both velocities are non-zero.
    shoulder_rate = 1.0;
    elbow_rate = 1.0;
    shoulder_mobilizer_->set_angular_rate(context_.get(), shoulder_rate);
    elbow_mobilizer_->set_angular_rate(context_.get(), elbow_rate);
    tree().CalcBiasTerm(*context_, &C);
    C_expected = acrobot_benchmark_.CalcCoriolisVector(
        shoulder_angle, elbow_angle, shoulder_rate, elbow_rate);
    EXPECT_TRUE(CompareMatrices(
        C, C_expected, kTolerance, MatrixCompareType::relative));
  }

  /// This method verifies the correctness of
  /// MultibodyTree::CalcForceElementsContribution() to compute the vector of
  /// generalized forces due to gravity.
  /// Generalized forces due to gravity are a function of positions only and are
  /// denoted by tau_g(q).
  /// The solution is verified against the independent benchmark from
  /// drake::multibody::benchmarks::Acrobot.
  Vector2d VerifyGravityTerm(
      const Eigen::Ref<const VectorXd>& q) const {
    DRAKE_DEMAND(q.size() == tree().num_positions());

    // This is the minimum factor of the machine precision within which these
    // tests pass. This factor incorporates an additional factor of two (2) to
    // be on the safe side on other architectures (particularly in Macs).
    const int kEpsilonFactor = 20;
    const double kTolerance = kEpsilonFactor * kEpsilon;

    const double shoulder_angle =  q(0);
    const double elbow_angle =  q(1);

    PositionKinematicsCache<double> pc(tree().get_topology());
    VelocityKinematicsCache<double> vc(tree().get_topology());
    // Even though tau_g(q) only depends on positions, other velocity dependent
    // forces (for instance damping) could depend on velocities. Therefore we
    // set the velocity kinematics cache entries to zero so that only tau_g(q)
    // gets computed (at least for this pendulum model that only includes
    // gravity and damping).
    vc.InitializeToZero();

    // ======================================================================
    // Compute position kinematics.
    shoulder_mobilizer_->set_angle(context_.get(), shoulder_angle);
    elbow_joint_->set_angle(context_.get(), elbow_angle);
    tree().CalcPositionKinematicsCache(*context_, &pc);

    // ======================================================================
    // The force of gravity gets included in this call since we have
    // UniformGravityFieldElement in the model.
    // Applied forcing:
    MultibodyForces<double> forcing(tree());
    tree().CalcForceElementsContribution(*context_, pc, vc, &forcing);

    // ======================================================================
    // To get generalized forces, compute inverse dynamics applying the forces
    // computed by CalcForceElementsContribution().

    // Output vector of generalized forces.
    VectorXd tau(tree().num_velocities());

    // Output vector of spatial forces for each body B at their inboard
    // frame Mo, expressed in the world W.
    vector<SpatialForce<double>> F_BMo_W_array(tree().num_bodies());

    // ======================================================================
    // Compute expected values using the acrobot benchmark.
    const Vector2d tau_g_expected = acrobot_benchmark_.CalcGravityVector(
        shoulder_angle, elbow_angle);

    // ======================================================================
    // Notice that we do not need to allocate extra memory since both
    // Fapplied_Bo_W_array and tau can be used as input and output arguments.
    // However, the data given at input is lost on output. A user might choose
    // then to have separate input/output arrays.

    const VectorXd vdot = VectorXd::Zero(tree().num_velocities());
    vector<SpatialAcceleration<double>> A_WB_array(tree().num_bodies());

    // Aliases to external forcing arrays:
    std::vector<SpatialForce<double>>& Fapplied_Bo_W_array =
        forcing.mutable_body_forces();
    VectorX<double>& tau_applied = forcing.mutable_generalized_forces();

    // Try first using different arrays for input/output:
    // Initialize output to garbage, it should not affect the results.
    tau.setConstant(std::numeric_limits<double>::quiet_NaN());
    tau_applied.setZero();
    tree().CalcInverseDynamics(
        *context_, vdot, Fapplied_Bo_W_array, tau_applied,
        &A_WB_array, &F_BMo_W_array, &tau);
    // The result from inverse dynamics must be tau = -tau_g(q).
    EXPECT_TRUE(tau.isApprox(-tau_g_expected, kTolerance));

    // Now try using the same arrays for input/output (input data
    // Fapplied_Bo_W_array will get overwritten through the output argument).
    tau_applied.setZero();  // This will now get overwritten.
    tree().CalcInverseDynamics(
        *context_, vdot, Fapplied_Bo_W_array, tau_applied,
        &A_WB_array, &Fapplied_Bo_W_array, &tau_applied);
    // The result from inverse dynamics must be tau = -tau_g(q).
    EXPECT_TRUE(tau.isApprox(-tau_g_expected, kTolerance));

    // Compute the system's potential energy:
    const double V_expected =
        acrobot_benchmark_.CalcPotentialEnergy(shoulder_angle, elbow_angle);
    const double V = tree().CalcPotentialEnergy(*context_);
    EXPECT_NEAR(V, V_expected, kTolerance);

    return tau;
  }

  /// Given the transformation `X_AB` between two frames A and B and its time
  /// derivative in frame A `Xdot_AB`, this method computes the spatial velocity
  /// `V_AB` of frame B as measured and expressed in A.
  static SpatialVelocity<double> ComputeSpatialVelocityFromXdot(
      const Matrix4d& X_AB, const Matrix4d& X_AB_dot) {
    const Matrix3d R_AB = X_AB.topLeftCorner(3, 3);
    const Matrix3d R_AB_dot = X_AB_dot.topLeftCorner(3, 3);
    // Compute cross product matrix w_ABx = [w_AB].
    Matrix3d w_ABx = R_AB_dot * R_AB.transpose();
    // Take the average to take into account both upper and lower parts.
    w_ABx = (w_ABx - w_ABx.transpose()) / 2.0;
    // Extract angular velocity vector.
    Vector3d w_AB(w_ABx(2, 1), w_ABx(0, 2), w_ABx(1, 0));
    // Extract linear velocity vector.
    Vector3d v_AB = X_AB_dot.col(3).head(3);
    return SpatialVelocity<double>(w_AB, v_AB);
  }

  const MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

 protected:
  std::unique_ptr<MultibodyTreeSystem<double>> system_;
  std::unique_ptr<Context<double>> context_;
  // Reference benchmark for verification.
  Acrobot<double> acrobot_benchmark_{
      Vector3d::UnitZ() /* Plane normal */, Vector3d::UnitY() /* Up vector */,
      link1_mass_, link2_mass_,
      link1_length_, link2_length_, half_link1_length_, half_link2_length_,
      link1_Ic_, link2_Ic_, 0.0, 0.0, acceleration_of_gravity_};

 private:
  // This method verifies the correctness of
  // MultibodyTree::CalcInverseDynamics() to compute the generalized forces that
  // would need to be applied in order to attain the generalized accelerations
  // vdot.
  // The generalized accelerations are given by:
  //   tau = M(q) * vdot + C(q, v) * v
  // where q and v are the generalized positions and velocities, respectively.
  // These, together with the generalized accelerations vdot are inputs to this
  // method.
  // The solution is verified against the independent benchmark from
  // drake::multibody::benchmarks::Acrobot.
  Vector2d VerifyInverseDynamics(
      const Eigen::Ref<const VectorXd>& q,
      const Eigen::Ref<const VectorXd>& v,
      const Eigen::Ref<const VectorXd>& vdot) const {
    DRAKE_DEMAND(q.size() == tree().num_positions());
    DRAKE_DEMAND(v.size() == tree().num_velocities());
    DRAKE_DEMAND(vdot.size() == tree().num_velocities());

    // This is the minimum factor of the machine precision within which these
    // tests pass. This factor incorporates an additional factor of two (2) to
    // be on the safe side on other architectures (particularly in Macs).
    const int kEpsilonFactor = 30;
    const double kTolerance = kEpsilonFactor * kEpsilon;

    const double shoulder_angle =  q(0);
    const double elbow_angle =  q(1);

    const double shoulder_angle_rate = v(0);
    const double elbow_angle_rate = v(1);

    PositionKinematicsCache<double> pc(tree().get_topology());
    VelocityKinematicsCache<double> vc(tree().get_topology());

    // ======================================================================
    // Compute position kinematics.
    shoulder_mobilizer_->set_angle(context_.get(), shoulder_angle);
    elbow_joint_->set_angle(context_.get(), elbow_angle);
    tree().CalcPositionKinematicsCache(*context_, &pc);

    // ======================================================================
    // Compute velocity kinematics.
    shoulder_mobilizer_->set_angular_rate(context_.get(), shoulder_angle_rate);
    elbow_joint_->set_angular_rate(context_.get(), elbow_angle_rate);
    tree().CalcVelocityKinematicsCache(*context_, pc, &vc);

    // ======================================================================
    // Compute inverse dynamics.
    VectorXd tau(tree().num_velocities());
    vector<SpatialAcceleration<double>> A_WB_array(tree().num_bodies());
    vector<SpatialForce<double>> F_BMo_W_array(tree().num_bodies());
    tree().CalcInverseDynamics(*context_, vdot, {}, VectorXd(),
                                &A_WB_array, &F_BMo_W_array, &tau);

    // ======================================================================
    // Compute acceleration kinematics.
    AccelerationKinematicsCache<double> ac(tree().get_topology());
    tree().CalcAccelerationKinematicsCache(*context_, pc, vc, vdot, &ac);

    // From acceleration kinematics.
    const SpatialAcceleration<double>& A_WUcm_ac =
        get_body_spatial_acceleration_in_world(tree(), ac, *upper_link_);
    const SpatialAcceleration<double>& A_WL_ac =
        get_body_spatial_acceleration_in_world(tree(), ac, *lower_link_);
    // From inverse dynamics.
    const SpatialAcceleration<double>& A_WUcm_id =
        A_WB_array[upper_link_->node_index()];
    const SpatialAcceleration<double>& A_WL_id =
        A_WB_array[lower_link_->node_index()];
    EXPECT_TRUE(A_WUcm_id.IsApprox(A_WUcm_ac, kTolerance));
    EXPECT_TRUE(A_WL_id.IsApprox(A_WL_ac, kTolerance));

    // ======================================================================
    // Compute expected values using the acrobot benchmark.
    const Vector2d C_expected = acrobot_benchmark_.CalcCoriolisVector(
        shoulder_angle, elbow_angle, shoulder_angle_rate, elbow_angle_rate);
    const Matrix2d H = acrobot_benchmark_.CalcMassMatrix(elbow_angle);
    const Vector2d tau_expected = H * vdot + C_expected;

    EXPECT_TRUE(CompareMatrices(tau, tau_expected, kTolerance,
                                MatrixCompareType::relative));
    return tau;
  }
};

// Verify the correctness of method MultibodyTree::CalcPositionKinematicsCache()
// comparing the computed results the reference solution provided by
// benchmarks::Acrobot.
TEST_F(PendulumKinematicTests, CalcPositionKinematics) {
  // This is the minimum factor of the machine precision within which these
  // tests pass.
  const int kEpsilonFactor = 3;
  const double kTolerance = kEpsilonFactor * kEpsilon;

  // By default CreateDefaultContext() sets mobilizer to their zero
  // configuration.
  EXPECT_EQ(shoulder_mobilizer_->get_angle(*context_), 0.0);
  EXPECT_EQ(elbow_joint_->get_angle(*context_), 0.0);

  // Test mobilizer's setter/getters.
  shoulder_mobilizer_->set_angle(context_.get(), M_PI);
  EXPECT_EQ(shoulder_mobilizer_->get_angle(*context_), M_PI);
  shoulder_mobilizer_->set_zero_state(*context_,
                                      &context_->get_mutable_state());
  EXPECT_EQ(shoulder_mobilizer_->get_angle(*context_), 0.0);

  PositionKinematicsCache<double> pc(tree().get_topology());

  const int num_angles = 50;
  const double kDeltaAngle = 2 * M_PI / (num_angles - 1.0);
  for (double ishoulder = 0; ishoulder < num_angles; ++ishoulder) {
    const double shoulder_angle = -M_PI + ishoulder * kDeltaAngle;
    for (double ielbow = 0; ielbow < num_angles; ++ielbow) {
      const double elbow_angle = -M_PI + ielbow * kDeltaAngle;

      shoulder_mobilizer_->set_angle(context_.get(), shoulder_angle);
      EXPECT_EQ(shoulder_mobilizer_->get_angle(*context_), shoulder_angle);
      elbow_joint_->set_angle(context_.get(), elbow_angle);
      EXPECT_EQ(elbow_joint_->get_angle(*context_), elbow_angle);

      // Verify this matches the corresponding entries in the context.
      EXPECT_EQ(tree().get_positions(*context_)(0), shoulder_angle);
      EXPECT_EQ(tree().get_positions(*context_)(1), elbow_angle);

      tree().CalcPositionKinematicsCache(*context_, &pc);

      // Indexes to the BodyNode objects associated with each mobilizer.
      const BodyNodeIndex shoulder_node =
          shoulder_mobilizer_->get_topology().body_node;
      const BodyNodeIndex elbow_node =
          elbow_mobilizer_->get_topology().body_node;

      // Expected poses of the outboard frames measured in the inboard frame.
      RigidTransformd X_SiSo(RotationMatrixd::MakeZRotation(shoulder_angle));
      RigidTransformd X_EiEo(RotationMatrixd::MakeZRotation(elbow_angle));

      // Verify the values in the position kinematics cache.
      EXPECT_TRUE(
          pc.get_X_FM(shoulder_node).IsNearlyEqualTo(X_SiSo, kTolerance));
      EXPECT_TRUE(pc.get_X_FM(elbow_node).IsNearlyEqualTo(X_EiEo, kTolerance));

      // Verify that both, const and mutable versions point to the same address.
      EXPECT_EQ(&pc.get_X_FM(shoulder_node),
                &pc.get_mutable_X_FM(shoulder_node));
      EXPECT_EQ(&pc.get_X_FM(elbow_node),
                &pc.get_mutable_X_FM(elbow_node));

      // Retrieve body poses from position kinematics cache.
      const RigidTransformd X_WW =
          get_body_pose_in_world(tree(), pc, *world_body_);
      const RigidTransformd X_WU =
          get_body_pose_in_world(tree(), pc, *upper_link_);
      const RigidTransformd X_WL =
          get_body_pose_in_world(tree(), pc, *lower_link_);

      const RigidTransformd X_WU_expected =
          acrobot_benchmark_.CalcLink1PoseInWorldFrame(shoulder_angle);

      const RigidTransformd X_WL_expected =
          acrobot_benchmark_.CalcElbowOutboardFramePoseInWorldFrame(
              shoulder_angle, elbow_angle);

      // Asserts that the retrieved poses match with the ones specified by the
      // unit test method SetPendulumPoses().
      EXPECT_TRUE(CompareMatrices(X_WW.GetAsMatrix4(),
                                  Matrix4d::Identity(),
                                  kTolerance, MatrixCompareType::relative));
      EXPECT_TRUE(CompareMatrices(X_WU.GetAsMatrix34(),
                                  X_WU_expected.GetAsMatrix34(),
                                  kTolerance, MatrixCompareType::relative));
      EXPECT_TRUE(CompareMatrices(X_WL.GetAsMatrix34(),
                                  X_WL_expected.GetAsMatrix34(),
                                  kTolerance, MatrixCompareType::relative));
    }
  }
}

TEST_F(PendulumKinematicTests, CalcVelocityAndAccelerationKinematics) {
  // This is the minimum factor of the machine precision within which these
  // tests pass. There is an additional factor of two (2) to be on the safe side
  // on other architectures (particularly in Macs).
  const int kEpsilonFactor = 30;
  const double kTolerance = kEpsilonFactor * kEpsilon;

  PositionKinematicsCache<double> pc(tree().get_topology());
  VelocityKinematicsCache<double> vc(tree().get_topology());
  AccelerationKinematicsCache<double> ac(tree().get_topology());

  const int num_angles = 50;
  const double kDeltaAngle = 2 * M_PI / (num_angles - 1.0);
  for (double ishoulder = 0; ishoulder < num_angles; ++ishoulder) {
    const double shoulder_angle = -M_PI + ishoulder * kDeltaAngle;
    for (double ielbow = 0; ielbow < num_angles; ++ielbow) {
      const double elbow_angle = -M_PI + ielbow * kDeltaAngle;

      // ======================================================================
      // Compute position kinematics.
      shoulder_mobilizer_->set_angle(context_.get(), shoulder_angle);
      elbow_joint_->set_angle(context_.get(), elbow_angle);
      tree().CalcPositionKinematicsCache(*context_, &pc);

      // Obtain the lower link center of mass to later shift its computed
      // spatial velocity and acceleration to the center of mass frame for
      // comparison with the benchmark.
      const RigidTransformd X_WL =
          get_body_pose_in_world(tree(), pc, *lower_link_);
      const RotationMatrixd R_WL = X_WL.rotation();
      const Vector3d p_LoLcm_L = lower_link_->default_com();
      const Vector3d p_LoLcm_W = R_WL * p_LoLcm_L;

      // ======================================================================
      // Compute velocity kinematics

      // Set the shoulder's angular velocity.
      const double shoulder_angle_rate = 1.0;
      shoulder_mobilizer_->set_angular_rate(context_.get(),
                                            shoulder_angle_rate);
      EXPECT_EQ(shoulder_mobilizer_->get_angular_rate(*context_),
                shoulder_angle_rate);

      // Set the elbow's angular velocity.
      const double elbow_angle_rate = -0.5;
      elbow_joint_->set_angular_rate(context_.get(), elbow_angle_rate);
      EXPECT_EQ(elbow_joint_->get_angular_rate(*context_), elbow_angle_rate);
      tree().CalcVelocityKinematicsCache(*context_, pc, &vc);

      // Retrieve body spatial velocities from velocity kinematics cache.
      const SpatialVelocity<double>& V_WUcm =
          get_body_spatial_velocity_in_world(tree(), vc, *upper_link_);
      const SpatialVelocity<double>& V_WL =
          get_body_spatial_velocity_in_world(tree(), vc, *lower_link_);
      // Obtain the lower link's center of mass frame spatial velocity by
      // shifting V_WL:
      const SpatialVelocity<double> V_WLcm = V_WL.Shift(p_LoLcm_W);

      const SpatialVelocity<double> V_WUcm_expected(
          acrobot_benchmark_.CalcLink1SpatialVelocityInWorldFrame(
              shoulder_angle, shoulder_angle_rate));
      const SpatialVelocity<double> V_WLcm_expected(
          acrobot_benchmark_.CalcLink2SpatialVelocityInWorldFrame(
              shoulder_angle, elbow_angle,
              shoulder_angle_rate, elbow_angle_rate));

      EXPECT_TRUE(V_WUcm.IsApprox(V_WUcm_expected, kTolerance));
      EXPECT_TRUE(V_WLcm.IsApprox(V_WLcm_expected, kTolerance));

      // ======================================================================
      // Compute acceleration kinematics
      // Test a number of acceleration configurations.
      // For zero vdot:
      VectorX<double> vdot(2);  // Vector of generalized accelerations.
      vdot = VectorX<double>::Zero(2);

      tree().CalcAccelerationKinematicsCache(*context_, pc, vc, vdot, &ac);

      // Retrieve body spatial accelerations from acceleration kinematics cache.
      SpatialAcceleration<double> A_WUcm =
          get_body_spatial_acceleration_in_world(tree(), ac, *upper_link_);
      SpatialAcceleration<double> A_WL =
          get_body_spatial_acceleration_in_world(tree(), ac, *lower_link_);
      // Obtain the lower link's center of mass frame spatial acceleration by
      // shifting A_WL:
      const Vector3d& w_WL = V_WL.rotational();
      SpatialAcceleration<double> A_WLcm = A_WL.Shift(p_LoLcm_W, w_WL);

      SpatialAcceleration<double> A_WUcm_expected(
          acrobot_benchmark_.CalcLink1SpatialAccelerationInWorldFrame(
              shoulder_angle, shoulder_angle_rate, vdot(0)));

      SpatialAcceleration<double> A_WLcm_expected(
          acrobot_benchmark_.CalcLink2SpatialAccelerationInWorldFrame(
              shoulder_angle, elbow_angle,
              shoulder_angle_rate, elbow_angle_rate,
              vdot(0), vdot(1)));

      EXPECT_TRUE(A_WUcm.IsApprox(A_WUcm_expected, kTolerance));
      EXPECT_TRUE(A_WLcm.IsApprox(A_WLcm_expected, kTolerance));

      // For a non-zero vdot [rad/sec^2]:
      shoulder_mobilizer_->get_mutable_accelerations_from_array(
          &vdot)(0) = -1.0;
      elbow_mobilizer_->get_mutable_accelerations_from_array(&vdot)(0) = 2.0;
      EXPECT_EQ(
          shoulder_mobilizer_->get_accelerations_from_array(vdot).size(), 1);
      EXPECT_EQ(
          shoulder_mobilizer_->get_accelerations_from_array(vdot)(0), -1.0);
      EXPECT_EQ(
          elbow_mobilizer_->get_accelerations_from_array(vdot).size(), 1);
      EXPECT_EQ(
          elbow_mobilizer_->get_accelerations_from_array(vdot)(0), 2.0);

      tree().CalcAccelerationKinematicsCache(*context_, pc, vc, vdot, &ac);

      // Retrieve body spatial accelerations from acceleration kinematics cache.
      A_WUcm = get_body_spatial_acceleration_in_world(tree(), ac, *upper_link_);
      A_WL = get_body_spatial_acceleration_in_world(tree(), ac, *lower_link_);
      A_WLcm = A_WL.Shift(p_LoLcm_W, w_WL);

      A_WUcm_expected = SpatialAcceleration<double>(
          acrobot_benchmark_.CalcLink1SpatialAccelerationInWorldFrame(
              shoulder_angle, shoulder_angle_rate, vdot(0)));

      A_WLcm_expected = SpatialAcceleration<double>(
          acrobot_benchmark_.CalcLink2SpatialAccelerationInWorldFrame(
              shoulder_angle, elbow_angle,
              shoulder_angle_rate, elbow_angle_rate,
              vdot(0), vdot(1)));

      EXPECT_TRUE(A_WUcm.IsApprox(A_WUcm_expected, kTolerance));
      EXPECT_TRUE(A_WLcm.IsApprox(A_WLcm_expected, kTolerance));
    }
  }
}

// Compute the bias term containing Coriolis and gyroscopic effects for a
// number of different pendulum configurations.
// This is computed using inverse dynamics with vdot = 0.
TEST_F(PendulumKinematicTests, CoriolisTerm) {
  // C(q, v) should be zero when elbow_angle = 0 independent of the shoulder
  // angle.
  VerifyCoriolisTermViaInverseDynamics(0.0, 0.0);
  VerifyCoriolisTermViaInverseDynamics(M_PI / 3.0, 0.0);

  // Attempt a number of non-zero elbow angles.
  VerifyCoriolisTermViaInverseDynamics(0.0, M_PI / 2.0);
  VerifyCoriolisTermViaInverseDynamics(0.0, M_PI / 3.0);
  VerifyCoriolisTermViaInverseDynamics(0.0, M_PI / 4.0);

  // Repeat previous tests but this time with different non-zero values of the
  // shoulder angle. Results should be independent of the shoulder angle for
  // this double pendulum system.
  VerifyCoriolisTermViaInverseDynamics(M_PI / 3.0, M_PI / 2.0);
  VerifyCoriolisTermViaInverseDynamics(M_PI / 3.0, M_PI / 3.0);
  VerifyCoriolisTermViaInverseDynamics(M_PI / 3.0, M_PI / 4.0);
}

// Compute the mass matrix using the inverse dynamics method.
TEST_F(PendulumKinematicTests, MassMatrix) {
  VerifyMassMatrixViaInverseDynamics(0.0, 0.0);
  VerifyMassMatrixViaInverseDynamics(0.0, M_PI / 2.0);
  VerifyMassMatrixViaInverseDynamics(0.0, M_PI / 3.0);
  VerifyMassMatrixViaInverseDynamics(0.0, M_PI / 4.0);

  // For the double pendulum system it turns out that the mass matrix is only a
  // function of the elbow angle, independent of the shoulder angle.
  // Therefore M(q) = H(elbow_angle). We therefore run the same previous tests
  // with different shoulder angles to verify this is true.
  VerifyMassMatrixViaInverseDynamics(M_PI / 3.0, 0.0);
  VerifyMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 2.0);
  VerifyMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 3.0);
  VerifyMassMatrixViaInverseDynamics(M_PI / 3.0, M_PI / 4.0);
}

// A test to compute generalized forces due to gravity.
TEST_F(PendulumKinematicTests, GravityTerm) {
  // A list of conditions used for testing.
  std::vector<Vector2d> test_matrix;

  test_matrix.push_back({0.0, 0.0});
  test_matrix.push_back({0.0, M_PI / 2.0});
  test_matrix.push_back({0.0, M_PI / 3.0});
  test_matrix.push_back({0.0, M_PI / 4.0});

  test_matrix.push_back({M_PI / 2.0, M_PI / 2.0});
  test_matrix.push_back({M_PI / 2.0, M_PI / 3.0});
  test_matrix.push_back({M_PI / 2.0, M_PI / 4.0});

  test_matrix.push_back({M_PI / 3.0, M_PI / 2.0});
  test_matrix.push_back({M_PI / 3.0, M_PI / 3.0});
  test_matrix.push_back({M_PI / 3.0, M_PI / 4.0});

  test_matrix.push_back({M_PI / 4.0, M_PI / 2.0});
  test_matrix.push_back({M_PI / 4.0, M_PI / 3.0});
  test_matrix.push_back({M_PI / 4.0, M_PI / 4.0});

  for (const Vector2d& q : test_matrix) {
    VerifyGravityTerm(q);
  }
}

// Compute the spatial velocity of each link as measured in the world frame
// using automatic differentiation through
// MultibodyTree::CalcPositionKinematicsCache(). The results are verified
// comparing with the reference solution provided by benchmarks::Acrobot.
TEST_F(PendulumKinematicTests, CalcVelocityKinematicsWithAutoDiffXd) {
  // This is the minimum factor of the machine precision within which these
  // tests pass.
  const int kEpsilonFactor = 100;
  const double kTolerance = kEpsilonFactor * kEpsilon;

  std::unique_ptr<MultibodyTree<AutoDiffXd>> model_autodiff =
      tree().ToAutoDiffXd();
  const MultibodyTree<AutoDiffXd>& tree_autodiff = *model_autodiff.get();

  const RevoluteMobilizer<AutoDiffXd>& shoulder_mobilizer_autodiff =
      model_autodiff->get_variant(*shoulder_mobilizer_);

  const RevoluteJoint<AutoDiffXd>& elbow_joint_autodiff =
      model_autodiff->get_variant(*elbow_joint_);

  const RigidBody<AutoDiffXd>& upper_link_autodiff =
      model_autodiff->get_variant(*upper_link_);
  const RigidBody<AutoDiffXd>& lower_link_autodiff =
      model_autodiff->get_variant(*lower_link_);

  MultibodyTreeSystem<AutoDiffXd> tree_system_autodiff(
      std::move(model_autodiff));
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff =
      tree_system_autodiff.CreateDefaultContext();

  PositionKinematicsCache<AutoDiffXd> pc(tree_autodiff.get_topology());

  const int num_angles = 50;
  const double kDeltaAngle = 2 * M_PI / (num_angles - 1.0);

  const int num_velocities = 2;
  const double w_WU_min = -1.0;
  const double w_WU_max = 1.0;
  const double w_UL_min = -0.5;
  const double w_UL_max = 0.5;

  const double kDelta_w_WU = (w_WU_max - w_WU_min) / (num_velocities - 1.0);
  const double kDelta_w_UL = (w_UL_max - w_UL_min) / (num_velocities - 1.0);

  // Loops over angular velocities.
  for (int iw_shoulder = 0; iw_shoulder < num_velocities; ++iw_shoulder) {
    const double w_WU = w_WU_min + iw_shoulder * kDelta_w_WU;
    for (int iw_elbow = 0; iw_elbow < num_velocities; ++iw_elbow) {
      const double w_UL = w_UL_min + iw_elbow * kDelta_w_UL;

      // Loops over angles.
      for (double iq_shoulder = 0; iq_shoulder < num_angles; ++iq_shoulder) {
        const AutoDiffXd shoulder_angle(
            -M_PI + iq_shoulder * kDeltaAngle, /* angle value */
            Vector1<double>::Constant(w_WU)  /* angular velocity */);
        for (double iq_elbow = 0; iq_elbow < num_angles; ++iq_elbow) {
          const AutoDiffXd elbow_angle(
              -M_PI + iq_elbow * kDeltaAngle,   /* angle value */
              Vector1<double>::Constant(w_UL) /* angular velocity */);

          // Update position kinematics.
          shoulder_mobilizer_autodiff.set_angle(context_autodiff.get(),
                                                shoulder_angle);
          elbow_joint_autodiff.set_angle(context_autodiff.get(), elbow_angle);
          tree_autodiff.CalcPositionKinematicsCache(*context_autodiff, &pc);

          // Retrieve body poses from position kinematics cache.
          const RigidTransform<AutoDiffXd> X_WU =
              get_body_pose_in_world(tree_autodiff, pc, upper_link_autodiff);
          const RigidTransform<AutoDiffXd> X_WL =
              get_body_pose_in_world(tree_autodiff, pc, lower_link_autodiff);

          const RigidTransformd X_WU_expected =
              acrobot_benchmark_.CalcLink1PoseInWorldFrame(
                  shoulder_angle.value());

          const RigidTransformd X_WL_expected =
              acrobot_benchmark_.CalcElbowOutboardFramePoseInWorldFrame(
                  shoulder_angle.value(), elbow_angle.value());

          // Extract the transformations' values.
          Eigen::MatrixXd X_WU_value = math::ExtractValue(X_WU.GetAsMatrix4());
          Eigen::MatrixXd X_WL_value = math::ExtractValue(X_WL.GetAsMatrix4());

          // Obtain the lower link center of mass to later shift its computed
          // spatial velocity to the center of mass frame for comparison with
          // the benchmark.
          const Matrix3d R_WL = X_WL_value.block<3, 3>(0, 0);
          const Vector3d p_LoLcm_L = lower_link_->default_com();
          const Vector3d p_LoLcm_W = R_WL * p_LoLcm_L;

          // Asserts that the retrieved poses match with the ones specified by
          // the unit test method SetPendulumPoses().
          EXPECT_TRUE(
              X_WU_value.isApprox(X_WU_expected.GetAsMatrix4(), kTolerance));
          EXPECT_TRUE(
              X_WL_value.isApprox(X_WL_expected.GetAsMatrix4(), kTolerance));

          // Extract the transformations' time derivatives.
          Eigen::MatrixXd X_WU_dot = math::ExtractGradient(X_WU.GetAsMatrix4());
          X_WU_dot.resize(4, 4);
          Eigen::MatrixXd X_WL_dot = math::ExtractGradient(X_WL.GetAsMatrix4());
          X_WL_dot.resize(4, 4);

          // Convert transformations' time derivatives to spatial velocities.
          SpatialVelocity<double> V_WUcm =
              ComputeSpatialVelocityFromXdot(X_WU_value, X_WU_dot);
          SpatialVelocity<double> V_WL =
              ComputeSpatialVelocityFromXdot(X_WL_value, X_WL_dot);
          // Obtain the lower link's center of mass frame spatial velocity by
          // shifting V_WL:
          const SpatialVelocity<double> V_WLcm = V_WL.Shift(p_LoLcm_W);

          const SpatialVelocity<double> V_WUcm_expected(
              acrobot_benchmark_.CalcLink1SpatialVelocityInWorldFrame(
                  shoulder_angle.value(), w_WU));
          const SpatialVelocity<double> V_WLcm_expected(
              acrobot_benchmark_.CalcLink2SpatialVelocityInWorldFrame(
                  shoulder_angle.value(), elbow_angle.value(), w_WU, w_UL));

          EXPECT_TRUE(V_WUcm.IsApprox(V_WUcm_expected, kTolerance));
          EXPECT_TRUE(V_WLcm.IsApprox(V_WLcm_expected, kTolerance));

          // Compute potential energy, and its time derivative.
          const AutoDiffXd V =
              tree_autodiff.CalcPotentialEnergy(*context_autodiff);
          const double V_value = V.value();
          const double V_expected =
              acrobot_benchmark_.CalcPotentialEnergy(
                  shoulder_angle.value(), elbow_angle.value());
          EXPECT_NEAR(V_value, V_expected, kTolerance);

          // Since in this case the only force is that of gravity, the time
          // derivative of the total potential energy must equal the total
          // conservative power.
          shoulder_mobilizer_->set_angle(
              context_.get(), shoulder_angle.value());
          shoulder_mobilizer_->set_angular_rate(
              context_.get(), shoulder_angle.derivatives()[0]);
          elbow_mobilizer_->set_angle(
              context_.get(), elbow_angle.value());
          elbow_mobilizer_->set_angular_rate(
              context_.get(), elbow_angle.derivatives()[0]);
          const double Pc = tree().CalcConservativePower(*context_);

          // Notice we define Pc = -d(Pc)/dt.
          const double Pc_from_autodiff = -V.derivatives()[0];
          EXPECT_NEAR(Pc, Pc_from_autodiff, kTolerance);
        }  // ielbow
      }  // ishoulder
    }  // iw_elbow
  }  // iw_shoulder
}

TEST_F(PendulumKinematicTests, PointsPositionsAndRelativeTransform) {
  // This is the minimum factor of the machine precision within which these
  // tests pass.
  const int kEpsilonFactor = 3;
  const double kTolerance = kEpsilonFactor * kEpsilon;

  shoulder_mobilizer_->set_angle(context_.get(), M_PI / 4.0);
  elbow_mobilizer_->set_angle(context_.get(), M_PI / 4.0);

  // Set of points Qi measured and expressed in the lower link's frame L.
  Matrix3X<double> p_LQi_set(3, 3);
  p_LQi_set.col(0) << 0.0, -2.0, 0.0;  // At the end effector.
  p_LQi_set.col(1) << 0.0, -1.0, 0.0;  // In the middle of the lower link.
  p_LQi_set.col(2) << 0.0,  0.0, 0.0;  // At the elbow.

  // World positions of the set of points Qi:
  Matrix3X<double> p_WQi_set(3, 3);
  tree().CalcPointsPositions(
      *context_,
      lower_link_->body_frame(), p_LQi_set,
      tree().world_frame(), &p_WQi_set);

  Matrix3X<double> p_WQi_set_expected(3, 3);
  p_WQi_set_expected.col(0) << 2.0 + M_SQRT1_2, -M_SQRT1_2, 0.0;
  p_WQi_set_expected.col(1) << 1.0 + M_SQRT1_2, -M_SQRT1_2, 0.0;
  p_WQi_set_expected.col(2) << M_SQRT1_2, -M_SQRT1_2, 0.0;

  EXPECT_TRUE(CompareMatrices(
      p_WQi_set, p_WQi_set_expected, kTolerance, MatrixCompareType::relative));

  // A scond set of points Pi but measured and expressed in the upper link's
  // frame U.
  Matrix3X<double> p_UPi_set(3, 3);
  p_UPi_set.col(0) << 0.0,  1.0, 0.0;  // Projecting 0.5 out from the shoulder.
  p_UPi_set.col(1) << 0.0,  0.0, 0.0;  // In the middle of the upper link.
  p_UPi_set.col(2) << 0.0, -1.0, 0.0;  // Projecting 0.5 out from the elbow.

  // World positions of the set of points Qi:
  Matrix3X<double> p_WPi_set(3, 3);
  tree().CalcPointsPositions(
      *context_,
      upper_link_->body_frame(), p_UPi_set,
      tree().world_frame(), &p_WPi_set);

  Matrix3X<double> p_WPi_set_expected(3, 3);
  p_WPi_set_expected.col(0) = 0.5 * Vector3d(-M_SQRT1_2, M_SQRT1_2, 0.0);
  p_WPi_set_expected.col(1) = -0.5 * Vector3d(-M_SQRT1_2, M_SQRT1_2, 0.0);
  p_WPi_set_expected.col(2) = -1.5 * Vector3d(-M_SQRT1_2, M_SQRT1_2, 0.0);

  EXPECT_TRUE(CompareMatrices(
      p_WPi_set, p_WPi_set_expected, kTolerance, MatrixCompareType::relative));

  const Vector3d p_UL_expected(0.0, -0.5, 0.0);
  const Matrix3d R_UL_expected(Eigen::AngleAxisd(M_PI_4, Vector3d::UnitZ()));

  // Verify the RotationMatrix and position returned by CalcRelativeTransform().
  const RigidTransformd X_UL(tree().CalcRelativeTransform(
      *context_, upper_link_->body_frame(), lower_link_->body_frame()));
  EXPECT_TRUE(CompareMatrices(X_UL.rotation().matrix(), R_UL_expected,
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(X_UL.translation(), p_UL_expected,
                              kTolerance, MatrixCompareType::relative));

  // Verify the RotationMatrix returned by CalcRelativeRotationMatrix().
  const RotationMatrixd R_UL = tree().CalcRelativeRotationMatrix(
      *context_, upper_link_->body_frame(), lower_link_->body_frame());
  EXPECT_TRUE(CompareMatrices(R_UL.matrix(), R_UL_expected,
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(PendulumKinematicTests, PointsHaveTheWrongSize) {
  shoulder_mobilizer_->set_angle(context_.get(), M_PI / 4.0);
  elbow_mobilizer_->set_angle(context_.get(), M_PI / 4.0);

  // Create a set of points with the wrong number of rows (it must be 3).
  // The values do not matter for this test, just the size.
  MatrixX<double> p_LQi_set = MatrixX<double>::Zero(4, 3);

  // World positions of the set of points Qi:
  Matrix3X<double> p_WQi_set(3, 3);
  EXPECT_THROW(tree().CalcPointsPositions(
      *context_,
      lower_link_->body_frame(), p_LQi_set,
      tree().world_frame(), &p_WQi_set), std::runtime_error);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
