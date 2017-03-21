/* clang-format off */
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
/* clang-format on */

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"

// This tests the *formula* for generating forces from contact. The current
// models use a continuous Stribeck function which models static and dynamic
// friction.

namespace drake {
namespace systems {
namespace {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::make_unique;
using std::unique_ptr;

typedef Vector6<double> Vector6d;

// Basic infrastructure for setting up and running contact tests.  Creates two
// spheres with a *fixed* penetration depth.  Variants of this test will
// modify the relative velocities.
class ContactFormulaTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto unique_tree = make_unique<RigidBodyTree<double>>();
    tree_ = unique_tree.get();

    Vector3d pos(0, 0, 0);
    AddSphere(pos, "sphere1");
    pos << kRadius * 2 - kPenetrationDepth, 0, 0;
    AddSphere(pos, "sphere2");

    tree_->compile();

    // Populate the plant.
    // Note: This is done here instead of the SetUp method because it appears
    //  the plant requires a *compiled* tree at constructor time.
    plant_ = make_unique<RigidBodyPlant<double>>(move(unique_tree));
    context_ = plant_->CreateDefaultContext();
    output_ = plant_->AllocateOutput(*context_);
    const int port_index = plant_->contact_results_output_port().get_index();

    // Set Sphere 2's velocity.
    auto velocities = context_->get_mutable_state()
                          ->get_mutable_continuous_state()
                          ->get_mutable_generalized_velocity();
    ASSERT_EQ(velocities->size(), 12);  // Two quaternion floating joints.
    Vector6d v_WS2 = get_v_WS2();
    VectorXd target_velocities;
    target_velocities.resize(12);
    target_velocities << 0, 0, 0,      // sphere 1 angular velocity
        0, 0, 0,                       // sphere 1 linear velocity
        v_WS2[0], v_WS2[1], v_WS2[2],  // sphere 2 angular velocity
        v_WS2[3], v_WS2[4], v_WS2[5];  // sphere 2 linear velocity
    velocities->SetFromVector(target_velocities);

    SetContactParameters();
    plant_->set_normal_contact_parameters(stiffness_, dissipation_);
    plant_->set_friction_contact_parameters(static_friction_, dynamic_friction_,
                                            v_stiction_tolerance_);

    plant_->CalcOutput(*context_.get(), output_.get());
    contacts_ =
        output_->get_data(port_index)->GetValue<ContactResults<double>>();
  }

  // Specifies the values to use as the rigid body plant's global contact
  // parameters.
  // TODO(SeanCurtis-TRI): Modify this as materials come into play.
  virtual void SetContactParameters() {
    stiffness_ = 10000;
    static_friction_ = 0.7;
    dynamic_friction_ = 0.5;
    v_stiction_tolerance_ = 0.01;
    dissipation_ = 0.5;
  }

  // Interprets the velocity of sphere 2 as the rate of change of penetration.
  double get_x_dot() {
    // The contact normal is parallel with the x-axis.  Because this is movement
    // of the right-hand sphere, positive values separate the spheres and
    // reflect a *decrease* in penetration.  Thus x_dot must be the negative of
    // this value.
    return -get_v_WS2()(3);
  }

  // Provides the linear velocity with which the second sphere is moving.
  // This allows for testing the damping and friction portions of the contact
  // force.  The velocity is v_WS2 (the velocity of sphere 2 w.r.t. the world
  // expressed in the world frame.)  Because sphere 2 is on the right, positive
  // x values *separate* the objects, negative values lead to deeper collision.
  virtual Vector6d get_v_WS2() {
    Vector6d vec;
    vec << 0., 0., 0., 0., 0., 0.;
    return vec;
  }

  // Add a sphere with default radius, placed at the given position.
  // Returns a raw pointer so that tests can use it for result validation.
  RigidBody<double>* AddSphere(const Vector3d& pos, const std::string& name) {
    RigidBody<double>* body;
    tree_->add_rigid_body(
        unique_ptr<RigidBody<double>>(body = new RigidBody<double>()));
    body->set_name(name);
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());
    Isometry3d pose = Isometry3d::Identity();
    pose.translate(pos);
    body->add_joint(&tree_->world(),
                    make_unique<QuaternionFloatingJoint>("base", pose));
    DrakeShapes::Sphere sphere(kRadius);
    DrakeCollision::Element collision_element(sphere);
    collision_element.set_body(body);
    tree_->addCollisionElement(collision_element, *body, "group1");
    return body;
  }

  // Instances owned by the class.
  unique_ptr<RigidBodyPlant<double>> plant_;
  unique_ptr<Context<double>> context_;
  unique_ptr<SystemOutput<double>> output_;
  ContactResults<double> contacts_;

  // Pointers used for poking into the structure to test values.
  RigidBodyTree<double>* tree_{};

  // Sphere configuration constants.
  const double kRadius = 1.0;
  const double kPenetrationDepth = 0.1;

  // Contact parameter constants.  These should get set by
  // SetContactParameters().
  double stiffness_ = 10000;
  double static_friction_ = 0.7;
  double dynamic_friction_ = 0.5;
  double v_stiction_tolerance_ = 0.01;
  double dissipation_ = 0.5;

  const double kTolerance = Eigen::NumTraits<double>::dummy_precision();
};

// Tests the case where the collision is a zero-velocity collision.  The force
// should be strictly dependent on the penetration depth.  This uses the default
// contact parameters.
TEST_F(ContactFormulaTest, ZeroVelocityCollision) {
  EXPECT_EQ(contacts_.get_num_contacts(), 1);
  // No relative velocity means that only the "spring" component affects the
  // force.
  double expected_force_magnitude = kPenetrationDepth * stiffness_;
  const auto info = contacts_.get_contact_info(0);
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // This force should be pushing sphere1 to the left.
  expected_spatial_force << 0, 0, 0, -expected_force_magnitude, 0, 0;
  ASSERT_TRUE(CompareMatrices(resultant.get_spatial_force(),
                              expected_spatial_force, kTolerance,
                              MatrixCompareType::absolute));
}

// This class provides a relative velocity in the normal direction which should
// engage the dissipation term. The velocity leads to *deeper* penetration.
class ConvergingContactFormulaTest : public ContactFormulaTest {
 protected:
  Vector6d get_v_WS2() override {
    Vector6d vec;
    vec << 0., 0., 0., kNormalVelocity, 0., 0.;
    return vec;
  }

  // The speed (m/s) at which they are *converging* (as indicated by negative
  // value.)
  const double kNormalVelocity = -1.f;
};

// Tests the case where the collision has a non-zero relative velocity in the
// contact normal direction.  The force will still only have a normal component,
// but adds the dissipating term.
TEST_F(ConvergingContactFormulaTest, ConvergingContactTest) {
  EXPECT_EQ(contacts_.get_num_contacts(), 1);
  // A non-zero relative velocity in the normal direction will change the force.
  // Still only a normal component, but we introduce the dissipation term.
  double expected_force_magnitude =
      kPenetrationDepth * stiffness_ * (1 + dissipation_ * get_x_dot());
  const auto info = contacts_.get_contact_info(0);
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // This force should be pushing sphere1 to the left.
  expected_spatial_force << 0, 0, 0, -expected_force_magnitude, 0, 0;
  ASSERT_TRUE(CompareMatrices(resultant.get_spatial_force(),
                              expected_spatial_force, kTolerance,
                              MatrixCompareType::absolute));
}

// This class provides a relative velocity in the normal direction which should
// engage the dissipation term. The velocity leads to *shallower* penetration,
// i.e., the spheres are diverging.  However, the diverging velocity is small
// enough that a repulsive force is still generated.
class DivergingContactFormulaTest : public ContactFormulaTest {
 protected:
  Vector6d get_v_WS2() override {
    Vector6d vec;
    vec << 0., 0., 0., kNormalVelocity, 0., 0.;
    return vec;
  }

  // The speed (m/s) at which they are *separating* (as indicated by positive
  // value.)  Its magnitude is less than 1/d so it will produce a repulsive
  // force.
  const double kNormalVelocity = 1 / dissipation_ / 2;
};

// Tests the case where the collision has a non-zero relative velocity in the
// direction opposite the contact normal. This reduces the normal force but
// it is still repulsive.
TEST_F(DivergingContactFormulaTest, DivergingContactTest) {
  EXPECT_EQ(contacts_.get_num_contacts(), 1);
  // A non-zero relative velocity in the normal direction will change the force.
  // Still only a normal component, but we introduce the dissipation term.
  double expected_force_magnitude =
      kPenetrationDepth * stiffness_ * (1 + dissipation_ * get_x_dot());
  const auto info = contacts_.get_contact_info(0);
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // This force should be pushing to the left.
  expected_spatial_force << 0, 0, 0, -expected_force_magnitude, 0, 0;
  ASSERT_TRUE(CompareMatrices(resultant.get_spatial_force(),
                              expected_spatial_force, kTolerance,
                              MatrixCompareType::absolute));
}

// This class sets up a scenario, where the instantaneous separating velocity is
// so great, that it would lead to a "suction" force, hindering separation.
// This test confirms that the force is clamped to zero, rather than become
// sucking.
class SuctionContactFormulaTest : public ContactFormulaTest {
 protected:
  Vector6d get_v_WS2() override {
    Vector6d vec;
    vec << 0., 0., 0., kNormalVelocity, 0., 0.;
    return vec;
  }

  // The relative velocity of the two contact points in the *normal* direction.
  // The normal force term is: kx(1 + dẋ), and k > 0 and d > 0.  In collision,
  // x > 0.  The force is repulsive for positive values. That requires,
  // 1 + dẋ > 0 --> ẋ > -1 / d.  We'll pick a separating velocity with a
  // magnitude greater than 1 / d.
  //
  // A positive value is a separating velocity. We pick one larger than the
  // threshold of 1 / d.
  const double kNormalVelocity = 1 / dissipation_ + kTolerance;
};

// This tests the case where there is a separating relative velocity beyond
// the threshold of the Hunt-Crossley model, producing no forces.
TEST_F(SuctionContactFormulaTest, SuctionContactTest) {
  EXPECT_EQ(contacts_.get_num_contacts(), 0);
}

// This class tests the stiction force -- the first interval of the Stribeck
// function (where the relative tangential velocity lies below the
// stiction speed tolerance.  It uses a relative *linear* velocity.
class StictionContactFormulaTest : public ContactFormulaTest {
 protected:
  Vector6d get_v_WS2() override {
    Vector6d vec;
    // Linear velocity in the +y direction.
    vec << 0., 0., 0., 0., kTangentSpeed, 0.;
    return vec;
  }

  // Tangent speed *is* the relative speed. By picking a speed that is half of
  // the stiction speed tolerance, the quintic interpolating function yields 0.5
  // (making for a simple evaluation in testing.)
  const double kTangentSpeed = v_stiction_tolerance_ / 2;
};

// Confirms that a slight tangential relative velocity produces the correct
// tangential force.  The relative velocity is due to linear motion and the
// magnitude falls below the stiction speed tolerance.
TEST_F(StictionContactFormulaTest, StictionContactTest) {
  EXPECT_EQ(contacts_.get_num_contacts(), 1);

  // The normal component is simply the undamped case (zero relative velocity
  // in the normal direction).
  double expected_normal_magnitude = kPenetrationDepth * stiffness_;
  // In the first interval (half the transition velocity) the coefficient of
  // friction is half the static coefficient.
  double expected_mu = static_friction_ * 0.5;
  double expected_tangent_magnitude = expected_mu * expected_normal_magnitude;
  const auto info = contacts_.get_contact_info(0);
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // The normal component should be pointing to the left (-x) and the tangential
  // component should be pointing up (+y).
  expected_spatial_force << 0, 0, 0, -expected_normal_magnitude,
      expected_tangent_magnitude, 0;
  ASSERT_TRUE(CompareMatrices(resultant.get_spatial_force(),
                              expected_spatial_force, kTolerance,
                              MatrixCompareType::absolute));
}

// This class tests the transition from stiction to sliding friction. The
// relative speed is two times the stiction speed threshold, putting it in the
// middle of the second interval of the Stribeck function.
class TransitionContactFormulaTest : public ContactFormulaTest {
 protected:
  Vector6d get_v_WS2() override {
    Vector6d vec;
    vec << 0., 0., 0., 0., kTangentSpeed, 0.;
    return vec;
  }

  // Tangent speed *is* the relative speed. Two times the stiction speed
  // tolerance will produce a coefficient of friction that is the average of the
  // dynamic and static coefficients of friction.
  const double kTangentSpeed = v_stiction_tolerance_ * 2;
};

// Confirms that a slight tangential force (between 1 and 3X the transition
// speed) produces the correct tangential force.
TEST_F(TransitionContactFormulaTest, TransitionContactTest) {
  EXPECT_EQ(contacts_.get_num_contacts(), 1);

  // The normal component is simply the undamped case (zero relative velocity
  // in the normal direction).
  double expected_normal_magnitude = kPenetrationDepth * stiffness_;
  // In the second interval (relative speed = 2 * transition speed) the
  // coefficient of friction is the average of dynamic and static coefficients.
  double expected_mu = (dynamic_friction_ + static_friction_) * 0.5;
  double expected_tangent_magnitude = expected_mu * expected_normal_magnitude;
  const auto info = contacts_.get_contact_info(0);
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // The normal component should be pointing to the left (-x) and the tangential
  // component should be pointing up (+y).
  expected_spatial_force << 0, 0, 0, -expected_normal_magnitude,
      expected_tangent_magnitude, 0;
  ASSERT_TRUE(CompareMatrices(resultant.get_spatial_force(),
                              expected_spatial_force, kTolerance,
                              MatrixCompareType::absolute));
}

// This class tests the friction force for when the relative velocity is clearly
// in the sliding regime (i.e., relative speed > 3 * stiction speed tolerance.
class SlidingContactFormulaTest : public ContactFormulaTest {
 protected:
  Vector6d get_v_WS2() override {
    Vector6d vec;
    vec << 0., 0., 0., 0., kTangentSpeed, 0.;
    return vec;
  }

  // Tangent speed *is* the relative speed. In the sliding regime, the
  // coefficient of friction is just the dynamic coefficient of friction.
  const double kTangentSpeed = v_stiction_tolerance_ * 5;
};

// Confirms that a slight tangential velocity (between 1X and 3X the stiction
// speed threshold) produces the correct tangential force.
TEST_F(SlidingContactFormulaTest, SlidingContactTest) {
  EXPECT_EQ(contacts_.get_num_contacts(), 1);

  // The normal component is simply the undamped case (zero relative velocity
  // in the normal direction).
  double expected_normal_magnitude = kPenetrationDepth * stiffness_;
  // In the final interval (relative speed > 3 * transition speed) the
  // coefficient of friction is the dynamic coefficient.
  double expected_mu = dynamic_friction_;
  double expected_tangent_magnitude = expected_mu * expected_normal_magnitude;
  const auto info = contacts_.get_contact_info(0);
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // The normal component should be pointing to the left (-x) and the tangential
  // component should be pointing up (+y).
  expected_spatial_force << 0, 0, 0, -expected_normal_magnitude,
      expected_tangent_magnitude, 0;
  ASSERT_TRUE(CompareMatrices(resultant.get_spatial_force(),
                              expected_spatial_force, kTolerance,
                              MatrixCompareType::absolute));
}

// This class confirms that relative velocity due to angular velocity produces
// the correct frictional force.
class SlidingSpinContactFormulaTest : public ContactFormulaTest {
 protected:
  Vector6d get_v_WS2() override {
    Vector6d vec;
    vec << 0., 0., kZAngularSpeed, 0., 0., 0.;
    return vec;
  }

  // The contact is at a distance of radius - 1/2 penetration depth to the
  // sphere center. I want that point to have a *linear* velocity of 5 *
  // stiction speed tolerance (in the +y direction). This is the angular
  // speed that provides it.
  const double kZAngularSpeed =
      -0.5 * v_stiction_tolerance_ / (kRadius - kPenetrationDepth * 0.5);
};

// Confirms that the correct frictional force is produced in the case where
// the relative velocity at the contact point is due to angular velocity.
TEST_F(SlidingSpinContactFormulaTest, SlidingSpinContactTest) {
  EXPECT_EQ(contacts_.get_num_contacts(), 1);

  // The normal component is simply the undamped case (zero relative velocity
  // in the normal direction).
  double expected_normal_magnitude = kPenetrationDepth * stiffness_;
  // In the final interval (slip speed > 3 * transition speed) the coefficient
  // of friction is the dynamic coefficients.
  double expected_mu = 0.5 * static_friction_;
  double expected_tangent_magnitude = expected_mu * expected_normal_magnitude;
  const auto info = contacts_.get_contact_info(0);
  const auto& resultant = info.get_resultant_force();
  SpatialForce<double> expected_spatial_force;
  // The normal component should be pointing to the left (-x) and the tangential
  // component should be pointing up (+y).
  expected_spatial_force << 0, 0, 0, -expected_normal_magnitude,
      expected_tangent_magnitude, 0;
  ASSERT_TRUE(CompareMatrices(resultant.get_spatial_force(),
                              expected_spatial_force, kTolerance,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace systems
}  // namespace drake
