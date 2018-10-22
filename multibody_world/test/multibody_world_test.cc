#include "drake/multibody_world/multibody_world.h"

#include <utility>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "drake/common/find_resource.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

using drake::multibody::parsing::AddModelFromSdfFile;

namespace drake {
namespace geometry {
namespace internal {

class MultibodyWorldClassTest : public ::testing::Test {
 protected:
  void SetUp() {}
  void AddCartPole() {
    auto& cart_pole = mbp_sg_.mutable_multibody_plant();

    // Make and add the cart_pole model.
    const std::string full_name = FindResourceOrThrow(
        "drake/examples/multibody/cart_pole/cart_pole.sdf");
    AddModelFromSdfFile(
        full_name, &cart_pole, &mbp_sg_.mutable_scene_graph());

    // Add gravity to the model.
    cart_pole.AddForceElement<multibody::UniformGravityFieldElement>(
        -9.81 * Vector3<double>::UnitZ());
  }

  MultibodyWorld<double> mbp_sg_;
};

// Verifies that attempting to use the system (by accessing a port) before it is
// finalized is not allowed.
TEST_F(MultibodyWorldClassTest, NoFinalizeYieldsFailure) {
  ASSERT_FALSE(mbp_sg_.is_finalized());

  // We check only the pose bundle output; others are dependent on *how* the
  // plant has been set up.
  EXPECT_THROW(mbp_sg_.get_pose_bundle_output_port(), std::logic_error);
}

// Verifies that attempting to finalize the system without registering
// geometries results in an error.
TEST_F(MultibodyWorldClassTest, FinalizeWithoutRegisteringGeomsFails) {
  EXPECT_THROW(mbp_sg_.Finalize(), std::logic_error);
}

// Tests that finalizing allows the system to be used.
// finalized is not allowed.
TEST_F(MultibodyWorldClassTest, FinalizeEqualsSuccess) {
  AddCartPole();

  // Now the model is complete.
  mbp_sg_.Finalize();

  ASSERT_TRUE(mbp_sg_.is_finalized());

  // We check only the pose bundle output; others are dependent on *how* the
  // plant has been set up.
  EXPECT_NO_THROW(mbp_sg_.get_pose_bundle_output_port());
}

// Tests that getting various ports works. 
TEST_F(MultibodyWorldClassTest, PortAccess) {
  AddCartPole();
  mbp_sg_.Finalize();
  ASSERT_TRUE(mbp_sg_.is_finalized());

  // TODO: Finish me.
}

// Tests that getting the MultibodyPlant context works.
TEST_F(MultibodyWorldClassTest, MultibodyPlantContext) {
  AddCartPole();
  mbp_sg_.Finalize();
  ASSERT_TRUE(mbp_sg_.is_finalized());
  auto& cart_pole = mbp_sg_.mutable_multibody_plant();

  // Create the Diagram.
  auto diagram = builder.Build();

  // Create a context for this system:
  auto diagram_context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& cart_pole_context = mbp_sg.
      GetMutableMultibodyPlantContext(diagram.get(), diagram_context.get());

  // Try using the cart pole context.
  cart_pole_context.FixInputPort(
      mbp_sg.get_actuation_input_port().get_index(), Vector1d(0));

  // Get joints so that we can set initial conditions.
  const PrismaticJoint<double>& cart_slider =
      cart_pole.GetJointByName<PrismaticJoint>("CartSlider");
  const RevoluteJoint<double>& pole_pin =
      cart_pole.GetJointByName<RevoluteJoint>("PolePin");

  // Set initial state.
  cart_slider.set_translation(&cart_pole_context, 1.0);
  pole_pin.set_angle(&cart_pole_context, 2.0);

  // Verify that the state was set properly.
  EXPECT_EQ(cart_slider.get_translation(cart_pole_context), 1.0);
  EXPECT_EQ(pole_pin.get_angle(cart_pole_context), 2.0);
}

// Tests that AutoDiff construction is possible.
GTEST_TEST(MultibodyWorldTest, AutoDiff) {
  EXPECT_NO_THROW(MultibodyWorld<AutoDiffXd>());
}


}  // namespace internal
}  // namespace geometry
}  // namespace drake
