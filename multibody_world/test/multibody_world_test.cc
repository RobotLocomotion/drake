#include "drake/multibody_world/multibody_world.h"

#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "drake/common/find_resource.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/diagram_builder.h"

using ::testing::HasSubstr;
using drake::multibody::parsing::AddModelFromSdfFile;

namespace drake {
namespace geometry {
namespace internal {

class MultibodyWorldClassTest : public ::testing::Test {
 protected:
  void SetUp() {}
  void AddCartPole(MultibodyWorld<double>& mbworld) const {
    auto& cart_pole = mbworld.mutable_multibody_plant();

    // Make and add the cart_pole model.
    const std::string full_name = FindResourceOrThrow(
        "drake/examples/multibody/cart_pole/cart_pole.sdf");
    AddModelFromSdfFile(
        full_name, &cart_pole, &mbworld.mutable_scene_graph());

    // Add gravity to the model.
    cart_pole.AddForceElement<multibody::UniformGravityFieldElement>(
        -9.81 * Vector3<double>::UnitZ());
  }

  MultibodyWorld<double> mbworld_;
};

// Verifies that attempting to use the system (by accessing a port) before it is
// finalized is not allowed.
TEST_F(MultibodyWorldClassTest, NoFinalizeYieldsFailure) {
  ASSERT_FALSE(mbworld_.is_finalized());

  // We check only the pose bundle output; others are dependent on *how* the
  // plant has been set up.
  EXPECT_THROW(mbworld_.get_pose_bundle_output_port(), std::logic_error);
}

// Verifies that attempting to finalize the system without registering
// geometries results in an error.
TEST_F(MultibodyWorldClassTest, FinalizeWithoutRegisteringGeomsFails) {
  EXPECT_THROW(mbworld_.Finalize(), std::logic_error);
}

// Tests that finalizing allows the system to be used.
// finalized is not allowed.
TEST_F(MultibodyWorldClassTest, FinalizeEqualsSuccess) {
  AddCartPole(mbworld_);

  // Now the model is complete.
  mbworld_.Finalize();

  ASSERT_TRUE(mbworld_.is_finalized());

  // We check only the pose bundle output; others are dependent on *how* the
  // plant has been set up.
  EXPECT_NO_THROW(mbworld_.get_pose_bundle_output_port());
}

// Tests that getting various ports works. 
TEST_F(MultibodyWorldClassTest, PortAccess) {
  AddCartPole(mbworld_);
  mbworld_.Finalize();
  ASSERT_TRUE(mbworld_.is_finalized());

  // Verify that we can get the pose bundle port.
  ASSERT_NO_THROW(mbworld_.get_pose_bundle_output_port());
  auto& pose_bundle_output = mbworld_.get_pose_bundle_output_port();
  EXPECT_EQ(pose_bundle_output.get_name(), "scene_graph_lcm_visualization");

  // Get the actuation input port for the cart-pole.
//  const auto& cart_pole = mbworld_.multibody_plant();

  // Get the actuation input port (actual string is long and nasty so we look
  // for a substring).
  ASSERT_NO_THROW(mbworld_.get_actuation_input_port());
  auto& actuation_input = mbworld_.get_actuation_input_port();
  EXPECT_THAT(actuation_input.get_name(), HasSubstr("actuation"));
//  EXPECT_EQ(actuation_input.get_name(), "scene_graph_lcm_visualization");

}

// Tests that getting the MultibodyPlant context works.
TEST_F(MultibodyWorldClassTest, MultibodyPlantContext) {
  // Have the DiagramBuilder create the MultibodyWorld system.
  systems::DiagramBuilder<double> builder;
  const double step_size = 0.0;
  auto& mbworld = *builder.AddSystem<MultibodyWorld<double>>(step_size);

  AddCartPole(mbworld);
  mbworld.Finalize();
  ASSERT_TRUE(mbworld.is_finalized());
  auto& cart_pole = mbworld.mutable_multibody_plant();

  // Instantiate the diagram.
  auto diagram = builder.Build();

  // Create a context for this system:
  auto diagram_context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& cart_pole_context = mbworld.
      GetMutableMultibodyPlantContext(diagram.get(), diagram_context.get());

  // Try using the cart pole context.
  cart_pole_context.FixInputPort(
      mbworld.get_actuation_input_port().get_index(), Vector1d(0));

  // Get joints so that we can set initial conditions.
  const multibody::PrismaticJoint<double>& cart_slider =
      cart_pole.GetJointByName<multibody::PrismaticJoint>("CartSlider");
  const multibody::RevoluteJoint<double>& pole_pin =
      cart_pole.GetJointByName<multibody::RevoluteJoint>("PolePin");

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
