#include "drake/multibody/cenic/continuous_icf_force_manager.h"

#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

// A double pendulum with no world contact. The two links are on adjacent bodies
// (and link1 is adjacent to the world), so default collision filtering removes
// all contact -- hence the ICF model has zero contact constraints.
constexpr char kDoublePendulumMjcf[] = R"""(
<?xml version="1.0"?>
<mujoco model="double_pendulum">
  <worldbody>
    <body name="link1">
      <joint type="hinge" name="joint1" axis="0 1 0" pos="0 0 0.1"/>
      <geom type="capsule" size="0.01 0.1"/>
      <body name="link2">
        <joint type="hinge" name="joint2" axis="0 1 0" pos="0 0 -0.1"/>
        <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
</mujoco>
)""";

// A ball on a vertical slider resting on a floor: exercises the contact path.
constexpr char kBallOnFloorMjcf[] = R"""(
<?xml version="1.0"?>
<mujoco model="ball_on_floor">
  <worldbody>
    <geom name="floor" type="box" pos="0 0 -0.1" size="50 50 0.1"/>
    <body name="ball" pos="0 0 1.0">
      <joint name="slider" type="slide" axis="0 0 1"/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)""";

// Builds a continuous plant + scene graph from `mjcf`. If `add_reporter`, wires
// the ICF continuous force reporter. Returns the diagram and a pointer to the
// plant within it.
std::pair<std::unique_ptr<Diagram<double>>, MultibodyPlant<double>*>
BuildDiagram(const char* mjcf, bool add_reporter) {
  DiagramBuilder<double> builder;
  auto items = AddMultibodyPlantSceneGraph(&builder, /* time_step = */ 0.0);
  MultibodyPlant<double>& plant = items.plant;
  Parser(&plant).AddModelsFromString(mjcf, "xml");
  plant.Finalize();
  if (add_reporter) {
    AddIcfContinuousForceReporting(&plant);
  }
  return {builder.Build(), &plant};
}

const std::vector<SpatialForce<double>>& EvalReactions(
    const MultibodyPlant<double>& plant, const Context<double>& plant_context) {
  return plant.get_reaction_forces_output_port()
      .template Eval<std::vector<SpatialForce<double>>>(plant_context);
}

// With no contact (and no other ICF constraints), the ICF reporter's applied
// forces and resulting accelerations equal the plant's compliant continuous
// path, so the reaction_forces output port must be unchanged by adding the
// reporter -- even at a dynamic (nonzero vdot) state.
GTEST_TEST(ContinuousIcfForceManagerTest, NoContactMatchesCompliantModel) {
  auto [diagram_ref, plant_ref] =
      BuildDiagram(kDoublePendulumMjcf, /* add_reporter = */ false);
  auto [diagram_icf, plant_icf] =
      BuildDiagram(kDoublePendulumMjcf, /* add_reporter = */ true);

  // A dynamic state: bent configuration with nonzero velocity.
  const Eigen::Vector2d q(0.3, -0.5);
  const Eigen::Vector2d v(0.7, 1.1);

  auto root_ref = diagram_ref->CreateDefaultContext();
  auto root_icf = diagram_icf->CreateDefaultContext();
  Context<double>& pc_ref = plant_ref->GetMyMutableContextFromRoot(&*root_ref);
  Context<double>& pc_icf = plant_icf->GetMyMutableContextFromRoot(&*root_icf);
  plant_ref->SetPositions(&pc_ref, q);
  plant_ref->SetVelocities(&pc_ref, v);
  plant_icf->SetPositions(&pc_icf, q);
  plant_icf->SetVelocities(&pc_icf, v);

  const std::vector<SpatialForce<double>>& reactions_ref =
      EvalReactions(*plant_ref, pc_ref);
  const std::vector<SpatialForce<double>>& reactions_icf =
      EvalReactions(*plant_icf, pc_icf);

  ASSERT_EQ(reactions_ref.size(), reactions_icf.size());
  for (int i = 0; i < static_cast<int>(reactions_ref.size()); ++i) {
    EXPECT_TRUE(CompareMatrices(reactions_ref[i].get_coeffs(),
                                reactions_icf[i].get_coeffs(), 1e-9))
        << "Mismatch at joint " << i;
  }
}

// A ball penetrating the floor produces an ICF contact patch. Evaluating the
// reaction_forces port through the reporter exercises the full contact-force
// extraction path (geometry cache, ICF model build, per-body spatial force
// assembly) and must yield finite values.
GTEST_TEST(ContinuousIcfForceManagerTest, ContactPathProducesFiniteReactions) {
  auto [diagram, plant] =
      BuildDiagram(kBallOnFloorMjcf, /* add_reporter = */ true);

  auto root = diagram->CreateDefaultContext();
  Context<double>& pc = plant->GetMyMutableContextFromRoot(&*root);
  // Slider displacement so the ball center sits at z = 0.05, penetrating the
  // floor (top at z = 0) by 0.05 m. Rest configuration is z = 1.0.
  const Vector1<double> q(0.05 - 1.0);
  plant->SetPositions(&pc, q);
  plant->SetVelocities(&pc, Vector1<double>::Zero());

  const std::vector<SpatialForce<double>>& reactions = EvalReactions(*plant, pc);
  ASSERT_EQ(reactions.size(), plant->num_joints());
  for (const SpatialForce<double>& F : reactions) {
    EXPECT_TRUE(F.get_coeffs().allFinite());
  }
}

GTEST_TEST(ContinuousIcfForceManagerTest, RejectsDiscretePlant) {
  MultibodyPlant<double> plant(/* time_step = */ 0.01);
  plant.Finalize();
  EXPECT_THROW(AddIcfContinuousForceReporting(&plant), std::exception);
}

GTEST_TEST(ContinuousIcfForceManagerTest, RejectsUnfinalizedPlant) {
  MultibodyPlant<double> plant(/* time_step = */ 0.0);
  EXPECT_THROW(AddIcfContinuousForceReporting(&plant), std::exception);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
