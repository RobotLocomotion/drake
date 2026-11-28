#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/examples/multibody/strandbeest_unassembled/assemble_nearest_configuration.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace strandbeest_unassembled {
namespace {

using drake::multibody::BodyIndex;
using drake::multibody::Frame;
using drake::multibody::Link;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector2d;
using math::RigidTransformd;
using systems::Context;

/* Returns the cross product of Q - P and R - P, for points in the plane of a
leg, given by their x and z coordinates. It is positive when R is
counterclockwise of the line from P through Q, viewed with x to the right and z
up, and negative when R is clockwise of it. */
double Cross(const Vector2d& P, const Vector2d& Q, const Vector2d& R) {
  const Vector2d u = Q - P;
  const Vector2d v = R - P;
  return u.x() * v.y() - u.y() * v.x();
}

/* Returns true if the quadrilateral ABCD is convex: as it goes around, it
turns the same way at every corner. A quadrilateral folded over on itself turns
different ways at different corners. */
bool IsConvex(const Vector2d& A, const Vector2d& B, const Vector2d& C,
              const Vector2d& D) {
  const double turns[] = {Cross(A, B, C), Cross(B, C, D), Cross(C, D, A),
                          Cross(D, A, B)};
  bool all_left = true;
  bool all_right = true;
  for (double turn : turns) {
    all_left = all_left && turn > 0;
    all_right = all_right && turn < 0;
  }
  return all_left || all_right;
}

/* Returns true if P and Q are on opposite sides of the line through A and B. */
bool AreOnOppositeSides(const Vector2d& A, const Vector2d& B, const Vector2d& P,
                        const Vector2d& Q) {
  return Cross(A, B, P) * Cross(A, B, Q) < 0;
}

/* The program assembles every leg as the model file draws it, with its crank
pointing straight away from it, before turning any crank to its phase. Every
leg is drawn the same way (some mirrored), so assembling the one-leg model the
same way checks them all. A linkage with loops can be assembled in more than
one way; this checks that the leg comes out the way Jansen designed it. */
GTEST_TEST(JansenLegAssemblyTest, AssemblesOnTheIntendedBranch) {
  // A discrete plant, as the program uses by default. The time step plays no
  // part in assembly.
  MultibodyPlant<double> plant(0.05);
  Parser(&plant).AddModelsFromUrl(
      "package://drake/examples/multibody/strandbeest_unassembled/model/"
      "JansenLeg.sdf");
  plant.Finalize();
  // Each of the leg's three kinematic loops costs a shadow link and a weld.
  ASSERT_EQ(plant.num_bodies() - plant.num_user_bodies(), 3);
  ASSERT_EQ(plant.num_weld_constraints(), 3);

  // Hold what the program holds while it assembles the legs: the floating
  // chassis, the crankshaft, and the crank, all where the model file puts
  // them.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  for (BodyIndex index : plant.GetFloatingBaseBodies()) {
    plant.get_body(index).Lock(context.get());
  }
  plant.GetJointByName("chassis_crankshaft").Lock(context.get());
  plant.GetJointByName("leg1_crankshaft_crank").Lock(context.get());

  const solvers::MathematicalProgramResult result =
      AssembleNearestConfiguration(plant, context.get());
  ASSERT_TRUE(result.is_success());

  // Every shadow link now coincides with the link it was split from.
  for (BodyIndex index(plant.num_user_bodies()); index < plant.num_bodies();
       ++index) {
    const Link<double>& shadow = plant.get_body(index);
    const RigidTransformd X_PS = plant.CalcRelativeTransform(
        *context, shadow.primary_link().body_frame(), shadow.body_frame());
    EXPECT_TRUE(X_PS.IsNearlyEqualTo(RigidTransformd::Identity(), 1e-5))
        << shadow.name();
  }

  // Where the leg's pins (and its foot) are in the plane of the leg, as (x, z)
  // in the chassis frame, whose origin is on the crank axle. The leg reaches
  // out in -x from the axle, and its crank points in +x.
  const Frame<double>& chassis = plant.GetFrameByName("chassis_O");
  auto pin = [&](const std::string& hole) {
    const Eigen::Vector3d p_CH = plant.GetFrameByName("leg1_" + hole)
                                     .CalcPose(*context, chassis)
                                     .translation();
    return Vector2d(p_CH.x(), p_CH.z());
  };
  const Vector2d X = pin("crank_X");    // The crank pin.
  const Vector2d Y = pin("bracket_Y");  // The fixed pivot.
  const Vector2d U1 = pin("bde_U1");
  const Vector2d U2 = pin("bde_U2");
  const Vector2d L1 = pin("ghi_L1");
  const Vector2d L2 = pin("ghi_L2");
  const Vector2d F = pin("ghi_F");  // The foot.

  // Each of these chooses between the two ways that a pair of parts can meet
  // at a pin, and together they leave only Jansen's design. Of the other 31
  // ways to assemble the leg, 16 can't turn the crank all the way around, and
  // the other 15 lose the long, flat bottom of the foot's path that makes the
  // machine walk smoothly: Jansen's foot stays within 1 cm of its lowest point
  // for 155 degrees of each crank turn, the others' for 60 degrees at most.

  // The crank's four-bar loop X-U1-Y-L1 (bars j, b, c, and k) is convex,
  // with U1 above the line from the fixed pivot Y to the crank pin X, and so
  // L1 below it.
  EXPECT_TRUE(IsConvex(X, U1, Y, L1));
  EXPECT_GT(Cross(Y, X, U1), 0);
  // The upper triangle bde is on the far side of its bar b (Y-U1) from the
  // crank pin.
  EXPECT_TRUE(AreOnOppositeSides(Y, U1, U2, X));
  // The four-bar loop Y-U2-L2-L1 (bars d, f, g, and c), which Jansen made
  // nearly a parallelogram, is convex.
  EXPECT_TRUE(IsConvex(Y, U2, L2, L1));
  // The foot hangs below the lower triangle's side g (L1-L2), on the far side
  // of it from the upper triangle.
  EXPECT_TRUE(AreOnOppositeSides(L1, L2, F, U2));
}

}  // namespace
}  // namespace strandbeest_unassembled
}  // namespace examples
}  // namespace drake
