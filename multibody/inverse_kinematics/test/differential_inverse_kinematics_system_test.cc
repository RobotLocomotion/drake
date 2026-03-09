#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h"

#include <future>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/planning/dof_mask.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/systems/framework/bus_value.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::Sphere;
using math::RigidTransformd;
using planning::CollisionChecker;
using planning::CollisionCheckerParams;
using planning::DofMask;
using planning::JointLimits;
using planning::RobotClearance;
using planning::RobotDiagram;
using planning::RobotDiagramBuilder;
using planning::SceneGraphCollisionChecker;
using solvers::Binding;
using solvers::EvaluatorBase;
using systems::BusValue;
using yaml::SaveYamlString;

using DiffIk = DifferentialInverseKinematicsSystem;
using CallbackDetails = DiffIk::CallbackDetails;
using Ingredient = DiffIk::Ingredient;
using Recipe = DiffIk::Recipe;

/* Assorted construction parameters. */
static constexpr std::string_view kRobotName{"robot"};
static constexpr double kTimeStep{0.25};
static constexpr double kK_VX{2};
static constexpr double kQpWeight{2.5};
static constexpr std::string_view kTaskFrame("task_frame");

/* Parameters for a single test evaluation of Diff Ik. */
struct Sample {
  /* Description to use for feedback; this will be printed upon failure. */
  std::string description;
  /* The desired translational velocity of the ball. */
  Vector3d vd_WB;
  /* The expected translational velocity of the ball. */
  Vector3d v_WB_expected;
  /* Axis masking on cartesian velocities means that Diff IK may produce
  unexpected velocities for the masked elements. This mask communicates which
  translational velocity elements matter to the test (1) or should be ignored
  (0). */
  Vector3d v_WB_mask = Vector3d(1, 1, 1);
  /* The values of q₀ and q₁ for the query. By default, it has initial values
  for the non-redundant robot. */
  VectorXd q_active = (VectorXd(2) << 0, 0).finished();
  /* The nominal pose of the active dofs. If not provided, it is the zero
  vector. This is only used for the joint centering cost. */
  std::optional<VectorXd> q_active_nominal = std::nullopt;
};

/* Testing strategy.

We're _not_ explicitly testing:

  - DiffIk::Ingredient
    - It is an abstract interface and has nothing to test.
  - DiffIk::Recipe
    - The only public method (AddToProgram()) is implicitly tested over and
      over in each invocation of the constructor.

The system has two possible inputs which control the commanded velocity: desired
position and velocity. We'll have a single test vis a vis the position input --
confirming how it infers a desired velocity -- but mostly test against the
velocity input.

The reason for this is that the position input is merely used to construct a
desired velocity. In the remaining tests, by explicitly specifying the desired
velocity, we can limit our reasoning about velocity space and not worry about
the mechanism for inferring velocity from position. */
class DifferentialInverseKinematicsTest : public ::testing::Test {
 protected:
  /* Constructs the differential inverse kinematic system with the given recipe.

  @param recipe          The optimization recipe.
  @param has_redundancy  If `true`, the robot will have 3-dofs (with one
                         redundant -- see MakeRobotDiagram()). */
  static DiffIk MakeDiffIk(std::shared_ptr<Recipe> recipe,
                           bool has_redundancy = false) {
    std::shared_ptr<CollisionChecker> collision_checker =
        MakeChecker(has_redundancy);
    const DofMask robot_dofs = DofMask::MakeFromModel(
        collision_checker->plant(), std::string(kRobotName));
    return DiffIk(std::move(recipe), kTaskFrame, collision_checker, robot_dofs,
                  kTimeStep, kK_VX, MakeSpatialLimits());
  }

  /* Convenience function for adding the least-squares cost. */
  static void AddLeastSquaresCost(
      Recipe* recipe, const string_unordered_map<Vector6d>& masks = {}) {
    auto ingredient = std::make_shared<DiffIk::LeastSquaresCost>(
        DiffIk::LeastSquaresCost::Config{.cartesian_axis_masks = masks});
    recipe->AddIngredient(std::move(ingredient));
  }

  /* Create the collision checker for the DiffIK dut. */
  static std::shared_ptr<CollisionChecker> MakeChecker(bool has_redundancy) {
    const int dof_count = has_redundancy ? 3 : 2;
    std::shared_ptr<RobotDiagram<double>> robot = MakeRobotDiagram(dof_count);
    const ModelInstanceIndex robot_index =
        robot->plant().GetModelInstanceByName(kRobotName);
    CollisionCheckerParams params{
        .model = std::move(robot),
        .robot_model_instances = {robot_index},
        .edge_step_size = 0.1,  // Arbitrary irrelevant value.
        .env_collision_padding = 0,
    };
    return std::make_shared<SceneGraphCollisionChecker>(std::move(params));
  }

  /* Build a model with a *very* simple robot and an obstacle.

    - The robot is simply a zero-radius sphere (a point) whose origin moves on
      the Wz = 0 plane.
    - The robot is joined to the task frame, which is a stationary frame
      aligned with, but offset from the world frame (giving us the ability to
      discern the distinction between a named task frame and world).
      - The task frame's only relevance is in evaluating
        CartesianPositionLimitConstraint.
    - The obstacle is simply a unit sphere at the task frame's origin. It _does_
      have a degree of freedom so that we can distinguish between the robot qs
      and environment qs, but we will keep the obstacle located at the origin.

  The robot can be configured to have two or three dofs.
    - Two dofs: q0 controls movement on the Wx axis and q1 the Wy axis.
    - Three dofs: we add a third _redundant_ dof: q2 is a linear combination
      (A = (Wx + Wy)/√2).

  Some notes on the intention behind this design:

    1. The mapping from generalized position and velocity to cartesian position
       and velocity is simple.
       - For the 2-dof robot: the position of the sphere p_WB is simply
         (q0, q1, 0) and its velocity v_WB is simply (q̇₀, q̇₁, 0).
       - The 3-dof version adds a redundant term: p_WB = q₀·W_x + q₁·W_y + q₂·A.
    2. The Diff IK system does not make any special allowances for the
       interpretation of the robot dofs. So, we can use this simple mapping
       between generalized state and cartesian kinematics without fear of
       under testing the system.
    3. The 3-dof robot, with its built in redundancy, serves as the basis for
       evaluating the "joint centering" constraint. It will only be introduced
       for *that* test -- as it is overly cumbersome in all other tests. */
  static std::shared_ptr<RobotDiagram<double>> MakeRobotDiagram(int dof_count) {
    DRAKE_DEMAND(dof_count == 2 || dof_count == 3);

    RobotDiagramBuilder<double> builder;
    MultibodyPlant<double>& plant = builder.plant();

    // Add the task frame.
    const auto& task_frame =
        plant.AddFrame(std::make_unique<FixedOffsetFrame<double>>(
            std::string(kTaskFrame), plant.world_frame(),
            RigidTransformd(GetTaskOrigin())));

    // Add the obstacle.
    const RigidBody<double>& obstacle =
        plant.AddRigidBody("obstacle", SpatialInertia<double>::MakeUnitary());
    plant.RegisterCollisionGeometry(obstacle, {}, Sphere(1.0), "obstacle",
                                    CoulombFriction<double>());
    plant.AddJoint<PrismaticJoint>("Wz", plant.world_body(), std::nullopt,
                                   obstacle, std::nullopt, Vector3d::UnitZ());

    // Add the articulated robot.
    const ModelInstanceIndex robot_index =
        plant.AddModelInstance(std::string(kRobotName));
    const RigidBody<double>& body0 = plant.AddRigidBody(
        "body0", robot_index, SpatialInertia<double>::Zero());
    const RigidBody<double>& ball = plant.AddRigidBody(
        "ball", robot_index, SpatialInertia<double>::MakeUnitary());
    plant.RegisterCollisionGeometry(ball, {}, Sphere(0.0), "ball",
                                    CoulombFriction<double>());
    plant.AddJoint(std::make_unique<PrismaticJoint<double>>(
        "Wx", task_frame, body0.body_frame(), Vector3d::UnitX()));
    if (dof_count == 3) {
      const RigidBody<double>& body1 = plant.AddRigidBody(
          "body1", robot_index, SpatialInertia<double>::Zero());
      plant.AddJoint<PrismaticJoint>("Wy", body0, std::nullopt, body1,
                                     std::nullopt, Vector3d::UnitY());
      plant.AddJoint<PrismaticJoint>(
          "A", body1, std::nullopt, ball, std::nullopt,
          (Vector3d::UnitX() + Vector3d::UnitY()).normalized());
    } else {
      plant.AddJoint<PrismaticJoint>("Wy", body0, std::nullopt, ball,
                                     std::nullopt, Vector3d::UnitY());
    }

    plant.Finalize();

    return builder.Build();
  }

  /* Given a plant containing the "ball" rigid body and commanded velocities for
  the active dofs, computes the spatial velocity of "ball": V_WB. */
  static SpatialVelocity<double> GetBallSpatialVelocity(
      const MultibodyPlant<double>& plant, const VectorXd& v_active) {
    auto context = plant.CreateDefaultContext();
    Eigen::VectorXd v_full(plant.num_velocities());
    v_full.setZero();
    auto active_dof = DofMask::MakeFromModel(plant, std::string(kRobotName));
    active_dof.SetInArray(v_active, &v_full);
    plant.SetVelocities(context.get(), v_full);
    const RigidBody<double>& body = plant.GetRigidBodyByName("ball");
    return plant.EvalBodySpatialVelocityInWorld(*context, body);
  }

  /* Reports the pose of the task frame in the world frame: p_WT. */
  static Vector3d GetTaskOrigin() { return Vector3d(-1, -2, 0); }

  /* An arbitrary value to use on the dut's constructor as the Vd_TG_limit
  value. */
  static SpatialVelocity<double> MakeSpatialLimits() {
    return SpatialVelocity<double>(Vector3d(2, 3, 4), Vector3d(5, 6, 7));
  }

  /* Evaluates the given `dut` against the set of samples; returns the commanded
  velocities. */
  static std::vector<VectorXd> EvaluateSamples(
      const DiffIk& dut, const std::vector<Sample>& samples) {
    std::vector<VectorXd> results;
    auto context = dut.CreateDefaultContext();

    VectorXd q(dut.plant().num_positions());

    BusValue desired_velocities;
    for (const auto& sample : samples) {
      SCOPED_TRACE(sample.description);
      // The reference pose will *always* be the zero pose. Only relevant for
      // joint centering.
      q.setZero();
      if (sample.q_active_nominal.has_value()) {
        dut.active_dof().SetInArray(*sample.q_active_nominal, &q);
      }
      dut.get_input_port_nominal_posture().FixValue(context.get(), q);
      // The current pose is as specified by the sample.
      q.setZero();
      dut.active_dof().SetInArray(sample.q_active, &q);
      dut.get_input_port_position().FixValue(context.get(), q);

      // Note: because R_WT = I, vd_TB = vd_WB.
      const Vector3d vd_TB = sample.vd_WB;
      // The robot can't exhibit angular velocity at all.
      const SpatialVelocity<double> Vd_TB(Vector3d{0, 0, 0}, vd_TB);
      desired_velocities.Set("robot::ball", Value{Vd_TB});
      dut.get_input_port_desired_cartesian_velocities().FixValue(
          context.get(), desired_velocities);

      const VectorXd commanded_velocity =
          dut.get_output_port_commanded_velocity().Eval(*context);
      results.push_back(commanded_velocity);

      const SpatialVelocity<double> V_WB_full =
          GetBallSpatialVelocity(dut.plant(), commanded_velocity);
      const SpatialVelocity<double> V_WB_masked(
          V_WB_full.rotational(),
          V_WB_full.translational().cwiseProduct(sample.v_WB_mask));
      const SpatialVelocity<double> V_WB_expected(
          Vector3d(0, 0, 0),
          sample.v_WB_expected.cwiseProduct(sample.v_WB_mask));

      EXPECT_TRUE(CompareMatrices(V_WB_masked.get_coeffs(),
                                  V_WB_expected.get_coeffs(), 1e-4));
    }
    return results;
  }
};

TEST_F(DifferentialInverseKinematicsTest, Structure) {
  // Add a few arbitrary ingredients (in arbitrary order), just so we can see
  // them in the owned recipe. These are the ingredients that can easily be
  // constructed.
  auto recipe = std::make_shared<Recipe>();
  recipe->AddIngredient(std::make_unique<DiffIk::LeastSquaresCost>(
      DiffIk::LeastSquaresCost::Config{}));
  recipe->AddIngredient(std::make_unique<DiffIk::CollisionConstraint>(
      DiffIk::CollisionConstraint::Config{}));
  recipe->AddIngredient(std::make_unique<DiffIk::JointCenteringCost>(
      DiffIk::JointCenteringCost::Config{}));
  recipe->AddIngredient(
      std::make_unique<DiffIk::CartesianVelocityLimitConstraint>(
          DiffIk::CartesianVelocityLimitConstraint::Config{
              .V_next_TG_limit = Vector6d::Zero()}));
  DiffIk dut = MakeDiffIk(std::move(recipe));

  // Port check.
  EXPECT_EQ(&dut.GetInputPort("position"), &dut.get_input_port_position());
  EXPECT_EQ(&dut.GetInputPort("nominal_posture"),
            &dut.get_input_port_nominal_posture());
  EXPECT_EQ(&dut.GetInputPort("desired_cartesian_velocities"),
            &dut.get_input_port_desired_cartesian_velocities());
  EXPECT_EQ(&dut.GetInputPort("desired_cartesian_poses"),
            &dut.get_input_port_desired_cartesian_poses());
  EXPECT_EQ(&dut.GetOutputPort("commanded_velocity"),
            &dut.get_output_port_commanded_velocity());

  // plant() aliases into the collision checker's plant.
  EXPECT_EQ(&dut.plant(), &dut.collision_checker().plant());

  // Construction parameters:
  EXPECT_EQ(dut.active_dof(),
            DofMask::MakeFromModel(dut.collision_checker().plant(),
                                   std::string(kRobotName)));
  EXPECT_EQ(dut.time_step(), kTimeStep);
  EXPECT_EQ(dut.task_frame().name(), kTaskFrame);
  EXPECT_EQ(dut.K_VX(), kK_VX);
  EXPECT_TRUE(CompareMatrices(dut.Vd_TG_limit().get_coeffs(),
                              MakeSpatialLimits().get_coeffs()));
  const Recipe& dut_recipe = dut.recipe();
  EXPECT_EQ(dut_recipe.num_ingredients(), 4);
  EXPECT_EQ(NiceTypeName::Get(dut_recipe.ingredient(0)),
            NiceTypeName::Get<DiffIk::LeastSquaresCost>());
  EXPECT_EQ(NiceTypeName::Get(dut_recipe.ingredient(1)),
            NiceTypeName::Get<DiffIk::CollisionConstraint>());
  EXPECT_EQ(NiceTypeName::Get(dut_recipe.ingredient(2)),
            NiceTypeName::Get<DiffIk::JointCenteringCost>());
  EXPECT_EQ(NiceTypeName::Get(dut_recipe.ingredient(3)),
            NiceTypeName::Get<DiffIk::CartesianVelocityLimitConstraint>());
}

/* A simple ingredient whose sole purpose is to examine the contents of details
and compare it with what we'd expect to see. */
class TestIngredient final : public Ingredient {
 public:
  TestIngredient() = default;
  ~TestIngredient() final = default;
  std::vector<Binding<EvaluatorBase>> AddToProgram(
      CallbackDetails* details) const final {
    DRAKE_DEMAND(details != nullptr);
    if (details->v_next.size() != dut_->active_dof().count()) {
      throw std::runtime_error("Detail has wrong number of decision variables");
    }
    if (&details->collision_checker != &dut_->collision_checker()) {
      throw std::runtime_error("Detail collision checker doesn't match system");
    }
    if (details->active_dof != dut_->active_dof()) {
      throw std::runtime_error("Detail active dof doesn't match system");
    }
    if (details->time_step != dut_->time_step()) {
      throw std::runtime_error("Detail time step doesn't match system");
    }
    // TODO(sean.curtis): Extend the test to confirm the details that arise from
    // the context: plant_context, nominal_posture, X_TGlist, VdTGlist, JvTGs,
    // and mathematical_program. For the program, we'll check: that it has no
    // constraints or costs and that details->v_next are its (only) decision
    // variables.
    return {};
  }

  void set_dut(const DiffIk* dut) {
    DRAKE_DEMAND(dut != nullptr);
    dut_ = dut;
  }

 private:
  const DiffIk* dut_{};
};

/* DiffIkSystem is responsible for collecting up the "details" of the program
for the recipe/ingredient. We'll confirm that the right values get passed. */
TEST_F(DifferentialInverseKinematicsTest, ImplementRecipe) {
  auto recipe = std::make_shared<Recipe>();
  // We need a quadratic cost function in order to evaluate the output.
  AddLeastSquaresCost(recipe.get());
  auto ingredient = std::make_shared<TestIngredient>();
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));
  ingredient->set_dut(&dut);

  // To trigger TestIngredient::AddToProgram(), we have to evaluate the output
  // port; so create a context, connect dummy values, and evaluate.
  auto context = dut.CreateDefaultContext();
  VectorXd q(dut.plant().num_positions());
  q.setZero();
  dut.get_input_port_position().FixValue(context.get(), q);
  dut.get_input_port_nominal_posture().FixValue(context.get(), q);
  BusValue desired_vel;
  desired_vel.Set("robot::ball", Value{SpatialVelocity<double>::Zero()});
  dut.get_input_port_desired_cartesian_velocities().FixValue(context.get(),
                                                             desired_vel);

  // If CallbackDetails don't match expectations, this throws.
  EXPECT_NO_THROW(dut.get_output_port_commanded_velocity().Eval(*context));
}

/* Test the correctness of the LeastSquares use of its axis mask.

We don't test LeastSquaresCost more generally. This cost is applied in all
constraint tests. Its properties are affirmed again and again based on the
projection of the desired velocity onto the convex hull of the feasible velocity
region. */
TEST_F(DifferentialInverseKinematicsTest, LeastSquaresAxisMask) {
  // Mask out everything except for vx, vy and the answer shouldn't change.
  // The "masked" rows were already zero.
  const string_unordered_map<Vector6d> identity_masks{
      {"robot::ball", Vector6d(0, 0, 0, 1, 1, 0)}};

  auto recipe = std::make_shared<Recipe>();
  auto ingredient = std::make_shared<DiffIk::LeastSquaresCost>(
      DiffIk::LeastSquaresCost::Config{.cartesian_axis_masks = identity_masks});
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));

  EvaluateSamples(dut, {{.description = "Only vx and vy included",
                         .vd_WB = Vector3d{1, -2, 0},
                         .v_WB_expected = Vector3d{1, -2, 0}}});

  /* Now run it again, with vy masked out; the solution ignores vd_y. */
  const string_unordered_map<Vector6d> vy_masked{
      {"robot::ball", Vector6d(0, 0, 0, 1, 0, 0)}};
  ingredient->SetConfig(
      DiffIk::LeastSquaresCost::Config{.cartesian_axis_masks = vy_masked});

  EvaluateSamples(dut, {{.description = "Only vx included",
                         .vd_WB = Vector3d{1, -2, 0},
                         .v_WB_expected = Vector3d{1, 0, 0},
                         .v_WB_mask = Vector3d(1, 0, 1)}});
}

/* We'll use the least-squares cost without constraints to confirm the value of
the desired velocity inferred from position; in the absence of constraints, the
least squares cost will always report the desired velocity as the solution.

We want to confirm two things:

  1. The inferred velocity includes both desired position and current position.
  2. The spatial velocity clamping is applied.

So, we'll set current position and desired position values such that velocity
will only be clamped if we compute their difference *and* we'll make sure they
get clamped. */
TEST_F(DifferentialInverseKinematicsTest, VelocityFromPosition) {
  auto recipe = std::make_shared<Recipe>();
  AddLeastSquaresCost(recipe.get());
  DiffIk dut = MakeDiffIk(std::move(recipe));
  // The limits used in construction of dut.
  const Vector3d vd_TB_limits = MakeSpatialLimits().translational();
  // The conversion factor from difference in position to inferred velocity.
  const double vel_scale = kK_VX / kTimeStep;
  // The minimum amount of displacement between current and desired poses to
  // introduce clamping. Note; we're zeroing out the Wz measure that gets
  // introduced by vd_WB_limits.
  const Vector3d clampable_displacement_T =
      (vd_TB_limits / vel_scale).cwiseProduct(Vector3d(1, 1, 0));

  // The current and desired positions which will require clamping. If the
  // current position were ignored, clamping would not occur.
  const Vector3d p_TB_init = -clampable_displacement_T / 2;
  const Vector3d pd_TB = p_TB_init + clampable_displacement_T * 1.1;
  const Vector3d p_WT = GetTaskOrigin();
  const Vector3d p_WB_init = p_WT + p_TB_init;
  const Vector3d pd_WB = p_WT + pd_TB;

  // Now we can run the system.
  auto context = dut.CreateDefaultContext();

  VectorXd q(dut.plant().num_positions());
  q.setZero();
  const Vector2d q_active_init(p_WB_init.x(), p_WB_init.y());
  dut.active_dof().SetInArray(q_active_init, &q);
  dut.get_input_port_nominal_posture().FixValue(context.get(), q);
  dut.get_input_port_position().FixValue(context.get(), q);

  BusValue desired_positions;
  desired_positions.Set("robot::ball", Value{RigidTransformd(pd_TB)});

  dut.get_input_port_desired_cartesian_poses().FixValue(context.get(),
                                                        desired_positions);

  const VectorXd commanded_velocity =
      dut.get_output_port_commanded_velocity().Eval(*context);

  const SpatialVelocity<double> V_WB =
      GetBallSpatialVelocity(dut.plant(), commanded_velocity);

  // The unconstrained velocity -- ignoring Vd_TG_limit. Again, v_WB = v_TB
  // because R_WT = I.
  const Vector3d v_WB_candidate = (pd_WB - p_WB_init) * (kK_VX / kTimeStep);
  const SpatialVelocity<double> V_WB_candidate(Vector3d(0, 0, 0),
                                               v_WB_candidate);

  auto my_clamp = [](double value, double limit) {
    return std::clamp(value, -limit, limit);
  };
  // The expected velocity, clamped by the Vd_TG_limit.
  const Vector3d v_WB_expected(my_clamp(v_WB_candidate[0], vd_TB_limits[0]),
                               my_clamp(v_WB_candidate[1], vd_TB_limits[1]), 0);
  const SpatialVelocity<double> V_WB_expected(Vector3d(0, 0, 0), v_WB_expected);

  // First, show that the candidate velocity doesn't match the expected due to
  // clamping.
  EXPECT_FALSE(
      CompareMatrices(V_WB.get_coeffs(), V_WB_candidate.get_coeffs(), 0.1));
  EXPECT_TRUE(
      CompareMatrices(V_WB.get_coeffs(), V_WB_expected.get_coeffs(), 1e-4));
}

/* The joint centering cost resolves scenarios where the optimal solution isn't
unique. The cost isn't _required_ -- an optimal solution will be produced
regardless.

The redundant robot with 3-dofs has a constant, configuration-independent Jv_TG.
The transform from q_active to the null space is simply P = [1/2, 1/2, -1/√2].
We also know that the so-called nominal pose is the zero vector (see
EvaluateSamples()). Therefore, for a given Vd_WB, we can pick a _specific_ pose
q that will shift v_next away from Osqp's default value.  See below for details.
*/
TEST_F(DifferentialInverseKinematicsTest, JointCenteringCost) {
  // The desired velocity. Note: the v_active that produces this with the
  // *smallest* projection into the nullspace is v₁ = (1.5, -0.5, 1/√2);
  // P⋅v₁ = 0. Curiously, that is what Osqp returns as a solution.
  const Vector3d vd_WB(2.0, 0, 0);

  // We can force the optimal solution to be the solution v₂ = [2, 0, 0] by
  // picking the right q. For the nominal pose q_n and the current pose q, the
  // secondary cost function is:
  //
  //    |P⋅(v_next - K⋅(q_n - q))|²
  //
  // We want v_next = v₂ So, we want to pick q such that we minimize:
  //
  //    |P⋅(v₂ - K⋅(q_n - q))|²
  //
  // This is at its minimum where:
  //
  //    P⋅v₂ = -2⋅P⋅(q_n - q)       - arbitrarily pick K = 2 (any value != 1).
  //       1 = -2⋅P⋅(q_n - q)       - P⋅v₂ = 1.
  //    -1/2 = P⋅(q_n - q)
  //
  // The equality P⋅(q_n - q) = -1/2 is (not uniquely) satisfied with
  // q_n - q = [-1/2, -1/2, 0]. We are free to pick q_n and q, so we pick
  // q_n = [1, 2, 0], q = [1/2, 3/2, 0].
  const Vector3d q_n(1, 2, 0);
  const Vector3d q(0.5, 1.5, 0);
  constexpr double K = 2.0;  // match the arbitrary non-unit value above.

  // We'll collect two velocity commands -- one without joint centering and one
  // with. We expect the two commands to be significantly different *and* that
  // the centered_command is, essentially, [2, 0, 0].

  std::vector<VectorXd> commands;
  for (bool add_centering : {false, true}) {
    auto recipe = std::make_shared<Recipe>();
    AddLeastSquaresCost(recipe.get());
    if (add_centering) {
      auto ingredient = std::make_shared<DiffIk::JointCenteringCost>(
          DiffIk::JointCenteringCost::Config{.posture_gain = K});
      recipe->AddIngredient(ingredient);
    }
    DiffIk dut = MakeDiffIk(std::move(recipe), /* add_redundant = */ true);

    std::vector<Sample> samples{
        {.description = fmt::format("Overactuated robot {} joint centering",
                                    add_centering ? "with" : "without"),
         .vd_WB = vd_WB,
         .v_WB_expected = vd_WB,
         .q_active = q,
         .q_active_nominal = q_n}};

    commands.push_back(EvaluateSamples(dut, samples)[0]);
  }
  const VectorXd& uncentered_command = commands[0];
  const VectorXd& centered_command = commands[1];

  // They don't match, even with a *large* tolerance.
  EXPECT_FALSE(CompareMatrices(uncentered_command, centered_command, 1e-1));

  // The centered command is our target velocity.
  Vector3d expected_command(2, 0, 0);
  EXPECT_TRUE(CompareMatrices(centered_command, expected_command, 1e-4));
}

/* We'll repeat the previous joint centering test, but, this time, we'll mask
out one of the two cartesian velocity elements.

By masking out vy, we change the null space of the Jacobian and its
corresponding P:

        |   0    1  0 |
    P = | -1/√2  0  1 |

This time, we'll set q_n to the zero configuration. This simplifies the cost
function. Therefore, to control the output, we want -P⋅v = P⋅q.

If I want the commanded velocity to be [1.5, 0, 0]ᵀ:

    P⋅v = P⋅[1.5, 0, 0]ᵀ = [0, -1.5/√2]ᵀ

So,

   [0, 1.5/√2]ᵀ = P⋅q  -->  q = [-0.5, 0, 1/√2]ᵀ
*/
TEST_F(DifferentialInverseKinematicsTest, JointCenteringCostAxisMask) {
  const Vector3d vd_WB(1.5, 2.0, 0);
  const Vector3d expected_command(1.5, 0, 0);
  // q picked such that we get the expected command.
  const Vector3d q(-0.5, 0, 1 / std::sqrt(2));

  // Masking out vy.
  const string_unordered_map<Vector6d> masks{
      {"robot::ball", Vector6d(0, 0, 0, 1, 0, 0)}};
  const Vector3d vd_WB_masked(1.5, 0, 0);

  std::vector<VectorXd> commands;
  for (bool add_centering : {false, true}) {
    auto recipe = std::make_shared<Recipe>();
    AddLeastSquaresCost(recipe.get(), masks);
    if (add_centering) {
      auto ingredient = std::make_shared<DiffIk::JointCenteringCost>(
          DiffIk::JointCenteringCost::Config{.cartesian_axis_masks = masks});
      recipe->AddIngredient(ingredient);
    }
    DiffIk dut = MakeDiffIk(std::move(recipe), true);

    std::vector<Sample> samples{
        {.description = fmt::format(
             "Overactuated robot {} joint centering with selection mask",
             add_centering ? "with" : "without"),
         .vd_WB = vd_WB,
         .v_WB_expected = vd_WB_masked,
         .v_WB_mask = Vector3d(1, 0, 1),
         .q_active = q}};

    commands.push_back(EvaluateSamples(dut, samples)[0]);
  }
  const VectorXd& uncentered_command = commands[0];
  const VectorXd& centered_command = commands[1];

  EXPECT_FALSE(CompareMatrices(uncentered_command, centered_command, 1e-1));

  EXPECT_TRUE(CompareMatrices(centered_command, expected_command, 1e-4));
}

/* We're starting the ball at p_WB = (0, 0, 0). The cartesian position limit
constraint limits the velocity by placing a box around the current position and
disallowing velocities that would cause the position to leave the box in *one*
time step.

We'll create a rectangular box around the position and show that as long as
p_WB + Vd_WB * Δt lies within the box, then the velocity is accepted, otherwise
it is projected.

         vd₀     vd₁          vd₀ and vd₁ lie outside the constraint and get
          ●       ●           projected onto the box.
          ┆       ┆
     2 ┼──○───────○────┐      vd₂ lies within the box and is the optimal
       │               │      solution.
     0 ┼  ·       ●    │
       │ p_WB     vd₂  │
    -2 ┼──┼────────────┤
      -1  0            5
*/
TEST_F(DifferentialInverseKinematicsTest, CartesianPositionLimitConstraint) {
  auto recipe = std::make_shared<Recipe>();
  AddLeastSquaresCost(recipe.get());
  auto ingredient = std::make_shared<DiffIk::CartesianPositionLimitConstraint>(
      DiffIk::CartesianPositionLimitConstraint::Config{
          .p_TG_next_lower = Vector3d(-1, -2, 0),
          .p_TG_next_upper = Vector3d(5, 2, 0)});
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));

  std::vector<Sample> samples{
      {.description = "vd0 - velocity clipped; direction unchanged",
       .vd_WB = Vector3d{0, 3 / kTimeStep, 0},
       .v_WB_expected = Vector3d{0, 2 / kTimeStep, 0}},
      {.description = "vd1 - velocity projected; direction change",
       .vd_WB = Vector3d{3 / kTimeStep, 3 / kTimeStep, 0},
       .v_WB_expected = Vector3d{3 / kTimeStep, 2 / kTimeStep, 0}},
      {.description = "vd2 - velocity already feasible",
       .vd_WB = Vector3d{3 / kTimeStep, 0, 0},
       .v_WB_expected = Vector3d{3 / kTimeStep, 0, 0}},
  };

  EvaluateSamples(dut, samples);
}

/* The Cartesian velocity constraint places a box in _cartesian velocity_ space.
Joint velocities must map to a cartesian velocity within the box; if the optimal
velocity lies outside the box, it gets projected onto the box.

The velocity box is strictly defined to be centered on the zero velocity.

                    vd₁
                      ●         vd₀ and vd₁ are outside the feasible region of
                      ┆         velocities and get projected onto it.
          1 ┼─────────○────┐
   vd₀ ●┄┄┄┄○              │    vd₂ is already feasible.
            │              │
            │       vd₂ ●  │
         -1 ┼──────────────┤
            -2             2
*/
TEST_F(DifferentialInverseKinematicsTest, CartesianVelocityLimitConstraint) {
  auto recipe = std::make_shared<Recipe>();
  AddLeastSquaresCost(recipe.get());
  auto ingredient = std::make_shared<DiffIk::CartesianVelocityLimitConstraint>(
      DiffIk::CartesianVelocityLimitConstraint::Config{
          .V_next_TG_limit = Vector6d(0, 0, 0, 2, 1, 0)});
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));

  std::vector<Sample> samples{
      {.description = "vd0 - velocity clipped on the x-axis",
       .vd_WB = Vector3d{-2.5, 0.5, 0},
       .v_WB_expected = Vector3d{-2, 0.5, 0}},
      {.description = "vd1 - velocity clipped on the y-axis",
       .vd_WB = Vector3d{1, 3, 0},
       .v_WB_expected = Vector3d{1, 1, 0}},
      {.description = "vd2 - velocity already feasible",
       .vd_WB = Vector3d{1.5, -0.5, 0},
       .v_WB_expected = Vector3d{1.5, -0.5, 0}},
  };

  EvaluateSamples(dut, samples);
}

/* CollisionConstraint constrains the next velocity by representing an obstacle
with a half space constraint in velocity space. The normal of the half space
boundary aligns with the gradient of the signed distance function. The plane's
distance to the zero-velocity origin is the speed necessary to travel the
signed distance in a single time step (ϕ/Δt).

We'll create a scenario using the spherical obstacle located at the origin and
compute velocities for the robot point to work its way past.

    Cartesian Space                     Velocity Space

                  Wy
                  ┆                                          vy
                **┆**                           vd₁░░┃        ┆
              *  ·┆·  *  ├── ϕ ─┤                ●┄┄┄○        ┆
            * ·   ┆   · *                       ░░░░░┃ vd₀    ┆
           * ·    ┆    · *                      ░░░░░┃  ●     ┆
    ┄┄┄┄┄┄┄*┄·┄┄┄┄┼┄┄┄┄·┄*┄┄┄┄┄┄⊙┄ Wx           ┄┄┄┄┄┃┄┄┄┄┄┄┄┄┼┄┄┄┄  vx
           *  ·   │   ·  *                      ░░░░░┃        ┆
             *   ·┆·   *                        ░░░░░┃        ┆
                **┆**                           ░░░░░├─ ϕ/Δt ─┤
            -1    │    1

    · - obstacle boundary                ░┃ - velocity-space proxy for obstacle
    * - obstacle with safety offset      vd₀ is feasible and is taken as
    ⊙ - "ball" object                        solution
    ϕ - distance from ball to safety     vd₁ is infeasible and gets projected
        boundary

In this test we're testing the following:
  - If the obstacle is included:
    - feasible velocities are passed through
    - infeasible velocities are clipped.
  - If the obstacle is omitted, the velocity is unconstrained. It may be omitted
    for one of two reasons:
    - being outside of the influence, or
    - because the "select data" callback. */
TEST_F(DifferentialInverseKinematicsTest, CollisionConstraint) {
  const double safety_dist = 0.25;
  // Obstacle geometry is a sphere with radius 1.
  const double obstacle_safety_radius = 1.0 + safety_dist;
  constexpr double phi = 1.0;
  // The speed in the Wx direction necessary to "hit" the velocity in one
  // time step.
  constexpr double kCollisionVelocity = -phi / kTimeStep;
  // We'll position the ball to achieve the target ϕ and ∇ϕ = [1, 0, 0]:
  // p_WB = [ϕ + safety_radius, 0]).
  const Vector2d p_WB(phi + obstacle_safety_radius, 0);
  const Vector3d p_WT = GetTaskOrigin();
  const Vector2d p_TB_W = p_WB - p_WT.head<2>();

  const VectorXd q_active_init = p_TB_W;

  auto recipe = std::make_shared<Recipe>();
  AddLeastSquaresCost(recipe.get());
  using Config = DiffIk::CollisionConstraint::Config;
  auto ingredient = std::make_shared<DiffIk::CollisionConstraint>(Config{});
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));

  // Collision constraint introduced; influence >> phi.
  ingredient->SetConfig(
      Config{.safety_distance = safety_dist, .influence_distance = phi * 10});

  // vd0 is feasible and passes through.
  const Vector3d vd0(0.9 * kCollisionVelocity, 0.5, 0);
  EvaluateSamples(dut, {{.description = "vd0 - already feasible",
                         .vd_WB = vd0,
                         .v_WB_expected = vd0,
                         .q_active = q_active_init}});

  // vd1 is infeasible and gets clipped.
  const Vector3d vd1(2 * kCollisionVelocity, 0.5, 0);
  const Vector3d vd1_clipped(kCollisionVelocity, vd1.y(), vd1.z());
  EvaluateSamples(dut, {{.description = "vd1 - clipped by the half space",
                         .vd_WB = vd1,
                         .v_WB_expected = vd1_clipped,
                         .q_active = q_active_init}});

  // vd1 is infeasible but the obstacle is filtered out by influence < phi.
  ingredient->SetConfig(
      Config{.safety_distance = safety_dist, .influence_distance = phi * 0.9});
  EvaluateSamples(dut, {{.description = "vd1 - influence ignores obstacle",
                         .vd_WB = vd1,
                         .v_WB_expected = vd1,
                         .q_active = q_active_init}});

  // vd1 is infeasible but the obstacle is filtered out by callback.
  ingredient->SetConfig(
      Config{.safety_distance = safety_dist, .influence_distance = phi * 10});
  ingredient->SetSelectDataForCollisionConstraintFunction(
      [](const DofMask&, const RobotClearance&, VectorXd*, Eigen::MatrixXd*) {
      });
  EvaluateSamples(dut, {{.description = "vd1 - callback ignores obstacle",
                         .vd_WB = vd1,
                         .v_WB_expected = vd1,
                         .q_active = q_active_init}});
}

TEST_F(DifferentialInverseKinematicsTest, CollisionConstraintThreaded) {
  // Make a simple recipe with a CollisionConstraint and an arbitrary cost.
  auto recipe = std::make_shared<Recipe>();
  AddLeastSquaresCost(recipe.get());
  using Config = DiffIk::CollisionConstraint::Config;
  auto ingredient = std::make_shared<DiffIk::CollisionConstraint>(Config{});
  ingredient->SetConfig(
      Config{.safety_distance = 0.25, .influence_distance = 10.0});
  recipe->AddIngredient(ingredient);
  const DiffIk dut = MakeDiffIk(std::move(recipe));

  // A simple functor to exercise the DUT's computation.
  auto do_work = [&dut](auto* context) {
    // Set some arbitrary inputs.
    const VectorXd q = VectorXd::Zero(dut.plant().num_positions());
    dut.get_input_port_nominal_posture().FixValue(context, q);
    dut.get_input_port_position().FixValue(context, q);
    BusValue desired_velocities;
    const auto Vd_TB = SpatialVelocity<double>::Zero();
    desired_velocities.Set("robot::ball", Value{Vd_TB});
    dut.get_input_port_desired_cartesian_velocities().FixValue(
        context, desired_velocities);
    // Evaluate the output.
    dut.get_output_port_commanded_velocity().Eval(*context);
  };

  // Run two do_work functions concurrently. Our TSan (etc.) might trip if there
  // are concurrency bugs.
  auto context_1 = dut.CreateDefaultContext();
  auto context_2 = context_1->Clone();
  auto context_3 = dut.CreateDefaultContext();
  auto future_1 = std::async(std::launch::async, do_work, context_1.get());
  auto future_2 = std::async(std::launch::async, do_work, context_2.get());
  auto future_3 = std::async(std::launch::async, do_work, context_3.get());
  future_1.get();
  future_2.get();
  future_3.get();
}

/* The constraint on joint velocities is conceptually simple, but has some
implementations that make it tricky. Rather than simply applying a bounding box
in the joint velocity space based on the declared joint velocity limits, the
constraint modifies the domain of the box based on where the robot configuration
is relative to the declared joint position limits. The closer to a position
boundary `q` is the tighter the velocity bound towards that boundary is. So,
we'll have to test with various q to see the effect of the constraint.

Note: the box isn't simply scaled; the boundaries on one side are moved. Limits
in the direction opposite the near boundary are unchanged.

We'll bound the robot to move within a region.

            Cartesian space

            Wy
             ┆
    2  ┏━━━━━┿━━━━━━━━━━━━━━━┓
       ┃ ┌───┼─────────────┐ ┃            ┃ - q limits boundary
       ┃ │   ┆         q_b │ ┃q_c         │ - padded boundary; outside this
       ┃ │   ┆┌┉┉┉┉┉┉┉┐    │ ┃    q_d         boundary, velocity @ 0%.
       ┃ │   ┆┊ q_a ★ ┊ ★  │★┃    ★       ┊ - padded boundary; inside this
 ┄┄┄┄┄┄╂┄┼┄┄┄┼└┉┉┉┉┉┉┉┘┄┄┄┄┼┄╂┄┄ Wx           boundary, velocity limit @ 100%.
       ┃ │ ★ ┆ q_e         │ ┃            ★ - configuration at which we'll
       ┃ └───┼─────────────┘ ┃                evaluate diff ik. Each one is in a
   -1  ┗━━━━━┿━━━━━━━━━━━━━━━┛                region of interest.
      -1     ┆               3

            Velocity space

           Vy
            ┆
     2 ┌────┼─────────┐
       │    ┆         │
       │    ┆         │
       │    ┆         │
       │    ┆         │
    ┄┄┄┼┄┄┄┄┼┄┄┄┄┄┄┄┄┄┼┄┄┄┄ Vx      │ - limits on v
       │    ┆         │
  -0.5 └────┼─────────┘
      -1    ┆        1.5

Ideally, we would configure the robot so that q is nearest each position
boundary in turn. We're only sampling a portion of these, relying on the
structure of the code to pick up the per-axis symmetry we need for free. */
TEST_F(DifferentialInverseKinematicsTest, JointVelocityLimitConstraint) {
  // If q is less than 0.25 units away from the q-limit boundary, the velocity
  // limit is zero.
  const double kMargin = 0.25;
  // If q is at least 0.5 units away from the q-limit boundary, the full
  // velocity limit will be used.
  const double kInfluence = 0.5;
  // clang-format off
  // x- and y- quantities map to q1 and q0, respectively.
  const JointLimits joint_limits(
      Vector2d(-1, -1),      /* q lower */
      Vector2d(3, 2),        /* q upper */
      Vector2d(-1, -0.5),    /* v lower */
      Vector2d(1.5, 2),      /* v upper */
      Vector2d(0, 0),        /* a lower (unused) */
      Vector2d(0, 0));       /* a upper (unused) */
  // clang-format on

  // Velocity values at the boundaries of specified limits.
  const auto vel_lower =
      (Vector3d() << joint_limits.velocity_lower(), 0).finished();
  const auto vel_upper =
      (Vector3d() << joint_limits.velocity_upper(), 0).finished();

  const Vector3d vel_offset(0.1, 0.1, 0);

  const auto& q_lower = joint_limits.position_lower();
  const auto& q_upper = joint_limits.position_upper();
  const Vector2d q_center = (q_lower + q_upper) / 2;
  // At q_a, the full joint limits should apply.
  const Vector2d q_a(q_upper[0] - kMargin - kInfluence - 0.1, q_center[1]);
  // At q_b, the velocity limit in the +v0 (+Wx) direction is cut in half.
  const double decay_width = kInfluence - kMargin;
  const Vector2d q_b(q_upper[0] - kMargin - decay_width / 2, q_center[1]);
  // At q_c, the velocity limit in the +v0 (+Wx) direction is zero.
  const Vector2d q_c(q_upper[0] - 0.5 * kMargin, q_center[1]);
  // q_d lies outside the position limits; the velocity limit in the +V0
  // direction is zero.
  const Vector2d q_d(q_upper[0] + 1, q_center[1]);
  // q_e lies 3/4 of the way towards the zero boundary, but towards the bottom
  // of the position limits.
  const Vector2d q_e(q_lower[0] + kMargin + decay_width / 4,
                     q_lower[1] + kMargin + decay_width / 4);

  auto recipe = std::make_shared<Recipe>();
  AddLeastSquaresCost(recipe.get());
  auto ingredient = std::make_shared<DiffIk::JointVelocityLimitConstraint>(
      DiffIk::JointVelocityLimitConstraint::Config{
          .min_margin = kMargin, .influence_margin = kInfluence},
      joint_limits);
  recipe->AddIngredient(ingredient);
  DiffIk dut = MakeDiffIk(std::move(recipe));

  // Note: in the description below, an "outward Vd" is towards the near
  // boundary. Similarly, an "inward" Vd points away from the near boundary.
  std::vector<Sample> samples{
      {.description = "At q_a, clamp outward Vd to standard joint limits",
       .vd_WB = vel_upper + vel_offset,
       .v_WB_expected = vel_upper,
       .q_active = q_a},
      {.description = "At q_a, feasible Vd is still feasible",
       .vd_WB = vel_upper - vel_offset,
       .v_WB_expected = vel_upper - vel_offset,
       .q_active = q_a},
      {.description = "At q_a, clamp inward Vd to standard joint limits",
       .vd_WB = vel_lower - vel_offset,
       .v_WB_expected = vel_lower,
       .q_active = q_a},
      {.description = "At q_b, clamp outward Vd to half standard joint limits",
       .vd_WB = vel_upper,
       .v_WB_expected = vel_upper.cwiseProduct(Vector3d(0.5, 1, 0)),
       .q_active = q_b},
      {.description = "At q_b, inward Vd already feasible",
       .vd_WB = vel_lower * 0.49,
       .v_WB_expected = vel_lower * 0.49,
       .q_active = q_b},
      {.description = "At q_b, clamp inward Vd to standard joint limits",
       .vd_WB = vel_lower - vel_offset,
       .v_WB_expected = vel_lower,
       .q_active = q_b},
      {.description = "At q_c, clamp outward Vd to zero",
       .vd_WB = vel_upper,
       .v_WB_expected = vel_upper.cwiseProduct(Vector3d(0, 1, 0)),
       .q_active = q_c},
      {.description = "At q_c, inward Vd already feasible",
       .vd_WB = vel_lower,
       .v_WB_expected = vel_lower,
       .q_active = q_c},
      {.description = "At q_d, clamp outward Vd to zero",
       .vd_WB = vel_upper,
       .v_WB_expected = vel_upper.cwiseProduct(Vector3d(0, 1, 0)),
       .q_active = q_d},
      {.description = "At q_d, inward Vd already feasible",
       .vd_WB = vel_lower,
       .v_WB_expected = vel_lower,
       .q_active = q_d},
      {.description = "At q_e, clamp outward Vd to 1/4 limit",
       .vd_WB = vel_lower - vel_offset,
       .v_WB_expected = vel_lower * 0.25,
       .q_active = q_e},
      {.description = "At q_e, feasible Vd is still feasible",
       .vd_WB = vel_upper * 0.5,
       .v_WB_expected = vel_upper * 0.5,
       .q_active = q_e},
      {.description = "At q_e, clamp inward Vd to standard joint limits",
       .vd_WB = vel_upper + vel_offset,
       .v_WB_expected = vel_upper,
       .q_active = q_e},
  };

  EvaluateSamples(dut, samples);
}

/* This tests all of the config structs getters and serialization
implementations. */
TEST_F(DifferentialInverseKinematicsTest, ConfigCommonOperations) {
  const string_unordered_map<Vector6d> masks{
      {"dummy", Vector6d(1, 0, 1, 0, 1, 0)}};

  {
    DiffIk::LeastSquaresCost i(
        DiffIk::LeastSquaresCost::Config{.cartesian_qp_weight = kQpWeight,
                                         .cartesian_axis_masks = masks,
                                         .use_legacy_implementation = true});
    EXPECT_EQ(SaveYamlString(i.GetConfig()),
              "cartesian_qp_weight: 2.5\n"
              "cartesian_axis_masks:\n"
              "  dummy: [1.0, 0.0, 1.0, 0.0, 1.0, 0.0]\n"
              "use_legacy_implementation: true\n");
  }

  {
    DiffIk::JointCenteringCost i(DiffIk::JointCenteringCost::Config{
        .posture_gain = 0.75, .cartesian_axis_masks = masks});
    EXPECT_EQ(SaveYamlString(i.GetConfig()),
              "posture_gain: 0.75\n"
              "cartesian_axis_masks:\n"
              "  dummy: [1.0, 0.0, 1.0, 0.0, 1.0, 0.0]\n");
  }

  {
    DiffIk::CartesianPositionLimitConstraint i(
        DiffIk::CartesianPositionLimitConstraint::Config{
            .p_TG_next_lower = Vector3d(1, 2, 3),
            .p_TG_next_upper = Vector3d(4, 5, 6)});
    EXPECT_EQ(SaveYamlString(i.GetConfig()),
              "p_TG_next_lower: [1.0, 2.0, 3.0]\n"
              "p_TG_next_upper: [4.0, 5.0, 6.0]\n");
  }

  {
    DiffIk::CartesianVelocityLimitConstraint i(
        DiffIk::CartesianVelocityLimitConstraint::Config{
            .V_next_TG_limit = Vector6d(1, 2, 3, 4, 5, 6)});
    EXPECT_EQ(SaveYamlString(i.GetConfig()),
              "V_next_TG_limit: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]\n");
  }

  {
    DiffIk::CollisionConstraint i(DiffIk::CollisionConstraint::Config{
        .safety_distance = 0.75, .influence_distance = 1.5});
    EXPECT_EQ(SaveYamlString(i.GetConfig()),
              "safety_distance: 0.75\n"
              "influence_distance: 1.5\n");
  }

  {
    DiffIk::JointVelocityLimitConstraint i(
        DiffIk::JointVelocityLimitConstraint::Config{.min_margin = 0.75,
                                                     .influence_margin = 1.5},
        JointLimits());
    EXPECT_EQ(SaveYamlString(i.GetConfig()),
              "min_margin: 0.75\n"
              "influence_margin: 1.5\n");
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
