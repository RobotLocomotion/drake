#include "drake/planning/collision_checker.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <common_robotics_utilities/openmp_helpers.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/test/planning_test_helpers.h"
#include "drake/planning/unimplemented_collision_checker.h"

namespace drake {
namespace planning {

using test::AddChain;

// Support for easily comparing EdgeMeasurements.
bool operator==(const EdgeMeasure& r1, const EdgeMeasure& r2) {
  bool equal = true;
  equal &= r1.distance() == r2.distance();
  equal &= r1.completely_free() == r2.completely_free();
  equal &= r1.partially_free() == r2.partially_free();
  if (r1.partially_free()) {
    equal &= r1.alpha() == r2.alpha();
  }
  return equal;
}

std::ostream& operator<<(std::ostream& out, const EdgeMeasure& r) {
  out << "EdgeMeasure(" << r.distance() << ", " << r.alpha_or(-1.0) << ")";
  return out;
}

namespace {

#if defined(_OPENMP)
constexpr bool kHasOpenmp = true;
#else
constexpr bool kHasOpenmp = false;
#endif

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::CollisionFilterDeclaration;
using geometry::FrameId;
using geometry::GeometryId;
using geometry::GeometrySet;
using geometry::Role;
using geometry::Shape;
using geometry::Sphere;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::BodyIndex;
using multibody::CoulombFriction;
using multibody::default_model_instance;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::RevoluteJoint;
using multibody::RigidBody;
using multibody::world_model_instance;
using std::optional;
using std::pair;
using std::vector;
using systems::Context;
using testing::ElementsAre;

// Adds a new model instance consisting of a non-zero number of floating bodies.
ModelInstanceIndex AddEnvironmentModelInstance(MultibodyPlant<double>* plant,
                                               int num_geo = 1) {
  // We want to be able to call this without model instances to conflict, so
  // we'll simply track how many times we call it.
  static int count = 0;
  const ModelInstanceIndex instance =
      plant->AddModelInstance(fmt::format("env{}", ++count));

  DRAKE_DEMAND(plant->geometry_source_is_registered());

  std::vector<const RigidBody<double>*> bodies;
  const RigidBody<double>& body = plant->AddRigidBody("env_body", instance);
  for (int i = 0; i < num_geo; ++i) {
    plant->RegisterCollisionGeometry(body, RigidTransformd::Identity(),
                                     Sphere(0.01), fmt::format("g{}", i),
                                     CoulombFriction<double>());
  }
  return instance;
}

// This is a minimal derived collision checker for unit testing. By default,
// this checker does *not* allocate contexts. However, it makes the allocation
// functionality public for testing.
class CollisionCheckerTester : public UnimplementedCollisionChecker {
 public:
  explicit CollisionCheckerTester(CollisionCheckerParams params,
                                  bool supports_parallel_checking = false)
      : UnimplementedCollisionChecker(
            MaybeNerfParamParallelism(std::move(params),
                                      supports_parallel_checking),
            supports_parallel_checking) {}

  //@{
  // Interesting virtual overrides.
  std::unique_ptr<CollisionChecker> DoClone() const override {
    return std::make_unique<CollisionCheckerTester>(*this);
  }

  void DoUpdateContextPositions(
      CollisionCheckerContext* model_context) const override {
    // We need to confirm two things promised by CollisionChecker:
    //   - model_context != nullptr
    //   - The plant's positions have been updated.
    ASSERT_TRUE(model_context != nullptr);
    latest_positions_ = plant().GetPositions(model_context->plant_context());
  }

  bool DoCheckContextConfigCollisionFree(
      const CollisionCheckerContext& model_context) const override {
    // CollisionChecker promises that positions have been updated from public
    // method arguments.
    positions_for_check_ = plant().GetPositions(model_context.plant_context());
    return collision_free_;
  }

  optional<GeometryId> DoAddCollisionShapeToBody(
      const std::string&, const RigidBody<double>&, const Shape&,
      const RigidTransform<double>&) override {
    return added_shape_id_;
  }

  void RemoveAddedGeometries(
      const std::vector<CollisionChecker::AddedShape>& shapes) override {
    // Don't need to do work; just don't throw.
  }

  void UpdateCollisionFilters() override {
    // Capture the updated collision filter matrix.
    updated_filter_matrix_ = GetFilteredCollisionMatrix();
  }

  RobotClearance DoCalcContextRobotClearance(
      const CollisionCheckerContext&, double influence_dist) const override {
    // CollisionChecker promises that influence_dist is finite and non-negative.
    EXPECT_GE(influence_dist, 0.0);
    EXPECT_TRUE(std::isfinite(influence_dist));

    // To confirm CollisionChecker takes responsibility for zeroing out non
    // robot columns, we'll populate non-zero values in *every* column.
    const int q_size = plant().num_positions();
    RobotClearance clearance{plant().num_positions()};
    VectorXd Jq_dist(q_size);
    for (int i = 0; i < q_size; ++i) {
      Jq_dist(i) = i + 1;
    }
    clearance.Append(BodyIndex(1), BodyIndex(0),
                     RobotCollisionType::kEnvironmentCollision, 1.5, Jq_dist);
    return clearance;
  }

  std::vector<RobotCollisionType> DoClassifyContextBodyCollisions(
      const CollisionCheckerContext& model_context) const override {
    // CollisionChecker promises that positions have been updated from public
    // method arguments.
    positions_for_classify_ =
        plant().GetPositions(model_context.plant_context());
    return {};
  }

  int32_t DoMaxContextNumDistances(
      const CollisionCheckerContext&) const override {
    return 13;
  }
  //@}

  //@{
  // Protected methods required in testing.
  using CollisionChecker::AllocateContexts;
  using CollisionChecker::GetMutableSetupModel;

  // Note: private overload foils a plain "using" statement.
  std::string CriticizePaddingMatrix() const {
    return CollisionChecker::CriticizePaddingMatrix();
  }
  //@}

  //@{
  // Testing knobs.
  void SetCollisionFree(bool value) { collision_free_ = value; }

  // This ultimately controls whether *any* of the AddCollisionShape APIs will
  // report "shape added". All of the APIs ultimately delegate to
  // AddCollisionShapeToBody which returns this value.
  void SetCanAddCollisionShapes(bool value) {
    if (value) {
      added_shape_id_ = GeometryId::get_new_id();
    } else {
      added_shape_id_ = std::nullopt;
    }
  }

  // Returns the latest positions set in this checker via Update*Positions().
  const VectorXd& latest_positions() const { return latest_positions_; }

  // Returns the positions available in this checker from the most recent
  // Classify*BodyCollisions() call.
  const VectorXd& positions_for_classify() const {
    return positions_for_classify_;
  }

  // Returns the positions available in this checker from the most recent
  // Check*ConfigCollisionFree() call.
  const VectorXd& positions_for_check() const { return positions_for_check_; }

  // Returns the filter matrix available in this checker from the most recent
  // operation that modified the filter matrix.
  const Eigen::MatrixXi& updated_filter_matrix() const {
    return updated_filter_matrix_;
  }
  //@}

 private:
  static CollisionCheckerParams MaybeNerfParamParallelism(
      CollisionCheckerParams params, bool supports_parallel_checking) {
    if (!supports_parallel_checking) {
      params.implicit_context_parallelism = Parallelism::None();
    }
    return params;
  }

  bool collision_free_{false};
  optional<GeometryId> added_shape_id_;
  // Updated with every call to DoUpdateContextPositions().
  mutable VectorXd latest_positions_;
  // Updated with every call to DoClassifyContextBodyCollisions().
  mutable VectorXd positions_for_classify_;
  // Updated with every call to DoCheckContextConfigCollisionFree().
  mutable VectorXd positions_for_check_;
  // Updated with every call to UpdateCollisionFilters().
  Eigen::MatrixXi updated_filter_matrix_;
};

// Makes a test checker for the given model and enumerated set of robot
// model instances - the contexts are *not* allocated.
std::unique_ptr<CollisionCheckerTester> MakeUnallocatedChecker(
    std::unique_ptr<RobotDiagram<double>> robot,
    const vector<ModelInstanceIndex>& robot_indices,
    bool supports_parallel = true) {
  const ConfigurationDistanceFunction dist{
      [](const VectorXd& a, const VectorXd& b) {
        return (b - a).norm();
      }};
  return std::make_unique<CollisionCheckerTester>(
      CollisionCheckerParams{.model = std::move(robot),
                             .robot_model_instances = robot_indices,
                             .configuration_distance_function = dist,
                             .edge_step_size = 0.1,
                             .env_collision_padding = 0,
                             .self_collision_padding = 0},
      supports_parallel);
}

struct ModelConfig {
  bool weld_robot{true};
  int per_body_geometries{1};
  bool on_env_base{false};  // Ignored if weld_robot is true.
};

// Creates a model with robot and environment bodies, returning the model and
// the model's robot's model instance index.
pair<std::unique_ptr<RobotDiagram<double>>, ModelInstanceIndex> MakeModel(
    const ModelConfig& config = {}) {
  RobotDiagramBuilder<double> builder;
  auto& builder_plant = builder.plant();
  // Add the robot -- a chain of three links -- and weld it to the world.
  // Don't change the ordering or the length of the chain; the tests for this
  // fixture rely on knowing body indices.
  auto robot_index = AddChain(&builder_plant, 3, config.per_body_geometries);
  if (config.weld_robot) {
    builder_plant.WeldFrames(builder_plant.get_body(BodyIndex(0)).body_frame(),
                             builder_plant.get_body(BodyIndex(1)).body_frame());
  } else if (config.on_env_base) {
    const ModelInstanceIndex instance =
        builder_plant.AddModelInstance("floating");
    const RigidBody<double>& floater =
        builder_plant.AddRigidBody("floater", instance);
    // Connect the first chain body (1) to this floating base.
    builder_plant.AddJoint<RevoluteJoint>("floating", floater, {},
                                          builder_plant.get_body(BodyIndex(1)),
                                          {}, Eigen::Vector3d::UnitY());
  }

  // Add environment model instances; don't track the model instance indices,
  // because we don't care.
  AddEnvironmentModelInstance(&builder_plant, config.per_body_geometries);
  AddEnvironmentModelInstance(&builder_plant, config.per_body_geometries);
  return {builder.Build(), robot_index};
}

// Reports the indices of the dofs of the robot's environmental floating base.
// These indices only apply if MakeModel was called with weld_robot = false and
// on_env_base = true.
std::set<int> GetFloatingBaseDofs() {
  return {0, 1, 2, 3, 4, 5, 6};
}

// In the case where we mount the robot on an environment floating base, we
// want to make sure that GetFloatingBaseDofs() returns the right values. This
// confirms them -- it is used to test RobotClearance.
GTEST_TEST(MakeModel, EnvironmentalFloatingBase) {
  auto [robot, robot_index] =
      MakeModel({.weld_robot = false, .on_env_base = true});
  const auto& floater = robot->plant().GetBodyByName("floater");
  ASSERT_TRUE(floater.is_floating_base_body());
  ASSERT_TRUE(floater.has_quaternion_dofs());

  const std::set<int> base_dofs = GetFloatingBaseDofs();
  const int start = floater.floating_positions_start();
  for (int i = start; i < start + 7; ++i) {
    EXPECT_TRUE(base_dofs.contains(i));
  }
}

class CollisionCheckerThrowTest : public testing::Test {
 public:
  void ExpectConstructorThrow(const std::string& throw_message_pattern) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        CollisionCheckerTester(
            {.model = std::move(diagram_),
             .robot_model_instances = robot_model_instances_,
             .configuration_distance_function = distance_fn_,
             .edge_step_size = edge_step_size_,
             .env_collision_padding = env_collision_padding_,
             .self_collision_padding = self_collision_padding_}),
        throw_message_pattern);
  }

 protected:
  // The default values are all legal. Tests will invalidate specific values.
  RobotDiagramBuilder<double> builder_;
  std::unique_ptr<RobotDiagram<double>> diagram_{builder_.Build()};
  std::vector<ModelInstanceIndex> robot_model_instances_{
      default_model_instance()};
  ConfigurationDistanceFunction distance_fn_{
      [](const VectorXd&, const VectorXd&) {
        return 0.0;
      }};
  double edge_step_size_{0.1};
  double env_collision_padding_{0.0};
  double self_collision_padding_{0.0};
};

TEST_F(CollisionCheckerThrowTest, BadStepSize) {
  edge_step_size_ = 0.0;
  ExpectConstructorThrow(".*edge_step_size.*");
}

TEST_F(CollisionCheckerThrowTest, NoRobots) {
  robot_model_instances_ = {};
  ExpectConstructorThrow(".*robot.* > 0.*");
}

TEST_F(CollisionCheckerThrowTest, WorldRobot) {
  robot_model_instances_ = {world_model_instance()};
  ExpectConstructorThrow(".*robot.* != *.world.*");
}

// When the user passes duplicate and/or out-of-order robots, the constructor
// sorts them.
GTEST_TEST(CollisionCheckerTest, SortedRobots) {
  RobotDiagramBuilder<double> builder;
  auto& plant = builder.plant();
  const ModelInstanceIndex robot1 = AddChain(&plant, 1);
  const ModelInstanceIndex robot2 = AddChain(&plant, 2);
  auto distance_function = [](auto...) {
    return 0;
  };
  auto dut = std::make_unique<CollisionCheckerTester>(CollisionCheckerParams{
      .model = builder.Build(),
      .robot_model_instances =
          std::vector<ModelInstanceIndex>{robot2, robot2, robot1},
      .configuration_distance_function = distance_function,
      .edge_step_size = 0.1,
      .env_collision_padding = 0,
      .self_collision_padding = 0});
  EXPECT_THAT(dut->robot_model_instances(), ElementsAre(robot1, robot2));
}

TEST_F(CollisionCheckerThrowTest, InfEnvPad) {
  env_collision_padding_ = std::numeric_limits<double>::infinity();
  ExpectConstructorThrow(".*Env.*isfinite.*");
}

TEST_F(CollisionCheckerThrowTest, InfSelfPad) {
  self_collision_padding_ = std::numeric_limits<double>::infinity();
  ExpectConstructorThrow(".*AllRobotRobot.*isfinite.*");
}

// This tests the APIs that depend on the RobotDiagram in the case where the
// diagram is empty; the only body is the world body. It confirms that the
// exercised APIs return meaningful values without throwing.
//
// This does not exercise the *full* set of APIs; only those that depend on the
// diagram. (See e.g. the EdgeCheckConfiguration test).
GTEST_TEST(CollisionCheckerTest, CollisionCheckerEmpty) {
  RobotDiagramBuilder<double> builder;
  auto diagram = builder.Build();
  const ConfigurationDistanceFunction fn0 = [](const VectorXd&,
                                               const VectorXd&) {
    return 0.0;
  };
  const ModelInstanceIndex robot = default_model_instance();
  const ModelInstanceIndex world = world_model_instance();
  CollisionCheckerTester dut({.model = std::move(diagram),
                              .robot_model_instances = {robot},
                              .configuration_distance_function = fn0,
                              .edge_step_size = 0.1,
                              .env_collision_padding = 0,
                              .self_collision_padding = 0});

  EXPECT_THAT(dut.robot_model_instances(), testing::ElementsAre(robot));
  RigidBody<double> free_body("free", world);
  EXPECT_FALSE(dut.IsPartOfRobot(free_body));
  EXPECT_FALSE(dut.IsPartOfRobot(BodyIndex(0)));

  EXPECT_EQ(dut.GetLargestPadding(), 0.0);
  EXPECT_EQ(dut.GetFilteredCollisionMatrix().size(), 1);
  EXPECT_EQ(dut.GetNominalFilteredCollisionMatrix().size(), 1);

  // Here is an operation that will help check memory safety of callbacks.
  auto op = [](const RobotDiagram<double>&, CollisionCheckerContext* context) {
    ASSERT_NE(context, nullptr);
  };

  // Examples of methods that throw with no contexts allocated. Derived classes
  // are required to allocate contexts during construction. This is proof that
  // such an erroneous implementation can't appear functional.
  EXPECT_THROW(dut.plant_context(), std::exception);
  EXPECT_THROW(dut.plant_context(0), std::exception);
  EXPECT_THROW(dut.model_context(), std::exception);
  EXPECT_THROW(dut.model_context(0), std::exception);
  EXPECT_THROW(dut.Clone(), std::exception);
  EXPECT_THROW(dut.MakeStandaloneModelContext(), std::exception);
  EXPECT_THROW(dut.PerformOperationAgainstAllModelContexts(op), std::exception);

  dut.AllocateContexts();
  EXPECT_NO_THROW(dut.plant_context());
  EXPECT_NO_THROW(dut.plant_context(0));
  EXPECT_NO_THROW(dut.model_context());
  EXPECT_NO_THROW(dut.model_context(0));
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_NO_THROW(dut.MakeStandaloneModelContext());
  EXPECT_NO_THROW(dut.PerformOperationAgainstAllModelContexts(op));

  // Asking for an out-of-range context number throws.
  EXPECT_FALSE(dut.SupportsParallelChecking());
  EXPECT_THROW(dut.plant_context(1), std::exception);
  EXPECT_THROW(dut.model_context(1), std::exception);
}

// Test the robot model introspection APIs.
GTEST_TEST(CollisionCheckerTest, ModelIntrospection) {
  auto [robot, robot_index] = MakeModel();
  RobotDiagram<double>* robot_raw = robot.get();
  std::unique_ptr<CollisionCheckerTester> checker =
      MakeUnallocatedChecker(std::move(robot), {robot_index});

  EXPECT_EQ(&checker->model(), robot_raw);
  EXPECT_EQ(&checker->plant(), &robot_raw->plant());

  const auto& b1 = checker->get_body(BodyIndex(1));
  EXPECT_TRUE(checker->IsPartOfRobot(BodyIndex(1)));
  EXPECT_TRUE(checker->IsPartOfRobot(b1));

  EXPECT_THAT(checker->robot_model_instances(),
              testing::ElementsAre(robot_index));

  const auto q = checker->GetZeroConfiguration();
  EXPECT_EQ(q.size(), checker->plant().num_positions());
  EXPECT_EQ(q.norm(), 0.0);
}

// The "setup" model only exists before allocation and goes away after
// allocation.
GTEST_TEST(CollisionCheckerTest, MutableSetupModel) {
  auto [robot, robot_index] = MakeModel();
  std::unique_ptr<CollisionCheckerTester> checker =
      MakeUnallocatedChecker(std::move(robot), {robot_index});

  EXPECT_NO_THROW(checker->GetMutableSetupModel());
  checker->AllocateContexts();  // Required explicit allocation step.
  EXPECT_THROW(checker->GetMutableSetupModel(), std::exception);
}

// CollisionChecker has several responsibilities when calling
// CalcRobotClearance():
//
//  - influence distance must be finite and positive
//  - non-robot dofs must be zeroed out in the Jacobian.
//  - confirm input only has finite values.
//
// Calling MaxNumDistance() simply defaults to the NVI implementation; we'll
// confirm we get back the magic number: 13.
GTEST_TEST(CollisionCheckerTest, RobotClearance) {
  // These tests only call the *implicit* method; they assume that the implicit
  // method always simply calls the explicit method, so that we're testing
  // both in one fell swoop.
  auto [robot, robot_index] =
      MakeModel({.weld_robot = false, .on_env_base = true});
  std::unique_ptr<CollisionCheckerTester> checker =
      MakeUnallocatedChecker(std::move(robot), {robot_index});
  checker->AllocateContexts();

  // MaxNumDistances called the implemented NVI.
  EXPECT_EQ(checker->MaxNumDistances(), 13);

  // The actual values in the configuration are ignored.
  const auto q = checker->GetZeroConfiguration();

  const std::set<int> non_robot_dofs = GetFloatingBaseDofs();
  // We have a non-empty set of dof indices that all index into q.
  ASSERT_GT(non_robot_dofs.size(), 0);
  const int q_size = q.size();
  for (int qi : non_robot_dofs) {
    ASSERT_LT(qi, q_size);
  }

  // Confirm zero-ing out non-robot dofs.
  const RobotClearance clearance = checker->CalcRobotClearance(q, 0);
  ASSERT_EQ(clearance.size(), 1);
  for (int i = 0; i < clearance.num_positions(); ++i) {
    if (non_robot_dofs.contains(i)) {
      EXPECT_EQ(clearance.jacobians()(i), 0);
    } else {
      EXPECT_EQ(clearance.jacobians()(i), i + 1);
    }
  }

  // Error conditions.
  EXPECT_THROW(checker->CalcRobotClearance(q, -1), std::exception);
  constexpr double kInf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(checker->CalcRobotClearance(q, kInf), std::exception);
  // We're only testing against NaN as an indication that the upstream is
  // validating at all.
  const VectorXd nan =
      VectorXd::Constant(q.size(), std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(checker->CalcRobotClearance(nan, 0), std::exception);
}

// Testing framework for the collision checker such that the model contains
// both robot and environment bodies.
class TrivialCollisionCheckerTest : public testing::Test {
 public:
  TrivialCollisionCheckerTest() {
    auto [robot, robot_index] = MakeModel();
    dut_ = MakeUnallocatedChecker(std::move(robot), {robot_index});
    dut_->AllocateContexts();
  }

  // Returns the indices of environment bodies in the model.
  vector<BodyIndex> environment_indices() const {
    return {BodyIndex(0), BodyIndex(4), BodyIndex(5)};
  }

  // Creates an filter matrix with *no* filtered collisions for the bodies
  // in the model of the given checker. If checker is null, then the fixture's
  // dut will be used.
  Eigen::MatrixXi MakeNothingFiltered(
      CollisionCheckerTester* checker = nullptr) {
    if (checker == nullptr) {
      checker = dut_.get();
    }
    const int n = checker->plant().num_bodies();
    Eigen::MatrixXi filtered = Eigen::MatrixXi::Zero(n, n);
    for (int i = 0; i < n; ++i) filtered(i, i) = -1;
    const vector<BodyIndex>& environment_bodies = environment_indices();
    // Loop variables below use `int` for Eigen indexing compatibility.
    for (const int i : environment_bodies) {
      for (const int j : environment_bodies) {
        filtered(i, j) = -1;
      }
    }
    return filtered;
  }

 protected:
  std::unique_ptr<CollisionCheckerTester> dut_;
};

TEST_F(TrivialCollisionCheckerTest, ClonesAndContexts) {
  const auto count_contexts = [](CollisionChecker* checker) {
    int count{0};
    auto op = [&count](const RobotDiagram<double>&, CollisionCheckerContext*) {
      ++count;
    };
    checker->PerformOperationAgainstAllModelContexts(op);
    return count;
  };

  EXPECT_EQ(count_contexts(dut_.get()),
            dut_->num_allocated_contexts() + 1 /* prototype context */);

  // Add one standalone context.
  auto context = dut_->MakeStandaloneModelContext();
  EXPECT_EQ(count_contexts(dut_.get()), dut_->num_allocated_contexts() +
                                            1 /* prototype context */ +
                                            1 /* our standalone context */);

  auto cloned = dut_->Clone();
  EXPECT_NE(cloned, nullptr);

  // The clone's allocated context count matches.
  EXPECT_EQ(count_contexts(cloned.get()),
            dut_->num_allocated_contexts() + 1 /* prototype context */ +
                0 /* standalone context has not been carried over. */);

  context.reset();  // Give up our standalone context.
  EXPECT_EQ(count_contexts(dut_.get()),
            dut_->num_allocated_contexts() + 1 /* prototype context */);
}

// We want to confirm that when setting distance/interpolation functions, they
// are validated using *default* configuration and *not* zero configuration.
// So, we'll create functions that have two properties:
//
//   1. They report they've been called.
//   2. They get angry if they're not provided the default configuration as
//      parameter values.
//
TEST_F(TrivialCollisionCheckerTest, NonZeroDefaultConfiguration) {
  // We need a plant that has a non-zero default configuration, so we can't
  // use dut_.
  auto [robot, robot_index] =
      MakeModel({.weld_robot = false, .on_env_base = true});

  // Retrieve the default configuration, and cross-check that it's non-zero.
  const VectorXd default_q =
      robot->plant().GetPositions(*robot->plant().CreateDefaultContext());
  ASSERT_TRUE((default_q.array() != 0).any());

  std::unique_ptr<CollisionChecker> dut =
      MakeUnallocatedChecker(std::move(robot), {robot_index});

  // Create distance function that rejects all configs except the default. This
  // will prove that the dut only probes the default config when validating the
  // distance function during SetConfigurationDistanceFunction.
  bool distance_called = false;
  auto distance = [&distance_called, &default_q](const VectorXd& q0,
                                                 const VectorXd& q1) {
    distance_called = true;
    EXPECT_EQ(q0, default_q);
    EXPECT_EQ(q1, default_q);
    return 0.0;
  };
  EXPECT_NO_THROW(dut->SetConfigurationDistanceFunction(distance));
  EXPECT_TRUE(distance_called);

  // Create interpolation function that rejects all configs except the default.
  // This will prove that the dut only probes the default config when validating
  // the interpolation function during SetConfigurationInterpolationFunction.
  bool interp_called = false;
  auto interp = [&interp_called, &default_q](const VectorXd& q0,
                                             const VectorXd& q1, double) {
    interp_called = true;
    EXPECT_EQ(q0, default_q);
    EXPECT_EQ(q1, default_q);
    return default_q;
  };
  EXPECT_NO_THROW(dut->SetConfigurationInterpolationFunction(interp));
  EXPECT_TRUE(interp_called);
}

TEST_F(TrivialCollisionCheckerTest, Padding) {
  const int nbodies = dut_->plant().num_bodies();
  ASSERT_EQ(nbodies, 6);
  const auto& b2 = dut_->get_body(BodyIndex(2));
  const auto& b3 = dut_->get_body(BodyIndex(3));

  EXPECT_EQ(dut_->MaybeGetUniformRobotEnvironmentPadding(), 0);
  EXPECT_EQ(dut_->MaybeGetUniformRobotRobotPadding(), 0);
  EXPECT_EQ(dut_->GetLargestPadding(), 0);

  EXPECT_NO_THROW(dut_->SetPaddingBetween(BodyIndex(2), BodyIndex(3), -0.1));
  EXPECT_EQ(dut_->GetLargestPadding(), 0);
  EXPECT_EQ(dut_->CriticizePaddingMatrix(), "");
  EXPECT_EQ(dut_->GetPaddingBetween(BodyIndex(2), BodyIndex(3)), -0.1);
  EXPECT_NO_THROW(dut_->SetPaddingBetween(b2, b3, 0.01));
  EXPECT_EQ(dut_->GetLargestPadding(), 0.01);
  EXPECT_EQ(dut_->CriticizePaddingMatrix(), "");
  EXPECT_EQ(dut_->GetPaddingBetween(b2, b3), 0.01);
  EXPECT_EQ(dut_->MaybeGetUniformRobotRobotPadding(), std::nullopt);

  // Padding between an object and itself is always zero.
  EXPECT_EQ(dut_->GetPaddingBetween(b2, b2), 0);
  EXPECT_EQ(dut_->GetPaddingBetween(b3, b3), 0);

  EXPECT_NO_THROW(dut_->SetPaddingBetween(BodyIndex(0), BodyIndex(3), 0.1));
  EXPECT_EQ(dut_->CriticizePaddingMatrix(), "");
  EXPECT_EQ(dut_->MaybeGetUniformRobotEnvironmentPadding(), std::nullopt);

  EXPECT_NO_THROW(
      dut_->SetPaddingMatrix(Eigen::MatrixXd::Zero(nbodies, nbodies)));
  EXPECT_EQ(dut_->GetLargestPadding(), 0);
  EXPECT_EQ(dut_->CriticizePaddingMatrix(), "");
  EXPECT_EQ(dut_->MaybeGetUniformRobotEnvironmentPadding(), 0);
  EXPECT_EQ(dut_->MaybeGetUniformRobotRobotPadding(), 0);

  // Confirm negative values come through.
  EXPECT_NO_THROW(dut_->SetPaddingAllRobotEnvironmentPairs(-1));
  EXPECT_NO_THROW(dut_->SetPaddingAllRobotRobotPairs(-2));
  EXPECT_EQ(dut_->GetLargestPadding(), -1);
  EXPECT_EQ(dut_->CriticizePaddingMatrix(), "");
  EXPECT_EQ(dut_->MaybeGetUniformRobotEnvironmentPadding(), -1);
  EXPECT_EQ(dut_->MaybeGetUniformRobotRobotPadding(), -2);

  // Reset.
  EXPECT_NO_THROW(
      dut_->SetPaddingMatrix(Eigen::MatrixXd::Zero(nbodies, nbodies)));

  EXPECT_NO_THROW(
      dut_->SetPaddingOneRobotBodyAllEnvironmentPairs(BodyIndex(3), 0.01));
  EXPECT_EQ(dut_->GetLargestPadding(), 0.01);
  EXPECT_EQ(dut_->CriticizePaddingMatrix(), "");
  EXPECT_EQ(dut_->MaybeGetUniformRobotEnvironmentPadding(), std::nullopt);
  EXPECT_EQ(dut_->MaybeGetUniformRobotRobotPadding(), 0);

  EXPECT_NO_THROW(dut_->SetPaddingAllRobotEnvironmentPairs(0.02));
  EXPECT_EQ(dut_->GetLargestPadding(), 0.02);
  EXPECT_EQ(dut_->CriticizePaddingMatrix(), "");
  EXPECT_EQ(dut_->MaybeGetUniformRobotEnvironmentPadding(), 0.02);
  EXPECT_EQ(dut_->MaybeGetUniformRobotRobotPadding(), 0);

  EXPECT_NO_THROW(dut_->SetPaddingAllRobotRobotPairs(0.03));
  EXPECT_EQ(dut_->GetLargestPadding(), 0.03);
  EXPECT_EQ(dut_->CriticizePaddingMatrix(), "");
  EXPECT_EQ(dut_->MaybeGetUniformRobotEnvironmentPadding(), 0.02);
  EXPECT_EQ(dut_->MaybeGetUniformRobotRobotPadding(), 0.03);
}

// Tests the ValidatePaddingMatrix() method by exercising SetPaddingMatrix() and
// passing otherwise invalid padding specifications.
TEST_F(TrivialCollisionCheckerTest, SetPaddingMatrix) {
  const int num_bodies = dut_->plant().num_bodies();
  // Bodies 1, 2, 3 are robot bodies and bodies 4 and 5 are environment (and
  // body zero is the world).
  ASSERT_EQ(num_bodies, 6);

  // Make a padding matrix with some non-zero padding. Confirm it constitutes a
  // change from the original state -- i.e., we can set the matrix with a valid
  // matrix.
  EXPECT_EQ(dut_->MaybeGetUniformRobotEnvironmentPadding(), 0.0);
  EXPECT_EQ(dut_->MaybeGetUniformRobotRobotPadding(), 0.0);
  Eigen::MatrixXd padding = Eigen::MatrixXd::Zero(num_bodies, num_bodies);
  padding(1, 2) = -1;
  padding(2, 1) = -1;
  padding(1, 4) = 2;
  padding(4, 1) = 2;
  ASSERT_NO_THROW(dut_->SetPaddingMatrix(padding));
  // Padding values are no longer uniform.
  EXPECT_FALSE(dut_->MaybeGetUniformRobotEnvironmentPadding().has_value());
  EXPECT_FALSE(dut_->MaybeGetUniformRobotRobotPadding().has_value());

  // Now test the various ways we can throw.

  {
    // The padding matrix is the wrong size.
    Eigen::MatrixXd wrong_size =
        Eigen::MatrixXd::Zero(num_bodies + 1, num_bodies + 1);

    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetPaddingMatrix(wrong_size),
                                ".*6x6.*wrong size: 7x7.*");
    Eigen::MatrixXd too_tall =
        Eigen::MatrixXd::Zero(num_bodies + 1, num_bodies);

    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetPaddingMatrix(too_tall),
                                ".*6x6.*wrong size: 7x6.*");
    Eigen::MatrixXd too_wide =
        Eigen::MatrixXd::Zero(num_bodies, num_bodies + 1);

    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetPaddingMatrix(too_wide),
                                ".*6x6.*wrong size: 6x7.*");
  }

  {
    // Diagonal is not 0.
    for (int i = 0; i < num_bodies; ++i) {
      Eigen::MatrixXd bad_padding(padding);
      bad_padding(i, i) = 10.0;
      DRAKE_EXPECT_THROWS_MESSAGE(
          dut_->SetPaddingMatrix(bad_padding),
          ".*invalid values on the diagonal.+ must always be 0.*");
    }
  }

  {
    // Environment-environment pairs aren't 0.
    const vector<pair<int, int>> environment_body_pairs{{0, 4}, {0, 5}, {4, 5}};
    for (const auto& [i, j] : environment_body_pairs) {
      Eigen::MatrixXd bad_padding(padding);
      bad_padding(i, j) = 22.0;
      bad_padding(j, i) = 22.0;
      DRAKE_EXPECT_THROWS_MESSAGE(
          dut_->SetPaddingMatrix(bad_padding),
          ".* must contain 0 for pairs of environment bodies.*");
    }
  }

  {
    // Matrix is not symmetric, tested for an arbitrary pair.
    Eigen::MatrixXd bad_padding(padding);
    bad_padding(0, 3) = 77.0;
    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetPaddingMatrix(bad_padding),
                                ".* must be symmetric.*");
  }

  {
    // Matrix is not symmetric, tested for an arbitrary pair, with sneaky NaNs.
    // Try NaN in both upper and lower triangles.
    Eigen::MatrixXd bad_padding(padding);
    // NaN is in the upper triangle. The implementation catches the non-finite
    // value.
    bad_padding(0, 3) = std::numeric_limits<double>::quiet_NaN();
    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetPaddingMatrix(bad_padding),
                                ".* must contain finite values.*");

    // NaN is in the lower triangle. The implementation catches the
    // asymmetry.
    bad_padding(0, 3) = 0.0;
    bad_padding(3, 0) = std::numeric_limits<double>::quiet_NaN();
    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetPaddingMatrix(bad_padding),
                                ".* must be symmetric.*");
  }

  {
    // Matrix is not finite, tested for an arbitrary pair.
    Eigen::MatrixXd bad_padding(padding);
    bad_padding(0, 3) = std::numeric_limits<double>::infinity();
    bad_padding(3, 0) = std::numeric_limits<double>::infinity();
    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetPaddingMatrix(bad_padding),
                                ".* must contain finite values.*");

    bad_padding(0, 3) = std::numeric_limits<double>::quiet_NaN();
    bad_padding(3, 0) = std::numeric_limits<double>::quiet_NaN();
    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetPaddingMatrix(bad_padding),
                                ".* must contain finite values.*");
  }
}

// The AddCollisionShape APIs ultimately depend on the virtual
// AddCollisionShapeToBody() method. For the CollisionChecker base class test,
// we are simply confirming that the return value defined by the implementation
// is properly propagated all the way up.
TEST_F(TrivialCollisionCheckerTest, Shapes) {
  // We use a sphere as being representative of Shape. Nothing here depends on
  // it being a sphere.
  const Sphere sphere(0.1);
  const BodyShapeDescription body_shape{sphere, {}, "m3", "b0"};
  const auto& b1 = dut_->get_body(BodyIndex(1));
  for (bool can_add : {false, true}) {
    SCOPED_TRACE(fmt::format("Can add shapes: {}", can_add));
    dut_->SetCanAddCollisionShapes(can_add);

    EXPECT_EQ(dut_->AddCollisionShape("test", body_shape), can_add);
    EXPECT_EQ(dut_->AddCollisionShapes("test", {body_shape}), can_add);

    auto added_shapes = dut_->AddCollisionShapes(
        {{"test", {body_shape}}, {"test2", {body_shape}}});
    const int expected_count = can_add ? 1 : 0;
    EXPECT_EQ(added_shapes,
              (std::map<std::string, int>(
                  {{"test", expected_count}, {"test2", expected_count}})));

    EXPECT_EQ(
        dut_->AddCollisionShapeToFrame("test3", b1.body_frame(), sphere, {}),
        can_add);
    EXPECT_EQ(dut_->AddCollisionShapeToBody("test3", b1, sphere, {}), can_add);

    const std::map<std::string, std::vector<BodyShapeDescription>> all_shapes =
        dut_->GetAllAddedCollisionShapes();
    if (can_add) {
      // Three groups with a number of shapes added to each group.
      EXPECT_EQ(all_shapes.size(), 3);
      EXPECT_EQ(all_shapes.at("test").size(), 3);
      EXPECT_EQ(all_shapes.at("test2").size(), 1);
      EXPECT_EQ(all_shapes.at("test3").size(), 2);

      // Remove a single group by name (with multiple shapes).
      EXPECT_NO_THROW(dut_->RemoveAllAddedCollisionShapes("test"));
      EXPECT_EQ(dut_->GetAllAddedCollisionShapes().size(), 2);
      EXPECT_EQ(all_shapes.at("test2").size(), 1);
      EXPECT_EQ(all_shapes.at("test3").size(), 2);

      // Remove remaining groups entirely.
      EXPECT_NO_THROW(dut_->RemoveAllAddedCollisionShapes());
      EXPECT_EQ(dut_->GetAllAddedCollisionShapes().size(), 0);
    } else {
      EXPECT_EQ(all_shapes.size(), 0);
    }
  }
}

TEST_F(TrivialCollisionCheckerTest, ReportParallelChecking) {
  for (bool parallel_supported : {true, false}) {
    auto [robot, robot_index] = MakeModel();
    auto checker = MakeUnallocatedChecker(std::move(robot), {robot_index},
                                          parallel_supported);
    EXPECT_EQ(checker->SupportsParallelChecking(), parallel_supported);
  }
}

// For Update*Positions, we need to test the following:
//   1 Do we operate on the right context?
//   2 Does the context get updated?
//   3 Does the context get updated before DoUpdateContextPositions gets called?
//   4 Does DoUpdateContextPositions get called with a non-null context?
//   5 Do we check the inputs for non-finite values?
TEST_F(TrivialCollisionCheckerTest, UpdatePositions) {
  VectorXd q = dut_->GetZeroConfiguration();

  q[0] = 0.1;
  const Context<double>& model_plant_context = dut_->UpdatePositions(q);
  // Test for #3 and #4; it doesn't throw and the right positions were recorded.
  EXPECT_EQ(dut_->latest_positions()[0], 0.1);

  // We're only testing against NaN as an indication that the upstream is
  // validating at all.
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(dut_->UpdatePositions(q), std::exception);

  q[0] = 0.2;
  std::shared_ptr<CollisionCheckerContext> collision_context =
      dut_->MakeStandaloneModelContext();
  const Context<double>& standalone_plant_context =
      dut_->UpdateContextPositions(collision_context.get(), q);
  // Test for #3 and #4; it doesn't throw and the right positions were recorded.
  EXPECT_EQ(dut_->latest_positions()[0], 0.2);

  // Test for #1 and #2; each context is observably updated as expected.
  EXPECT_EQ(dut_->plant().GetPositions(model_plant_context)[0], 0.1);
  EXPECT_EQ(dut_->plant().GetPositions(standalone_plant_context)[0], 0.2);

  // We're only testing against NaN as an indication that the upstream is
  // validating at all.
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(dut_->UpdateContextPositions(collision_context.get(), q),
               std::exception);
}

// Verify that positions get updated before the NVI method
// DoClassifyContextBodyCollisions gets called.
TEST_F(TrivialCollisionCheckerTest, ClassifyBodyCollisions) {
  VectorXd q = dut_->GetZeroConfiguration();

  q[0] = 0.1;
  dut_->ClassifyBodyCollisions(q);
  // Update got called.
  EXPECT_EQ(dut_->latest_positions()[0], 0.1);
  // Update got called before forwarding to a derived class.
  EXPECT_EQ(dut_->positions_for_classify()[0], 0.1);

  // We're only testing against NaN as an indication that the upstream is
  // validating at all.
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(dut_->ClassifyBodyCollisions(q), std::exception);

  q[0] = 0.2;
  std::shared_ptr<CollisionCheckerContext> collision_context =
      dut_->MakeStandaloneModelContext();
  dut_->ClassifyContextBodyCollisions(collision_context.get(), q);
  // Update got called.
  EXPECT_EQ(dut_->latest_positions()[0], 0.2);
  // Update got called before forwarding to a derived class.
  EXPECT_EQ(dut_->positions_for_classify()[0], 0.2);

  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(dut_->ClassifyContextBodyCollisions(collision_context.get(), q),
               std::exception);
}

// Verify that positions get checked for finite values and updated before the
/// NVI method DoCheckContextConfigCollisionFree gets called.
TEST_F(TrivialCollisionCheckerTest, CheckConfigCollisionFree) {
  VectorXd q = dut_->GetZeroConfiguration();

  q[0] = 0.1;
  dut_->CheckConfigCollisionFree(q);
  // Update got called.
  EXPECT_EQ(dut_->latest_positions()[0], 0.1);
  // Update got called before forwarding to a derived class.
  EXPECT_EQ(dut_->positions_for_check()[0], 0.1);

  // We're only testing against NaN as an indication that the upstream is
  // validating at all.
  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(dut_->CheckConfigCollisionFree(q), std::exception);

  q[0] = 0.2;
  std::shared_ptr<CollisionCheckerContext> collision_context =
      dut_->MakeStandaloneModelContext();
  dut_->CheckContextConfigCollisionFree(collision_context.get(), q);
  // Update got called.
  EXPECT_EQ(dut_->latest_positions()[0], 0.2);
  // Update got called before forwarding to a derived class.
  EXPECT_EQ(dut_->positions_for_check()[0], 0.2);

  q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      dut_->CheckContextConfigCollisionFree(collision_context.get(), q),
      std::exception);

  EXPECT_THROW(dut_->CheckConfigsCollisionFree({q, q}), std::exception);
}

// Tests the ValidateFilteredCollisionMatrix() method by exercising
// SetCollisionFilterMatrix() and passing otherwise invalid filtered
// specifications. Other APIs that attempt to manipulate the matrix are covered
// in their own tests (including their failure conditions).
TEST_F(TrivialCollisionCheckerTest, SetCollisionFilterMatrix) {
  const int num_bodies = dut_->plant().num_bodies();
  // Bodies 1, 2, 3 are robot bodies and bodies 4 and 5 are environment (and
  // body zero is the world).
  ASSERT_EQ(num_bodies, 6);

  // Make a baseline filter matrix with no filtered collisions for the model.
  // Confirm it constitutes a change from the original state -- i.e., we can
  // set the matrix with a valid matrix.
  const Eigen::MatrixXi no_filters = MakeNothingFiltered();
  EXPECT_FALSE(CompareMatrices(dut_->GetFilteredCollisionMatrix(), no_filters));
  ASSERT_NO_THROW(dut_->SetCollisionFilterMatrix(no_filters));
  EXPECT_TRUE(CompareMatrices(dut_->GetFilteredCollisionMatrix(), no_filters));
  // Confirm that the collision filter matrix was updated before calling
  // UpdateCollisionFilters().
  EXPECT_TRUE(CompareMatrices(dut_->updated_filter_matrix(),
                              dut_->GetFilteredCollisionMatrix()));

  // Now test the various ways we can throw.

  {
    // The filter matrix is the wrong size.
    Eigen::MatrixXi wrong_size =
        Eigen::MatrixXi::Zero(num_bodies + 1, num_bodies + 1);

    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetCollisionFilterMatrix(wrong_size),
                                ".*wrong size.*");
    Eigen::MatrixXi not_square =
        Eigen::MatrixXi::Zero(num_bodies + 1, num_bodies);

    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetCollisionFilterMatrix(not_square),
                                ".*wrong size.*");
  }

  {
    // Diagonal is not -1.
    for (int i = 0; i < num_bodies; ++i) {
      Eigen::MatrixXi bad_filters(no_filters);
      bad_filters(i, i) = 0;
      DRAKE_EXPECT_THROWS_MESSAGE(
          dut_->SetCollisionFilterMatrix(bad_filters),
          ".*invalid values on the diagonal.+ must always be -1.*");
      bad_filters(i, i) = 1;
      DRAKE_EXPECT_THROWS_MESSAGE(
          dut_->SetCollisionFilterMatrix(bad_filters),
          ".*invalid values on the diagonal.+ must always be -1.*");
    }
  }

  {
    // Environment-environment pairs aren't -1.
    const vector<pair<int, int>> environment_body_pairs{{0, 4}, {0, 5}, {4, 5}};
    for (const auto& [i, j] : environment_body_pairs) {
      Eigen::MatrixXi bad_filters(no_filters);
      bad_filters(i, j) = 0;
      bad_filters(j, i) = 0;
      DRAKE_EXPECT_THROWS_MESSAGE(
          dut_->SetCollisionFilterMatrix(bad_filters),
          ".* must contain -1 for pairs of environment bodies.*");
      bad_filters(i, j) = 1;
      bad_filters(j, i) = 1;
      DRAKE_EXPECT_THROWS_MESSAGE(
          dut_->SetCollisionFilterMatrix(bad_filters),
          ".* must contain -1 for pairs of environment bodies.*");
    }
  }

  {
    // Values not included in {-1, 0, 1}, tested for an arbitrary
    // robot-environment pair.
    Eigen::MatrixXi bad_filters(no_filters);
    bad_filters(0, 3) = 2;
    bad_filters(3, 0) = 2;
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut_->SetCollisionFilterMatrix(bad_filters),
        ".* must contain values that are 0, 1, or -1.*");
    bad_filters(0, 3) = -2;
    bad_filters(3, 0) = -2;
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut_->SetCollisionFilterMatrix(bad_filters),
        ".* must contain values that are 0, 1, or -1.*");
  }

  {
    // Matrix is not symmetric, tested for an arbitrary pair.
    Eigen::MatrixXi bad_filters(no_filters);
    bad_filters(0, 3) = 1;
    DRAKE_EXPECT_THROWS_MESSAGE(dut_->SetCollisionFilterMatrix(bad_filters),
                                ".* must be symmetric.*");
  }

  {
    // Pairs with bodies marked as -1.
    Eigen::MatrixXi bad_filters(no_filters);
    bad_filters(0, 3) = -1;
    bad_filters(3, 0) = -1;
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut_->SetCollisionFilterMatrix(bad_filters),
        ".* can only be 1 or 0 for a pair with a robot body.*");
  }
}

// Confirms construction of the nominal collision filter matrix. Explores the
// various properties based on the initial configuration of the underlying
// model. Finally, uses SetCollisionFilterMatrix() on the resulting filter
// matrix to assert its validity.
//
// We'll be creating a RobotDiagram over and over to control its underlying
// properties to achieve specific preconditions when creating the checker.
TEST_F(TrivialCollisionCheckerTest, CollisionFiltersNominal) {
  {
    // Initial collision filters are equal to nominal.
    EXPECT_TRUE(CompareMatrices(dut_->GetNominalFilteredCollisionMatrix(),
                                dut_->GetFilteredCollisionMatrix()));
  }

  {
    // A change to the checker's filter configuration doesn't change the
    // nominal matrix.
    auto [robot, robot_index] = MakeModel();
    auto checker = MakeUnallocatedChecker(std::move(robot), {robot_index});
    const Eigen::MatrixXi first_nominal =
        checker->GetNominalFilteredCollisionMatrix();
    const Eigen::MatrixXi no_filters = MakeNothingFiltered(checker.get());
    // The nominal filters don't match what we're going to set.
    EXPECT_FALSE(CompareMatrices(first_nominal, no_filters));

    EXPECT_NO_THROW(checker->SetCollisionFilterMatrix(no_filters));

    // Filter matrix has updated, but the reported nominal matrix hasn't
    // changed.
    EXPECT_TRUE(
        CompareMatrices(checker->GetFilteredCollisionMatrix(), no_filters));
    EXPECT_TRUE(CompareMatrices(first_nominal,
                                checker->GetNominalFilteredCollisionMatrix()));
  }

  {
    // Underlying model has *zero* collision filters and nothing welded; should
    // be all -1s and 0s.
    auto [robot, robot_index] = MakeModel({.weld_robot = false});
    auto& scene_graph = robot->mutable_scene_graph();
    auto collision_filters = scene_graph.collision_filter_manager();
    collision_filters.Apply(CollisionFilterDeclaration().AllowWithin(
        GeometrySet(scene_graph.model_inspector().GetAllFrameIds())));
    auto checker = MakeUnallocatedChecker(std::move(robot), {robot_index});
    const Eigen::MatrixXi filters = checker->GetFilteredCollisionMatrix();
    // Confirm it is valid.
    EXPECT_NO_THROW(checker->SetCollisionFilterMatrix(filters));
    // Confirm there are no `1` values.
    const Eigen::MatrixXi filters_expected = MakeNothingFiltered(checker.get());
    EXPECT_TRUE(CompareMatrices(filters, filters_expected));
  }

  {
    // Underlying model has *zero* collision filters but robot body (1) is
    // welded to the world (0); should be all -1s and 0s, except at (0,1) and
    // (1, 0).
    auto [robot, robot_index] = MakeModel();
    auto& scene_graph = robot->mutable_scene_graph();
    auto collision_filters = scene_graph.collision_filter_manager();
    collision_filters.Apply(CollisionFilterDeclaration().AllowWithin(
        GeometrySet(scene_graph.model_inspector().GetAllFrameIds())));
    auto checker = MakeUnallocatedChecker(std::move(robot), {robot_index});
    const Eigen::MatrixXi filters = checker->GetFilteredCollisionMatrix();
    // Confirm it is valid.
    EXPECT_NO_THROW(checker->SetCollisionFilterMatrix(filters));
    // The only 1s are between the world and the welded robot root.
    Eigen::MatrixXi filters_expected = MakeNothingFiltered(checker.get());
    filters_expected(0, 1) = filters_expected(1, 0) = 1;
    EXPECT_TRUE(CompareMatrices(filters, filters_expected));
  }

  {
    // Underlying model has collision filters added, but nothing welded;
    // should be -1s and 0s, except for where the collision filter has been
    // specified.
    auto [robot, robot_index] = MakeModel({.weld_robot = false});
    auto& scene_graph = robot->mutable_scene_graph();
    auto collision_filters = scene_graph.collision_filter_manager();
    // Clear all collision filters and then add one back between robot bodies
    // 1 & 3.
    collision_filters.Apply(CollisionFilterDeclaration().AllowWithin(
        GeometrySet(scene_graph.model_inspector().GetAllFrameIds())));
    const FrameId id1 = robot->plant().GetBodyFrameIdOrThrow(BodyIndex(1));
    const FrameId id3 = robot->plant().GetBodyFrameIdOrThrow(BodyIndex(3));
    collision_filters.Apply(
        CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id1, id3})));
    auto checker = MakeUnallocatedChecker(std::move(robot), {robot_index});
    const Eigen::MatrixXi filters = checker->GetFilteredCollisionMatrix();
    // Confirm it is valid.
    EXPECT_NO_THROW(checker->SetCollisionFilterMatrix(filters));
    // The only 1s are between bodies 1 and 3.
    Eigen::MatrixXi filters_expected = MakeNothingFiltered(checker.get());
    filters_expected(3, 1) = filters_expected(1, 3) = 1;
    EXPECT_TRUE(CompareMatrices(filters, filters_expected));
  }

  {
    // SceneGraph can have multiple geometries on a single body that have
    // different filter status w.r.t. geometries on another body.
    // CollisionChecker will get angry about that.
    auto [robot, robot_index] = MakeModel({.per_body_geometries = 2});
    auto& scene_graph = robot->mutable_scene_graph();
    auto collision_filters = scene_graph.collision_filter_manager();
    // Clear all collision filters and then add one back between two
    // geometries belonging to robot bodies 1 & 3.
    collision_filters.Apply(CollisionFilterDeclaration().AllowWithin(
        GeometrySet(scene_graph.model_inspector().GetAllFrameIds())));
    const FrameId f_id1 = robot->plant().GetBodyFrameIdOrThrow(BodyIndex(1));
    const GeometryId g_id1 =
        scene_graph.model_inspector().GetGeometries(f_id1, Role::kProximity)[0];
    const FrameId f_id3 = robot->plant().GetBodyFrameIdOrThrow(BodyIndex(3));
    const GeometryId g_id3 =
        scene_graph.model_inspector().GetGeometries(f_id3, Role::kProximity)[0];
    collision_filters.Apply(CollisionFilterDeclaration().ExcludeWithin(
        GeometrySet({g_id1, g_id3})));

    DRAKE_EXPECT_THROWS_MESSAGE(
        MakeUnallocatedChecker(std::move(robot), {robot_index}),
        ".*SceneGraph's collision filters .+ are not homogeneous.*");
  }
}

// Tests SetCollisionFilteredBetween(). This function doesn't call validate
// because its correctness doesn't depend on input parameters. Instead,
// we test its validity here by confirming the filter matrix produced
// passes the validity test exercised by SetCollisionFilterMatrix().
TEST_F(TrivialCollisionCheckerTest, SetCollisionFilteredBetween) {
  const int num_bodies = dut_->plant().num_bodies();
  // Bodies 1, 2, 3 are robot bodies and bodies 4 and 5 are environment (and
  // body zero is the world).
  ASSERT_EQ(num_bodies, 6);

  const vector<BodyIndex> b{BodyIndex(0), BodyIndex(1), BodyIndex(2),
                            BodyIndex(3), BodyIndex(4), BodyIndex(5)};

  // There are two overrides: one takes bodies, one takes body indices. We'll
  // test the body-index-based override, relying on the fact that the body-based
  // override delegates to it.

  {
    // Error conditions.
    // Both bodies are environment bodies.
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut_->SetCollisionFilteredBetween(b[0], b[4], true),
        ".* cannot be used on pairs of environment bodies.*");
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut_->SetCollisionFilteredBetween(b[0], b[4], false),
        ".* cannot be used on pairs of environment bodies.*");

    // Out of range indices.
    EXPECT_THROW(dut_->SetCollisionFilteredBetween(b[0], BodyIndex(10), true),
                 std::exception);
    EXPECT_THROW(dut_->SetCollisionFilteredBetween(BodyIndex(10), b[0], true),
                 std::exception);

    // Trying to set filter on a robot body with itself.
    EXPECT_THROW(dut_->SetCollisionFilteredBetween(b[1], b[1], true),
                 std::exception);
  }

  {
    // Setting filters between robot-environment, in either order is valid.
    EXPECT_NO_THROW(dut_->SetCollisionFilteredBetween(b[1], b[4], true));
    EXPECT_TRUE(dut_->IsCollisionFilteredBetween(b[1], b[4]));
    EXPECT_NO_THROW(
        dut_->SetCollisionFilterMatrix(dut_->GetFilteredCollisionMatrix()));

    // Clearing filters is likewise valid - we'll reverse the order of indices
    // to show it doesn't matter.
    EXPECT_NO_THROW(dut_->SetCollisionFilteredBetween(b[4], b[1], false));
    EXPECT_FALSE(dut_->IsCollisionFilteredBetween(b[1], b[4]));
    EXPECT_NO_THROW(
        dut_->SetCollisionFilterMatrix(dut_->GetFilteredCollisionMatrix()));
    // Confirm that the collision filter matrix was updated before calling
    // UpdateCollisionFilters().
    EXPECT_TRUE(CompareMatrices(dut_->updated_filter_matrix(),
                                dut_->GetFilteredCollisionMatrix()));
  }

  {
    // Setting filters between robot-robot, in either order, is valid.
    EXPECT_NO_THROW(dut_->SetCollisionFilteredBetween(b[1], b[2], true));
    EXPECT_TRUE(dut_->IsCollisionFilteredBetween(b[1], b[2]));
    EXPECT_NO_THROW(
        dut_->SetCollisionFilterMatrix(dut_->GetFilteredCollisionMatrix()));

    // Clearing filters is likewise valid - we'll reverse the order of indices
    // to show it doesn't matter.
    EXPECT_NO_THROW(dut_->SetCollisionFilteredBetween(b[2], b[1], false));
    EXPECT_FALSE(dut_->IsCollisionFilteredBetween(b[1], b[2]));
    EXPECT_NO_THROW(
        dut_->SetCollisionFilterMatrix(dut_->GetFilteredCollisionMatrix()));
  }

  // Token test against bodies, instead of indices.
  EXPECT_NO_THROW(dut_->SetCollisionFilteredBetween(
      dut_->get_body(b[2]), dut_->get_body(b[3]), true));
  EXPECT_TRUE(dut_->IsCollisionFilteredBetween(b[2], b[3]));
  EXPECT_NO_THROW(
      dut_->SetCollisionFilterMatrix(dut_->GetFilteredCollisionMatrix()));
}

// Tests SetCollisionFilteredWithAllBodies(). This function doesn't call
// validate because its correctness doesn't depend on input parameters. Instead,
// we test its validity here by confirming the filter matrix produced
// passes the validity test exercised by SetCollisionFilterMatrix().
TEST_F(TrivialCollisionCheckerTest, SetCollisionFilteredWithAllBodies) {
  const int num_bodies = dut_->plant().num_bodies();
  // Bodies 1, 2, 3 are robot bodies and bodies 4 and 5 are environment (and
  // body zero is the world).
  ASSERT_EQ(num_bodies, 6);

  const vector<BodyIndex> b{BodyIndex(0), BodyIndex(1), BodyIndex(2),
                            BodyIndex(3), BodyIndex(4), BodyIndex(5)};

  {
    // Setting an environment body is simply bad.
    EXPECT_THROW(dut_->SetCollisionFilteredWithAllBodies(b[4]), std::exception);
  }

  {
    // Setting a robot body (via index) doesn't throw, produces a consistent
    // matrix and reports filters as expected.
    EXPECT_NO_THROW(dut_->SetCollisionFilteredWithAllBodies(b[3]));
    // Confirm that the collision filter matrix was updated before calling
    // UpdateCollisionFilters().
    EXPECT_TRUE(CompareMatrices(dut_->updated_filter_matrix(),
                                dut_->GetFilteredCollisionMatrix()));
    EXPECT_NO_THROW(
        dut_->SetCollisionFilterMatrix(dut_->GetFilteredCollisionMatrix()));
    for (BodyIndex i(0); i < num_bodies; ++i) {
      EXPECT_TRUE(dut_->IsCollisionFilteredBetween(i, b[3]));
    }
  }

  {
    // Setting a robot body (via reference) doesn't throw, produces a consistent
    // matrix and reports filters as expected.
    EXPECT_NO_THROW(
        dut_->SetCollisionFilteredWithAllBodies(dut_->get_body(b[2])));
    // Confirm that the collision filter matrix was updated before calling
    // UpdateCollisionFilters().
    EXPECT_TRUE(CompareMatrices(dut_->updated_filter_matrix(),
                                dut_->GetFilteredCollisionMatrix()));
    EXPECT_NO_THROW(
        dut_->SetCollisionFilterMatrix(dut_->GetFilteredCollisionMatrix()));
    for (BodyIndex i(0); i < num_bodies; ++i) {
      EXPECT_TRUE(dut_->IsCollisionFilteredBetween(i, b[2]));
    }
  }
}

// Simple tests against the IsCollisionFilteredBetween() API. It assumes the
// collision filter matrix is consistent and known and simply confirms that
// it is interpreted correctly and doesn't depend on the ordering of the
// parameter values (matrix is symmetric).
TEST_F(TrivialCollisionCheckerTest, IsCollisionFilteredBetween) {
  const int num_bodies = dut_->plant().num_bodies();
  // Bodies 1, 2, 3 are robot bodies and bodies 4 and 5 are environment (and
  // body zero is the world).
  ASSERT_EQ(num_bodies, 6);

  // Make a baseline filter matrix with no filtered collisions for the model.
  Eigen::MatrixXi filters = MakeNothingFiltered();

  // Now filter collisions between one robot-robot pair and one
  // robot-environment pair.
  filters(1, 3) = filters(3, 1) = 1;
  filters(2, 4) = filters(4, 2) = 1;

  EXPECT_NO_THROW(dut_->SetCollisionFilterMatrix(filters));

  // Sample *unfiltered* robot-robot pair.
  EXPECT_FALSE(dut_->IsCollisionFilteredBetween(BodyIndex(1), BodyIndex(2)));
  EXPECT_FALSE(dut_->IsCollisionFilteredBetween(BodyIndex(2), BodyIndex(1)));

  // Sample *filtered* robot-robot pair.
  EXPECT_TRUE(dut_->IsCollisionFilteredBetween(BodyIndex(1), BodyIndex(3)));
  EXPECT_TRUE(dut_->IsCollisionFilteredBetween(BodyIndex(3), BodyIndex(1)));

  // Sample *unfiltered* robot-environment pair.
  EXPECT_FALSE(dut_->IsCollisionFilteredBetween(BodyIndex(0), BodyIndex(1)));
  EXPECT_FALSE(dut_->IsCollisionFilteredBetween(BodyIndex(1), BodyIndex(0)));

  // Sample *filtered* robot-environment pair.
  EXPECT_TRUE(dut_->IsCollisionFilteredBetween(BodyIndex(2), BodyIndex(4)));
  EXPECT_TRUE(dut_->IsCollisionFilteredBetween(BodyIndex(4), BodyIndex(2)));

  // Sample *filtered* environment-environment pair.
  EXPECT_TRUE(dut_->IsCollisionFilteredBetween(BodyIndex(0), BodyIndex(4)));
  EXPECT_TRUE(dut_->IsCollisionFilteredBetween(BodyIndex(4), BodyIndex(0)));

  // Token invocation of the same API using bodies instead of body indices. We
  // exploit the fact that the overload that takes bodies delegates to the
  // body index overload tested above.
  EXPECT_TRUE(dut_->IsCollisionFilteredBetween(dut_->get_body(BodyIndex(1)),
                                               dut_->get_body(BodyIndex(3))));
}

// Creates a checker on a plant with an N-link chain (optionally) welded to the
// world. Part of the edge-checking API test infrastructure (see below).
template <typename CheckerType>
CheckerType MakeEdgeChecker(ConfigurationDistanceFunction calc_dist,
                            double step_size = 0.25,
                            ConfigurationInterpolationFunction interp = nullptr,
                            bool welded = true, int N = 2) {
  RobotDiagramBuilder<double> builder;
  // We need just enough state so we can save values in q.
  auto& plant = builder.plant();
  const ModelInstanceIndex robot = AddChain(&plant, N);
  // Weld the chain to the world so we don't have to worry about quaternions.
  const auto& body = plant.GetBodyByName("b0");
  if (welded) {
    plant.WeldFrames(plant.world_frame(), body.body_frame(), {});
  }
  auto diagram = builder.Build();

  CheckerType checker({.model = std::move(diagram),
                       .robot_model_instances = {robot},
                       .configuration_distance_function = calc_dist,
                       .edge_step_size = step_size,
                       .env_collision_padding = 0,
                       .self_collision_padding = 0});
  checker.SetConfigurationInterpolationFunction(interp);
  return checker;
}

// The edge-checking API is configured with a distance function, interpolation
// function, and edge step size. This test simply confirms that those quantities
// get configured as expected upon construction and can be modified after
// construction via their setters.
//
// These configuration values do *not* depend on the underlying robot model or
// checker's contexts; so, we'll use the most bare bones instance possible.
GTEST_TEST(EdgeCheckTest, Configuration) {
  // This distance function is designed to return an arbitrary, recognizable
  // value that clearly signals its invocation: -1.5. However, to be accepted
  // by CollisionChecker dist(q, q) *must* return 0.
  const ConfigurationDistanceFunction dist0 = [](const VectorXd& q1,
                                                 const VectorXd& q2) {
    const double dist = (q1 - q2).norm();
    if (dist == 0) return 0.0;
    return 1.5;
  };
  CollisionCheckerTester dut = MakeEdgeChecker<CollisionCheckerTester>(dist0);
  const int q_size = dut.plant().num_positions();

  // Edge step size: set by constructor, is retrievable and settable.
  // Constructor value came through.
  EXPECT_EQ(dut.edge_step_size(), 0.25);
  // Explicitly setting post-construction likewise works.
  dut.set_edge_step_size(0.1);
  EXPECT_EQ(dut.edge_step_size(), 0.1);

  // Both distance and interpolation functionality provide parallel APIs:
  //   (1) Function to evaluate the function directly.
  //   (2) Function to create a functor that wraps the checker.
  //   (3) Ability to set the functor.
  // We'll use the same pattern for both distance and interpolation functions:
  //   - Confirm post-construction behavior
  //     - Evaluate (1) and (2) above.
  //   - Set the function to something else (3):
  //     - Evaluate (1) and (2) again.
  // We're not testing the underlying distance and interpolation functions; just
  // looking for evidence that they're being called.

  const Eigen::VectorXd q1 = Eigen::VectorXd::Constant(q_size, 1);
  const Eigen::VectorXd q2 = Eigen::VectorXd::Zero(q_size);
  const Eigen::VectorXd nan = Eigen::VectorXd::Constant(
      q_size, std::numeric_limits<double>::quiet_NaN());

  {
    // Distance function.

    // Evaluate (1) and (2) as constructed. dist0 should always return -1.5.
    EXPECT_EQ(dut.ComputeConfigurationDistance(q1, q2), 1.5);
    ASSERT_NE(dut.MakeStandaloneConfigurationDistanceFunction(), nullptr);
    EXPECT_EQ(dut.MakeStandaloneConfigurationDistanceFunction()(q1, q2), 1.5);

    // Change the function via (3).
    const ConfigurationDistanceFunction dist1 = [](const VectorXd& a,
                                                   const VectorXd& b) {
      return (b - a).norm();
    };
    dut.SetConfigurationDistanceFunction(dist1);

    // Evaluate (1) and (2) again.
    EXPECT_EQ(dut.ComputeConfigurationDistance(q1, q2), q1.norm());
    ASSERT_NE(dut.MakeStandaloneConfigurationDistanceFunction(), nullptr);
    EXPECT_EQ(dut.MakeStandaloneConfigurationDistanceFunction()(q1, q2),
              q1.norm());

    EXPECT_THROW(dut.ComputeConfigurationDistance(q1, nan), std::exception);
    EXPECT_THROW(dut.ComputeConfigurationDistance(nan, q2), std::exception);
  }

  {
    // Interpolation function.

    // Evaluate (1) and (2) as constructed; default interpolation is linear
    // interpolation.
    EXPECT_EQ(dut.InterpolateBetweenConfigurations(q1, q2, 0.5),
              0.5 * (q1 + q2));
    ASSERT_NE(dut.MakeStandaloneConfigurationInterpolationFunction(), nullptr);
    EXPECT_EQ(
        dut.MakeStandaloneConfigurationInterpolationFunction()(q1, q2, 0.5),
        0.5 * (q1 + q2));

    // Change the function via (3).
    const ConfigurationInterpolationFunction interpolation_fn =
        [](const VectorXd& q, const VectorXd&, double alpha) {
          return VectorXd::Constant(q.size(), alpha);
        };
    dut.SetConfigurationInterpolationFunction(interpolation_fn);

    // Evaluate (1) and (2) again.
    EXPECT_EQ(dut.InterpolateBetweenConfigurations(q1, q2, 0.5),
              Eigen::VectorXd::Constant(q_size, 0.5));
    ASSERT_NE(dut.MakeStandaloneConfigurationInterpolationFunction(), nullptr);
    EXPECT_EQ(
        dut.MakeStandaloneConfigurationInterpolationFunction()(q1, q2, 0.5),
        Eigen::VectorXd::Constant(q_size, 0.5));

    // The interpolation function has a "reset" function that restores the
    // default interpolation function. Evaluate (1) and (2) again.
    dut.SetConfigurationInterpolationFunction(nullptr);
    EXPECT_EQ(dut.InterpolateBetweenConfigurations(q1, q2, 0.5),
              0.5 * (q1 + q2));
    ASSERT_NE(dut.MakeStandaloneConfigurationInterpolationFunction(), nullptr);
    EXPECT_EQ(
        dut.MakeStandaloneConfigurationInterpolationFunction()(q1, q2, 0.5),
        0.5 * (q1 + q2));

    // We're only testing against NaN as an indication that the upstream is
    // validating at all.
    EXPECT_THROW(dut.InterpolateBetweenConfigurations(q1, nan, 0.5),
                 std::exception);
    EXPECT_THROW(dut.InterpolateBetweenConfigurations(nan, q2, 0.5),
                 std::exception);
  }
}

// The default interpolation should handle quaternion and other joints in an
// expected way (SLERP for the former, LERP for the latter).
GTEST_TEST(EdgeCheckTest, DefaultInterpolation) {
  const ConfigurationDistanceFunction dist = [](const VectorXd& a,
                                                const VectorXd& b) {
    return (b - a).norm();
  };
  // We want a floating body to guarantee we have quaternions.
  auto dut = MakeEdgeChecker<CollisionCheckerTester>(
      dist, 0.1, nullptr /* default interpolator */, false /* welded */);

  const auto& plant = dut.plant();
  // Body "b0" should be floating (7 dof) and "b1" should be linked to "b0" by
  // a single revolute joint.
  ASSERT_EQ(dut.plant().num_positions(), 8);

  // Arbitrary initial pose for free body.
  const RigidTransform X_WB0_init(
      RotationMatrixd(AngleAxisd(M_PI / 7, Vector3d{1, 2, 3}.normalized())),
      Vector3d{0.5, -0.3, 1.2});
  const double j12_init = -M_PI + 0.1;

  // Final pose is defined as offset from initial pose. Define the offset.
  const Vector3d rot_axis_W = Vector3d{-3, 1, 2}.normalized();
  const double init_final_theta = 5 * M_PI / 7;
  const Vector3d p_InitFinal{-0.25, 0.75, 0.8};
  const RigidTransformd X_InitFinal(AngleAxisd(init_final_theta, rot_axis_W),
                                    p_InitFinal);

  // Now define the final.
  const RigidTransformd X_WB0_final = X_WB0_init * X_InitFinal;
  const double j12_final = M_PI - 0.1;

  // Given a pose of the free body and the angle theta between bodies b0 and b1,
  // returns the plant's q.
  auto get_q = [&plant](const RigidTransformd& X_B0, double theta) {
    const RigidBody<double>& body0 = plant.GetBodyByName("b0");
    auto plant_context = plant.CreateDefaultContext();
    plant.SetFreeBodyPose(plant_context.get(), body0, X_B0);
    VectorXd positions = plant.GetPositions(*plant_context);
    positions[7] = theta;
    return positions;
  };

  const VectorXd q_init = get_q(X_WB0_init, j12_init);
  const VectorXd q_final = get_q(X_WB0_final, j12_final);

  // Compute the interpolation based on the known relationship between init and
  // final.
  auto get_interpolated = [&](double s) {
    const Vector3d p_InitS = p_InitFinal * s;
    const RigidTransformd X_InitS(
        RotationMatrixd(AngleAxisd(init_final_theta * s, rot_axis_W)), p_InitS);
    const RigidTransformd X_WS = X_WB0_init * X_InitS;
    const double j12_s = j12_init + s * (j12_final - j12_init);
    return get_q(X_WS, j12_s);
  };

  for (const double s : {0.0, 0.25, 0.6, 1.0}) {
    EXPECT_TRUE(CompareMatrices(
        dut.InterpolateBetweenConfigurations(q_init, q_final, s),
        get_interpolated(s), 1e-15));
  }

  // Definitely not simply LERP.
  for (const double s : {0.25, 0.6}) {
    EXPECT_FALSE(CompareMatrices(
        dut.InterpolateBetweenConfigurations(q_init, q_final, s),
        q_init + s * (q_final - q_init), 1e-7));
  }
}

// The configuration of a test case for the MeasureEdgeCollisionFree* APIs.
struct EdgeTestConfig {
  // The expected interpolant value (in the range [0, 1]) returned by
  // MeasureEdgeCollisionFree(). If this is negative, we'll simply test for an
  // edge that is *not* partially free.
  double alpha;

  // The expected interpolant value (greater than alpha) that represents the
  // last colliding configuration.
  double last_colliding_alpha;

  // Request the calculation to be done in parallel.
  bool parallel;
};

std::ostream& operator<<(std::ostream& out, const EdgeTestConfig& c) {
  // Note: no spaces because we are using this as a gtest parameterized test
  // name.
  out << "EdgeTestWith" << c.alpha << "AlphaIn"
      << (c.parallel ? "Parallel" : "Serial");
  return out;
}

class ParameterizedEdgeCheckTest
    : public testing::TestWithParam<EdgeTestConfig> {};

// To evaluate the edge-checking APIs, we need direct control over when an edge
// is collision free and when it is not. The edge-checking APIs completely rely
// on the user-defined distance and interpolation functions, as well as the
// derived CollisionChecker's DoCheckContextConfigCollisionFree()
// implementation.

// To cater to both the Measure* and Check* variants, we can't just define an
// edge as being in collision for every sample *beyond* a certain interpolant
// value. If we were to do so, we couldn't test an edge that only has
// collisions on the *interior*.So, the test creates ranges of collision.
// Ranges with more than a single colliding sample allows us to confirm that
// algorithms always return the last valid interpolant value closest to the
// minimum colliding sample.
//
// To that end, each test characterizes the colliding interval with two test
// parameters: αᶠ (the "last" free value) and αᶜ (the last collision value)
// defined such that:
//
//   - αᶠ < αᶜ
//   - ∀ α ∈ [0, 1], and the interpolation qα = interpolation(q1, q2, α)
//     - α ∈ [0, αᶠ]  → qα is collision free;
//     - α ∈ (αᶠ, αᶜ] → qα is in collision;
//     - α ∈ (αᶜ, 1]  → qα is collision free.
//
// This class encodes values for α, αᶠ, and αᶜ inside the end configuration, q2.
// In particular, we set q2[0] = αᶠ, q2[1] = αᶜ, and q2[2] = α. No geometry is
// required and we only need enough bodies in the plant to provide sufficient
// robot dofs to hold the encoding.
//
// The configuration interpolator is responsible for providing the encoding to
// the collision check. The "interpolation" simply copies q2[0] and q2[1] into
// the first two entries of the interpolated result (q[0] and q[1]) and then
// sets the interpolant into q[2]. CollisionChecker updates the plant's
// positions with these values so that the collision test can read them. The
// interpolator always ignores the values of q1.
//
// The collision checker uses the "colliding range" (αᶠ, αᶜ] to determine if
// the current configuration is in collision. The test configurations will be
// set up so that multiple configurations will be colliding. When this range
// includes interpolant s = 1, special care must be taken for
// CheckEdgeCollisionFree() (see below for details).
//
// Finally, the distance function is responsible for guaranteeing we have a
// small, fixed number of steps on each edge. This serves two purposes: keeps
// the test runtime short and makes the values of the alpha value easier to
// reason about.
//
// Also, the checker tracks work done across threads and can report if work has
// been done in more than one thread.
class MockEdgeChecker : public UnimplementedCollisionChecker {
 public:
  explicit MockEdgeChecker(CollisionCheckerParams params)
      : UnimplementedCollisionChecker(std::move(params), true) {
    DRAKE_DEMAND(plant().num_positions() >= kQSize);
    AllocateContexts();
    const int num_omp_threads =
        common_robotics_utilities::openmp_helpers::GetNumOmpThreads();
    const int max_num_omp_threads =
        common_robotics_utilities::openmp_helpers::GetMaxNumOmpThreads();
    const int num_threads = std::max(num_omp_threads, max_num_omp_threads);
    thread_signals_ = vector<int>(num_threads, 0);
  }

  using CollisionChecker::CanEvaluateInParallel;

  // Returns the number of threads this was evaluated on.
  int thread_count() const {
    return std::accumulate(thread_signals_.begin(), thread_signals_.end(), 0);
  }

  // Force five samples based on the given `step_size`.
  static ConfigurationDistanceFunction MakeEdgeDistance(double step_size) {
    return [step_size](const VectorXd& q1, const VectorXd& q2) {
      // f(q, q) = 0 is a requirement of CollisionChecker and gets validated
      // when the function is set in the checker. That is the only time this
      // function will be evaluated with |q1| = 0, so we'll game the system to
      // convince the checker this is a valid distance function.
      if (q1.norm() == 0.0) return 0.0;
      // This *guarantees* four steps without recourse to numerical errors. Four
      // steps --> five samples.
      return step_size * 3.5;
    };
  }

  // Interpolation function responsible for propagating encoded parameters into
  // a new position vector.
  static ConfigurationInterpolationFunction MakeEdgeInterpolation() {
    return [](const VectorXd&, const VectorXd& q2, double s) {
      VectorXd result(q2);
      result(2) = s;
      return result;
    };
  }

  // Encodes a configuration vector q* with the information about whether the
  // edge (q, q*) is in collision, and, if so, where. The result has the
  // values:
  //  q(0): The highest value of alpha that is collision free.
  //  q(1): The highest value of alpha that is colliding; q(1) > q(0).
  //  q(2): We set it to 1.0 so that when we encode the end point configuration,
  //        the value for interpolant is consistent with that interpretation.
  //        Necessary for CheckEdgeCollisionFree() as it doesn't interpolate
  //        prior to testing the end point.
  static VectorXd EncodeConfiguration(int size, double last_free,
                                      double last_colliding) {
    DRAKE_DEMAND(size >= kQSize);
    DRAKE_DEMAND(last_free <= 1.0);
    DRAKE_DEMAND(last_colliding > last_free);

    VectorXd q = VectorXd::Zero(size);
    q(0) = last_free;
    q(1) = last_colliding;
    q(2) = 1.0;
    return q;
  }

  static constexpr int kQSize = 3;
  static constexpr int kNumSamples = 5;

 protected:
  bool DoCheckContextConfigCollisionFree(
      const CollisionCheckerContext& model_context) const override {
    // Make this call artificially more expensive so that parallel edge checks
    // will actually perform work in multiple OpenMP threads.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const int thread_index =
        common_robotics_utilities::openmp_helpers::GetContextOmpThreadNum();
    thread_signals_[thread_index] = 1;
    const auto q = plant().GetPositions(model_context.plant_context());
    const double s = q(2);
    const bool free = s <= q(0) || q(1) < s;
    return free;
  }

  // We just want this to *not* throw.
  void DoUpdateContextPositions(CollisionCheckerContext*) const override {}

  // A per-thread signal; if the code was exercised in thread i, the value
  // at the ith index is one, otherwise zero.
  mutable vector<int> thread_signals_;
};

std::vector<EdgeTestConfig> MakeEdgeTestCases() {
  std::vector<EdgeTestConfig> configs;

  const double divisor = static_cast<double>(MockEdgeChecker::kNumSamples - 1);

  for (const bool in_parallel : {true, false}) {
    if (in_parallel & !kHasOpenmp) {
      // We don't have OpenMP in all test configurations.
      continue;
    }

    // Edges are 100% valid.
    configs.push_back(
        {.alpha = 1.0, .last_colliding_alpha = 2.0, .parallel = in_parallel});

    // Edges are invalid to varying degrees (this includes edges where q1 is not
    // valid -- alpha < 0).
    for (int step = 0; step < MockEdgeChecker::kNumSamples; ++step) {
      const double free = (step - 1) / divisor;
      // We want *two* samples to be in collision.
      const double colliding = (step + 1) / divisor;
      configs.push_back({.alpha = free,
                         .last_colliding_alpha = colliding,
                         .parallel = in_parallel});
    }
  }

  return configs;
}

INSTANTIATE_TEST_SUITE_P(EdgeCheckTest, ParameterizedEdgeCheckTest,
                         testing::ValuesIn(MakeEdgeTestCases()));

// CollisionChecker logic for checking a single edge in serial includes the
// following unique responsibilities:
//
//   1. Makes use of edge_step_size and distance function to compute the
//      expected number of samples.
//   2. It uses the registered interpolation function to create sample
//      configurations.
//   3. All edge samples are evaluated to report the edge free of collision.
//   4. Failure on any given sample returns the right alpha value.
//
// (1) and (2) will be validated by the test producing the result that we
// expect, when we expect. The configurations provided are nonsensical. The
// only way for them to make sense, is to use our provided functions and
// edge step size.
//
// (3) We'll provide test cases such that collisions at all possible locations
// are covered. This provides evidence that every sample is tested.
//
// (4) Finally, we confirm the reported alpha with the expected alpha.
//
// We hit both MeasureEdgeCollisionFree() and MeasureEdgeCollisionFreeParallel()
// in this test. For a given configuration, we set up a scenario which should
// produce a requested number of samples (with some portion of them being
// collision free). We explicitly exercise MeasureEdgeCollisionFree knowing it
// delegates to MeasureContextEdgeCollisionFree, getting two tests for one.
//
// This test depends *heavily* on MockEdgeChecker to map test configuration to
// the expected outcome. Understanding that mechanism is a prerequisite to
// understanding this test.
TEST_P(ParameterizedEdgeCheckTest, MeasureEdgeCollisionFree) {
  const EdgeTestConfig& config = GetParam();
  ASSERT_LE(config.alpha, 1.0);

  // Set up the mock checker with our mock functions.

  // Arbitrary value; friendly to base 2.
  const double step_size = 0.25;
  const int q_size = MockEdgeChecker::kQSize;
  auto dut = MakeEdgeChecker<MockEdgeChecker>(
      MockEdgeChecker::MakeEdgeDistance(step_size), step_size,
      MockEdgeChecker::MakeEdgeInterpolation(), true /* welded */,
      q_size + 1 /* num_bodies */);

  // Reality check. If we've requested parallel evaluation we need to confirm
  // it'll happen; otherwise we're simply testing the serial implementation
  // again.
  if (config.parallel) {
    ASSERT_TRUE(dut.CanEvaluateInParallel());
  }

  // Start configuration values are simply ignored.
  const VectorXd q1 = VectorXd::Constant(q_size, 0.75);
  // End configuration encodes failure based on the expected colliding range.
  const VectorXd q2 = dut.EncodeConfiguration(q_size, config.alpha,
                                              config.last_colliding_alpha);
  const VectorXd nan =
      VectorXd::Constant(q_size, std::numeric_limits<double>::quiet_NaN());

  const EdgeMeasure result = config.parallel
                                 ? dut.MeasureEdgeCollisionFreeParallel(q1, q2)
                                 : dut.MeasureEdgeCollisionFree(q1, q2);

  const double expected_alpha = config.alpha;
  if (expected_alpha == 1.0) {
    EXPECT_TRUE(result.completely_free());
  }
  if (expected_alpha >= 0) {
    ASSERT_TRUE(result.partially_free());
    EXPECT_EQ(result.alpha(), expected_alpha);
  } else {
    EXPECT_FALSE(result.completely_free());
    EXPECT_FALSE(result.partially_free());
    EXPECT_THROW(result.alpha(), std::exception);
  }
  // Are things parallel as we expect?
  if (config.parallel) {
    EXPECT_GT(dut.thread_count(), 1);
  }

  // We're only testing against NaN as an indication that the upstream is
  // validating at all.
  if (config.parallel) {
    EXPECT_THROW(dut.MeasureEdgeCollisionFree(q1, nan), std::exception);
    EXPECT_THROW(dut.MeasureEdgeCollisionFree(nan, q2), std::exception);
  } else {
    EXPECT_THROW(dut.MeasureEdgeCollisionFreeParallel(q1, nan), std::exception);
    EXPECT_THROW(dut.MeasureEdgeCollisionFreeParallel(nan, q2), std::exception);
  }
}

// Same test as for MeasureEdgeCollisionFree, but uses the boolean variant.
// There is one *further* change due to the differences between Check* and Is*.
// The Is* variant does an early check on q2. If q2 isn't free, it never tests
// anything else. This means that in the parallel mode, we never enter parallel
// evaluation.
TEST_P(ParameterizedEdgeCheckTest, CheckEdgeCollisionFree) {
  const EdgeTestConfig& config = GetParam();
  ASSERT_LE(config.alpha, 1.0);

  // Set up the mock checker with our mock functions.

  // Arbitrary value; friendly to base 2.
  const double step_size = 0.25;
  const int q_size = MockEdgeChecker::kQSize;
  auto dut = MakeEdgeChecker<MockEdgeChecker>(
      MockEdgeChecker::MakeEdgeDistance(step_size), step_size,
      MockEdgeChecker::MakeEdgeInterpolation(), true /* welded */,
      q_size + 1 /* num_bodies */);

  // Reality check. If we've requested parallel evaluation we need to confirm
  // it'll happen; otherwise we're simply testing the serial implementation
  // again.
  if (config.parallel) {
    ASSERT_TRUE(dut.CanEvaluateInParallel());
  }

  // Start configuration values are simply ignored.
  const VectorXd q1 = VectorXd::Constant(q_size, 0.75);
  // End configuration encodes failure based on the expected colliding range.
  const VectorXd q2 = dut.EncodeConfiguration(q_size, config.alpha,
                                              config.last_colliding_alpha);
  const VectorXd nan =
      VectorXd::Constant(q_size, std::numeric_limits<double>::quiet_NaN());

  const bool result = config.parallel
                          ? dut.CheckEdgeCollisionFreeParallel(q1, q2)
                          : dut.CheckEdgeCollisionFree(q1, q2);

  // The encoded alpha (as documented above on EncodeConfiguration()).
  const bool expected_result = config.alpha == 1.0;
  EXPECT_EQ(result, expected_result);
  if (config.parallel) {
    if (config.alpha >= 0.5 && config.alpha < 1.0) {
      // If 0.5 <= expected_result < 1.0, then the colliding range includes q2
      // (due to the fact that there are five samples, each 0.25 away from
      // each other and that q[0] and q[1] are two samples away from each
      // other).
      // For the parallel implementation, we should fail fast on q2 and never
      // enter the parallel regime.
      EXPECT_EQ(dut.thread_count(), 1);
    } else {
      EXPECT_GT(dut.thread_count(), 1);
    }
  }

  // We're only testing against NaN as an indication that the upstream is
  // validating at all.
  if (config.parallel) {
    EXPECT_THROW(dut.CheckEdgeCollisionFree(q1, nan), std::exception);
    EXPECT_THROW(dut.CheckEdgeCollisionFree(nan, q2), std::exception);
  } else {
    EXPECT_THROW(dut.CheckEdgeCollisionFreeParallel(q1, nan), std::exception);
    EXPECT_THROW(dut.CheckEdgeCollisionFreeParallel(nan, q2), std::exception);
  }
}

// The test for MeasureEdgesCollisionFree() (plural) uses
// MeasureEdgeCollisionFree() (singular) to test individual edges. For this
// function, we only need to test:
//
//   1. Do *all* the edges get evaluated with the expected result?
//   2. Does the `parallelize` parameter have the documented effect?
//
GTEST_TEST(EdgeCheckTest, MeasureMultipleEdges) {
  // Use the MockEdgeChecker to control edge failures.
  const double step_size = 0.05;
  auto calc_dist = MockEdgeChecker::MakeEdgeDistance(step_size);
  auto interp = MockEdgeChecker::MakeEdgeInterpolation();

  const int q_size = MockEdgeChecker::kQSize;

  const VectorXd nan =
      VectorXd::Constant(q_size, std::numeric_limits<double>::quiet_NaN());

  // Garbage start configuration; the values are ignored.
  const VectorXd q_start = VectorXd::Constant(q_size, 0.75);
  // All edges start from the same configuration, q_start. The end configuration
  // gets encoded by MockEdgeChecker.
  // Note: we want to make sure that the parallel collision checker properly
  // reconciles multiple internal colliding configurations, reporting the
  // earliest. To test that, we need two colliding internal samples. We know
  // that we're getting five samples (by design) so the distance between values
  // of alpha is 0.25. So, to make sure *two* samples are in collision, the last
  // *colliding* alpha should be *two* samples values beyond the last free, or
  // last free + 0.5.
  vector<std::pair<VectorXd, VectorXd>> edges;
  // 100% clear.
  edges.emplace_back(q_start,
                     MockEdgeChecker::EncodeConfiguration(q_size, 1.0, 1.5));
  // Quarter free (collision at 0.5 and 0.75).
  edges.emplace_back(q_start,
                     MockEdgeChecker::EncodeConfiguration(q_size, 0.25, 0.75));
  // Three quarters free (collision at 1.0).
  edges.emplace_back(q_start,
                     MockEdgeChecker::EncodeConfiguration(q_size, 0.75, 1.25));
  const VectorXd& q2 = edges[0].second;

  // The distance for all of the edges is the same.
  const double edge_dist = calc_dist(edges[0].first, edges[0].second);
  const vector<EdgeMeasure> expected_results{
      EdgeMeasure(edge_dist, edges[0].second(0)),
      EdgeMeasure(edge_dist, edges[1].second(0)),
      EdgeMeasure(edge_dist, edges[2].second(0))};

  for (const bool parallel : {false, true}) {
    if (parallel & !kHasOpenmp) {
      // We don't have OpenMP in all test configurations.
      continue;
    }
    auto dut = MakeEdgeChecker<MockEdgeChecker>(calc_dist, step_size, interp,
                                                true /* welded */, q_size + 1);
    ASSERT_EQ(dut.plant().num_positions(), q_size);

    // Reality check; if we've requested parallel evaluation we need to confirm
    // it'll happen; otherwise we're simply testing the serial implementation
    // again.
    if (parallel) {
      ASSERT_TRUE(dut.CanEvaluateInParallel());
    }

    vector<EdgeMeasure> results =
        dut.MeasureEdgesCollisionFree(edges, parallel);

    // Confirm behavior (1). Results for every edge.
    EXPECT_EQ(results, expected_results);
    // Confirm behavior (2). Parallel when asked, serial when not.
    if (parallel) {
      EXPECT_GT(dut.thread_count(), 1);
    } else {
      EXPECT_EQ(dut.thread_count(), 1);
    }
    // We make sure there is one valid edge first so we know we catch the
    // invalid edge, even if it isn't first.
    // We're only testing against NaN as an indication that the upstream is
    // validating at all.
    EXPECT_THROW(dut.MeasureEdgesCollisionFree({{q_start, q2}, {q_start, nan}},
                                               parallel),
                 std::exception);
    EXPECT_THROW(
        dut.MeasureEdgesCollisionFree({{q_start, q2}, {nan, q2}}, parallel),
        std::exception);
  }
}

// Same test as for MeasureEdgeCollisionFree, but uses the boolean variant.
GTEST_TEST(EdgeCheckTest, CheckMultipleEdgesFree) {
  // Use the MockEdgeChecker to control edge failures.
  const double step_size = 0.05;
  auto calc_dist = MockEdgeChecker::MakeEdgeDistance(step_size);
  auto interp = MockEdgeChecker::MakeEdgeInterpolation();

  const int q_size = MockEdgeChecker::kQSize;

  const VectorXd nan =
      VectorXd::Constant(q_size, std::numeric_limits<double>::quiet_NaN());

  // Garbage start configuration; the values are ignored.
  const VectorXd q_start = VectorXd::Constant(q_size, 0.75);
  // All edges start from the same configuration, q_start. The end configuration
  // gets encoded by MockEdgeChecker. Note: we know that we're getting five
  // samples so the distance between values of alpha is 0.25. So, to make sure
  // *two* samples are in collision, the last colliding alpha should be the
  // last free + 0.5.
  vector<std::pair<VectorXd, VectorXd>> edges;
  // 100% clear.
  edges.emplace_back(q_start,
                     MockEdgeChecker::EncodeConfiguration(q_size, 1.0, 1.5));
  // Early collision.
  edges.emplace_back(q_start,
                     MockEdgeChecker::EncodeConfiguration(q_size, 0.25, 0.75));
  // Later collision.
  edges.emplace_back(q_start,
                     MockEdgeChecker::EncodeConfiguration(q_size, 0.75, 1.25));
  const VectorXd& q2 = edges[0].second;

  const vector<uint8_t> expected_results{edges[0].second(0) == 1.0,
                                         edges[1].second(0) == 1.0,
                                         edges[2].second(0) == 1.0};

  for (const bool parallel : {false, true}) {
    if (parallel & !kHasOpenmp) {
      // We don't have OpenMP in all test configurations.
      continue;
    }
    auto dut = MakeEdgeChecker<MockEdgeChecker>(calc_dist, step_size, interp,
                                                true /* welded */, q_size + 1);
    ASSERT_EQ(dut.plant().num_positions(), q_size);

    // Reality check; if we've requested parallel evaluation we need to confirm
    // it'll happen; otherwise we're simply testing the serial implementation
    // again.
    if (parallel) {
      ASSERT_TRUE(dut.CanEvaluateInParallel());
    }

    vector<uint8_t> results = dut.CheckEdgesCollisionFree(edges, parallel);

    // Confirm behavior (1). Results for every edge.
    EXPECT_EQ(results, expected_results);
    // Confirm behavior (2). Parallel when asked, serial when not.
    if (parallel) {
      EXPECT_GT(dut.thread_count(), 1);
    } else {
      EXPECT_EQ(dut.thread_count(), 1);
    }
    // We make sure there is one valid edge first so we know we catch the
    // invalid edge, even if it isn't first.
    // We're only testing against NaN as an indication that the upstream is
    // validating at all.
    EXPECT_THROW(
        dut.CheckEdgesCollisionFree({{q_start, q2}, {q_start, nan}}, parallel),
        std::exception);
    EXPECT_THROW(
        dut.CheckEdgesCollisionFree({{q_start, q2}, {nan, q2}}, parallel),
        std::exception);
  }
}

// Additional test cases for basic EdgeMeasure functionality not covered already
// in the above cases.
GTEST_TEST(EdgeMeasureTest, Test) {
  const EdgeMeasure completely_free(10.0, 1.0);
  EXPECT_EQ(completely_free.distance(), 10.0);
  EXPECT_EQ(completely_free.alpha(), 1.0);
  EXPECT_TRUE(completely_free.completely_free());
  EXPECT_TRUE(completely_free.partially_free());
  EXPECT_EQ(completely_free.alpha_or(-1.0), 1.0);

  const EdgeMeasure partially_free(10.0, 0.5);
  EXPECT_EQ(partially_free.distance(), 10.0);
  EXPECT_EQ(partially_free.alpha(), 0.5);
  EXPECT_FALSE(partially_free.completely_free());
  EXPECT_TRUE(partially_free.partially_free());
  EXPECT_EQ(partially_free.alpha_or(-1.0), 0.5);

  const EdgeMeasure invalid(10.0, -5.0);
  EXPECT_EQ(invalid.distance(), 10.0);
  EXPECT_THROW(invalid.alpha(), std::exception);
  EXPECT_FALSE(invalid.completely_free());
  EXPECT_FALSE(invalid.partially_free());
  EXPECT_EQ(invalid.alpha_or(-10.0), -10.0);

  // Invalid distance throws.
  EXPECT_THROW(EdgeMeasure(-1.0, 0.0), std::exception);
  // Invalid alpha throws.
  EXPECT_THROW(EdgeMeasure(1.0, 10.0), std::exception);
}

}  // namespace
}  // namespace planning
}  // namespace drake
