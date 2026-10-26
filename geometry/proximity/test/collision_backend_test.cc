#include "drake/geometry/proximity/collision_backend.h"

#include <cstdlib>

#include <gtest/gtest.h>

#include "drake/geometry/proximity_engine.h"

namespace drake {
namespace geometry {
namespace internal {

/* ProximityEngine's backend accessor is private and befriends this name. */
class ProximityEngineTester {
 public:
  ProximityEngineTester() = delete;

  template <typename T>
  static CollisionBackend backend(const ProximityEngine<T>& engine) {
    return engine.collision_backend();
  }
};

namespace {

GTEST_TEST(CollisionBackendTest, ToString) {
  EXPECT_EQ(to_string(CollisionBackend::kFcl), "fcl");
  EXPECT_EQ(to_string(CollisionBackend::kCoal), "coal");
}

/* The parsing is factored out of the environment read so that it can be
 tested directly; the environment itself is read once per process, on first
 use, and so cannot be re-exercised from within a test. */
GTEST_TEST(CollisionBackendTest, ParseNames) {
  EXPECT_EQ(ParseCollisionBackend("fcl"), CollisionBackend::kFcl);
  EXPECT_EQ(ParseCollisionBackend("coal"), CollisionBackend::kCoal);

  // Case insensitive, as an environment variable should be.
  EXPECT_EQ(ParseCollisionBackend("FCL"), CollisionBackend::kFcl);
  EXPECT_EQ(ParseCollisionBackend("Coal"), CollisionBackend::kCoal);
  EXPECT_EQ(ParseCollisionBackend("CoAl"), CollisionBackend::kCoal);

  // Anything else names no backend; the caller warns and falls back.
  EXPECT_FALSE(ParseCollisionBackend("").has_value());
  EXPECT_FALSE(ParseCollisionBackend("hpp-fcl").has_value());
  EXPECT_FALSE(ParseCollisionBackend("fcl ").has_value());
  EXPECT_FALSE(ParseCollisionBackend("bullet").has_value());
}

/* Every name to_string() produces must parse back to the same backend; this
 is what lets a user copy a logged backend name into the environment. */
GTEST_TEST(CollisionBackendTest, RoundTrip) {
  for (const auto backend : {CollisionBackend::kFcl, CollisionBackend::kCoal}) {
    EXPECT_EQ(ParseCollisionBackend(to_string(backend)), backend);
  }
}

class DefaultBackendTest : public ::testing::Test {
 protected:
  void SetUp() override { original_ = GetDefaultCollisionBackend(); }
  void TearDown() override { SetDefaultCollisionBackend(original_); }

  CollisionBackend original_{};
};

/* The default follows the environment: unset or unrecognized means FCL,
 otherwise whatever was named. This file is built twice -- once with
 DRAKE_PROXIMITY_BACKEND unset and once with it set to "coal" -- so this one
 assertion covers both the shipped default and the environment read. */
TEST_F(DefaultBackendTest, HonorsTheEnvironment) {
  const char* const env = std::getenv("DRAKE_PROXIMITY_BACKEND");
  const CollisionBackend expected =
      (env == nullptr)
          ? CollisionBackend::kFcl
          : ParseCollisionBackend(env).value_or(CollisionBackend::kFcl);
  EXPECT_EQ(GetDefaultCollisionBackend(), expected);
}

TEST_F(DefaultBackendTest, SetterTakesEffect) {
  SetDefaultCollisionBackend(CollisionBackend::kCoal);
  EXPECT_EQ(GetDefaultCollisionBackend(), CollisionBackend::kCoal);
  SetDefaultCollisionBackend(CollisionBackend::kFcl);
  EXPECT_EQ(GetDefaultCollisionBackend(), CollisionBackend::kFcl);
}

/* A default-constructed engine follows the selector. */
TEST_F(DefaultBackendTest, EngineFollowsTheDefault) {
  SetDefaultCollisionBackend(CollisionBackend::kCoal);
  const ProximityEngine<double> engine;
  EXPECT_EQ(ProximityEngineTester::backend(engine), CollisionBackend::kCoal);
}

/* The property the whole design rests on: an engine latches its backend at
 construction. A simulation already under way cannot be perturbed by a later
 call to the setter, and the latch survives copying and scalar conversion. */
TEST_F(DefaultBackendTest, EngineLatchesItsBackend) {
  SetDefaultCollisionBackend(CollisionBackend::kCoal);
  const ProximityEngine<double> engine;
  ASSERT_EQ(ProximityEngineTester::backend(engine), CollisionBackend::kCoal);

  // Change the global out from under the live engine.
  SetDefaultCollisionBackend(CollisionBackend::kFcl);
  EXPECT_EQ(ProximityEngineTester::backend(engine), CollisionBackend::kCoal);

  // And the latch is what gets copied and converted, not the global.
  const ProximityEngine<double> copied_engine(engine);
  EXPECT_EQ(ProximityEngineTester::backend(copied_engine),
            CollisionBackend::kCoal);
  const auto ad_engine = engine.ToScalarType<AutoDiffXd>();
  EXPECT_EQ(ProximityEngineTester::backend(*ad_engine),
            CollisionBackend::kCoal);

  // A *newly* constructed engine does follow the new default.
  const ProximityEngine<double> later_engine;
  EXPECT_EQ(ProximityEngineTester::backend(later_engine),
            CollisionBackend::kFcl);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
