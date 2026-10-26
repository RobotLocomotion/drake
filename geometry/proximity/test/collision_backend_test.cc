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

/* The default follows the environment: unset or unrecognized means FCL,
 otherwise whatever was named. The same test program runs once with
 DRAKE_PROXIMITY_BACKEND set to "fcl" and once with it set to "coal", so this
 assertion covers both recognized values and the environment read. */
GTEST_TEST(CollisionBackendTest, HonorsTheEnvironment) {
  const char* const env = std::getenv("DRAKE_PROXIMITY_BACKEND");
  const CollisionBackend expected =
      (env == nullptr)
          ? CollisionBackend::kFcl
          : ParseCollisionBackend(env).value_or(CollisionBackend::kFcl);
  EXPECT_EQ(GetDefaultCollisionBackend(), expected);
}

/* A default-constructed engine follows the selector. */
GTEST_TEST(CollisionBackendTest, EngineFollowsTheDefault) {
  const ProximityEngine<double> engine;
  EXPECT_EQ(ProximityEngineTester::backend(engine),
            GetDefaultCollisionBackend());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
