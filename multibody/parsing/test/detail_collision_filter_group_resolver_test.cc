#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using ::testing::MatchesRegex;

using geometry::GeometryId;
using geometry::SceneGraph;
using math::RigidTransformd;

class CollisionFilterGroupResolverTest : public test::DiagnosticPolicyTestBase {
 public:
  CollisionFilterGroupResolverTest() {
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  }

  // Add a body to a model, with collision geometry and return the geometry id.
  GeometryId AddBody(const std::string& name,
                     std::optional<ModelInstanceIndex> model_instance) {
    auto model = model_instance.value_or(default_model_instance());
    return plant_.RegisterCollisionGeometry(
        plant_.AddRigidBody(name, model, SpatialInertia<double>()),
        RigidTransformd::Identity(), geometry::Sphere(1.0), name,
        CoulombFriction<double>());
  }

 protected:
  MultibodyPlant<double> plant_{0.0};
  SceneGraph<double> scene_graph_;
  const geometry::SceneGraphInspector<double>& inspector_{
      scene_graph_.model_inspector()};
  internal::CollisionFilterGroupResolver resolver_{&plant_};
};

// These tests concentrate on name resolution details and responses to errors
// in the parsed text.  TODO(rpoyner-tri) Migrate actual filter construction
// tests here from detail_{urdf,sdf}_parser_test.

TEST_F(CollisionFilterGroupResolverTest, BogusPairScoped) {
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  resolver_.AddPair(diagnostic_policy_, "a", "sub::b", r1);
  resolver_.Resolve(diagnostic_policy_);
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'r1::a'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'r1::sub::b'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, BogusPairGlobal) {
  resolver_.AddPair(diagnostic_policy_, "a", "b", {});
  resolver_.Resolve(diagnostic_policy_);
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'a'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'b'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, BogusGroupName) {
  resolver_.AddGroup(diagnostic_policy_, "haha::a", {}, {});
  EXPECT_THAT(TakeError(), MatchesRegex(".*'haha::a' cannot be.*scoped.*"));
}

TEST_F(CollisionFilterGroupResolverTest, EmptyGroupGlobal) {
  resolver_.AddGroup(diagnostic_policy_, "a", {}, {});
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'a'.*no members"));
}

TEST_F(CollisionFilterGroupResolverTest, EmptyGroupsScoped) {
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  resolver_.AddGroup(diagnostic_policy_, "a", {}, r1);
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'r1::a'.*no members"));
  ModelInstanceIndex r2 = plant_.AddModelInstance("r2");
  resolver_.AddGroup(diagnostic_policy_, "b", {}, r2);
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'r2::b'.*no members"));
}

TEST_F(CollisionFilterGroupResolverTest, MissingBodyGlobal) {
  plant_.AddModelInstance("r1");
  resolver_.AddGroup(diagnostic_policy_, "a",
                     {"abody", "r1::abody", "zzz::abody"}, {});
  EXPECT_THAT(TakeError(), MatchesRegex(".*'abody'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::abody'.*not found"));
  // Note that AddGroup() doesn't choke on an instance name that doesn't exist.
  EXPECT_THAT(TakeError(), MatchesRegex(".*'zzz::abody'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, OutOfParseBodyGlobal) {
  // The world body and bodies in the default model are always outside of any
  // parse operation.
  AddBody("stuff", {});
  resolver_.AddGroup(diagnostic_policy_, "a", {
      "DefaultModelInstance::stuff",
      "WorldModelInstance::world",
      "stuff",
    },
    {});
  EXPECT_THAT(TakeError(), MatchesRegex(".*'DefaultModelInstance::stuff'"
                                        ".*outside the current parse"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'WorldModelInstance::world'"
                                        ".*outside the current parse"));
  // Ensure that unqualified bodies names at global scope aren't looked up in
  // the default model inadvertently.
  EXPECT_THAT(TakeError(), MatchesRegex(".*'stuff'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, MissingBodyScoped) {
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  plant_.AddModelInstance("r1::sub");
  resolver_.AddGroup(diagnostic_policy_, "a",
                     {"abody", "sub::abody", "zzz::abody"}, r1);
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::abody'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::sub::abody'.*not found"));
  // Note that AddGroup() doesn't choke on an instance name that doesn't exist.
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::zzz::abody'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, GroupGlobal) {
  // Ensure names can be resolved for groups defined at global scope.
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  ModelInstanceIndex r2 = plant_.AddModelInstance("r2");
  GeometryId b1 = AddBody("body1", r1);
  GeometryId b2 = AddBody("body2", r2);
  resolver_.AddGroup(diagnostic_policy_, "a", {"r1::body1"}, {});
  resolver_.AddGroup(diagnostic_policy_, "b", {"r2::body2"}, {});
  resolver_.AddPair(diagnostic_policy_, "a", "b", {});
  resolver_.Resolve(diagnostic_policy_);
  EXPECT_TRUE(inspector_.CollisionFiltered(b1, b2));
}

TEST_F(CollisionFilterGroupResolverTest, LinkScoped) {
  // Ensure body names can be resolved in various model scopes.
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  GeometryId ra = AddBody("abody", r1);
  ModelInstanceIndex sub = plant_.AddModelInstance("r1::sub");
  GeometryId rsa = AddBody("abody", sub);

  resolver_.AddGroup(diagnostic_policy_, "a", {"abody", "sub::abody"}, r1);
  resolver_.AddPair(diagnostic_policy_, "a", "a", r1);
  resolver_.Resolve(diagnostic_policy_);

  // The expected filter gets created on the plant/scene_graph.
  EXPECT_TRUE(inspector_.CollisionFiltered(ra, rsa));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
