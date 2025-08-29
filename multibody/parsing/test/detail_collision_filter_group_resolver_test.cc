#include "drake/multibody/parsing/detail_collision_filter_group_resolver.h"

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

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
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::Sphere;
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
        plant_.AddRigidBody(name, model), RigidTransformd::Identity(),
        geometry::Sphere(1.0), name, CoulombFriction<double>());
  }

 protected:
  MultibodyPlant<double> plant_{0.01};
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
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  resolver_.AddGroup(diagnostic_policy_, "haha::a", {}, {}, r1);
  EXPECT_THAT(TakeError(), MatchesRegex(".*haha::a' cannot be.*scoped.*"));
}

TEST_F(CollisionFilterGroupResolverTest, EmptyGroupGlobal) {
  resolver_.AddGroup(diagnostic_policy_, "a", {}, {}, {});
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'a'.*no members"));
}

TEST_F(CollisionFilterGroupResolverTest, EmptyGroupsScoped) {
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  resolver_.AddGroup(diagnostic_policy_, "a", {}, {}, r1);
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'r1::a'.*no members"));
  ModelInstanceIndex r2 = plant_.AddModelInstance("r2");
  resolver_.AddGroup(diagnostic_policy_, "b", {}, {}, r2);
  EXPECT_THAT(TakeError(), MatchesRegex(".*group.*'r2::b'.*no members"));
}

TEST_F(CollisionFilterGroupResolverTest, MissingBodyGlobal) {
  plant_.AddModelInstance("r1");
  resolver_.AddGroup(diagnostic_policy_, "a",
                     {"abody", "r1::abody", "zzz::abody"}, {}, {});
  EXPECT_THAT(TakeError(), MatchesRegex(".*'abody'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::abody'.*not found"));
  // Note that AddGroup() doesn't choke on an instance name that doesn't exist.
  EXPECT_THAT(TakeError(), MatchesRegex(".*'zzz::abody'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, MissingMemberGroupGlobal) {
  plant_.AddModelInstance("r1");
  resolver_.AddGroup(diagnostic_policy_, "a", {},
                     {"agroup", "r1::agroup", "zzz::agroup"}, {});
  // Group insertions get evaluated at resolution time.
  resolver_.Resolve(diagnostic_policy_);

  EXPECT_THAT(TakeError(), MatchesRegex(".*'agroup'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::agroup'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'zzz::agroup'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, DuplicateGroupDefinitions) {
  plant_.AddModelInstance("r1");
  // The definitions are nonsense, but the redefinition attempt should emit
  // error right away.
  resolver_.AddGroup(diagnostic_policy_, "a", {},
                     {"agroup", "r1::agroup", "zzz::agroup"}, {});
  resolver_.AddGroup(diagnostic_policy_, "a", {},
                     {"agroup", "r1::agroup", "zzz::agroup"}, {});
  EXPECT_THAT(TakeError(), MatchesRegex(".*'a'.*already.*defined.*"));
}

TEST_F(CollisionFilterGroupResolverTest, OutOfParseBodyGlobal) {
  // The world body and bodies in the default model are always outside of any
  // parse operation.

  // The resolver knows (by the minimum model index rule) that these models are
  // out of bounds.
  ASSERT_LT(plant_.GetModelInstanceByName("DefaultModelInstance"),
            resolver_.minimum_model_instance_index());
  ASSERT_LT(plant_.GetModelInstanceByName("WorldModelInstance"),
            resolver_.minimum_model_instance_index());

  AddBody("stuff", {});
  resolver_.AddGroup(diagnostic_policy_, "a",
                     {
                         "DefaultModelInstance::stuff",
                         "WorldModelInstance::world",
                         "stuff",
                     },
                     {}, {});
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
                     {"abody", "sub::abody", "zzz::abody"}, {}, r1);
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::abody'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::sub::abody'.*not found"));
  // Note that AddGroup() doesn't choke on an instance name that doesn't exist.
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::zzz::abody'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, MissingMemberGroupScoped) {
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  plant_.AddModelInstance("r1::sub");
  resolver_.AddGroup(diagnostic_policy_, "a", {},
                     {"agroup", "sub::agroup", "zzz::agroup"}, r1);
  // Group insertions get evaluated at resolution time.
  resolver_.Resolve(diagnostic_policy_);

  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::agroup'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::sub::agroup'.*not found"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'r1::zzz::agroup'.*not found"));
}

TEST_F(CollisionFilterGroupResolverTest, AmbiguousRigidAndDeformableName) {
  ModelInstanceIndex model = plant_.AddModelInstance("amodel");

  // Add a rigid body to the model.
  AddBody("abody", model);

  // Add a deformable body to the model with the same name.
  auto& deformable_model = plant_.mutable_deformable_model();
  auto sphere = std::make_unique<GeometryInstance>(
      math::RigidTransformd::Identity(), std::make_unique<Sphere>(1.0),
      "abody");
  deformable_model.RegisterDeformableBody(std::move(sphere), model,
                                          /*config=*/{},
                                          /*resolution_hint=*/2.0);

  // Ensure that the resolver catches the ambiguous name in the group body
  // names.
  resolver_.AddGroup(diagnostic_policy_, "a", {"abody"}, {}, model);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*body.*'amodel::abody'.*is ambiguous.*"));
}

TEST_F(CollisionFilterGroupResolverTest, GroupGlobalBodies) {
  // Ensure body names can be resolved for groups defined at global scope.
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  ModelInstanceIndex r2 = plant_.AddModelInstance("r2");
  GeometryId b1 = AddBody("body1", r1);
  GeometryId b2 = AddBody("body2", r2);
  resolver_.AddGroup(diagnostic_policy_, "a", {"r1::body1"}, {}, {});
  resolver_.AddGroup(diagnostic_policy_, "b", {"r2::body2"}, {}, {});
  resolver_.AddPair(diagnostic_policy_, "a", "b", {});
  const CollisionFilterGroupsImpl<std::string> actual_groups =
      ConvertInstancedNamesToStrings(resolver_.Resolve(diagnostic_policy_),
                                     plant_);

  CollisionFilterGroupsImpl<std::string> expected_groups;
  expected_groups.AddGroup("a", {"r1::body1"});
  expected_groups.AddGroup("b", {"r2::body2"});
  expected_groups.AddExclusionPair({"a", "b"});
  EXPECT_EQ(actual_groups, expected_groups);
  EXPECT_TRUE(inspector_.CollisionFiltered(b1, b2));
}

TEST_F(CollisionFilterGroupResolverTest, GroupGlobalMemberGroups) {
  // Ensure member group names can be resolved for groups defined at global
  // scope.
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  ModelInstanceIndex r2 = plant_.AddModelInstance("r2");

  GeometryId b1 = AddBody("body1", r1);
  resolver_.AddGroup(diagnostic_policy_, "ra", {"body1"}, {}, r1);
  GeometryId b2 = AddBody("body2", r2);
  resolver_.AddGroup(diagnostic_policy_, "rb", {"body2"}, {}, r2);

  resolver_.AddGroup(diagnostic_policy_, "a", {}, {"r1::ra"}, {});
  resolver_.AddGroup(diagnostic_policy_, "b", {}, {"r2::rb"}, {});
  resolver_.AddPair(diagnostic_policy_, "a", "b", {});
  const CollisionFilterGroupsImpl<std::string> actual_groups =
      ConvertInstancedNamesToStrings(resolver_.Resolve(diagnostic_policy_),
                                     plant_);

  CollisionFilterGroupsImpl<std::string> expected_groups;
  expected_groups.AddGroup("a", {"r1::body1"});
  expected_groups.AddGroup("b", {"r2::body2"});
  expected_groups.AddGroup("r1::ra", {"r1::body1"});
  expected_groups.AddGroup("r2::rb", {"r2::body2"});
  expected_groups.AddExclusionPair({"a", "b"});
  EXPECT_EQ(actual_groups, expected_groups);
  EXPECT_TRUE(inspector_.CollisionFiltered(b1, b2));
}

TEST_F(CollisionFilterGroupResolverTest, LinkScoped) {
  // Ensure body names can be resolved in various model scopes.
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  GeometryId ra = AddBody("abody", r1);
  ModelInstanceIndex sub = plant_.AddModelInstance("r1::sub");
  GeometryId rsa = AddBody("abody", sub);

  resolver_.AddGroup(diagnostic_policy_, "a", {"abody", "sub::abody"}, {}, r1);
  resolver_.AddPair(diagnostic_policy_, "a", "a", r1);
  const CollisionFilterGroupsImpl<std::string> actual_groups =
      ConvertInstancedNamesToStrings(resolver_.Resolve(diagnostic_policy_),
                                     plant_);

  CollisionFilterGroupsImpl<std::string> expected_groups;
  expected_groups.AddGroup("r1::a", {"r1::abody", "r1::sub::abody"});
  expected_groups.AddExclusionPair({"r1::a", "r1::a"});
  EXPECT_EQ(actual_groups, expected_groups);
  EXPECT_TRUE(inspector_.CollisionFiltered(ra, rsa));
}

// As a convenience, the tests below reuse body names as group names; each
// group contains the correspondingly named body. Note, however, that group
// names and body names are separate kinds; they never actually mix in the
// resolution process.

TEST_F(CollisionFilterGroupResolverTest, MemberGroupCycle) {
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  const int length = 100;
  std::vector<GeometryId> geoms;
  auto body_of = [](int k) {
    return fmt::format("body{}", k);
  };
  for (int k = 0; k < length; ++k) {
    std::string body_k = body_of(k);
    geoms.push_back(AddBody(body_k, r1));
    resolver_.AddGroup(diagnostic_policy_, body_k, {body_k},
                       {body_of((k + 1) % length)}, r1);
  }
  // Create a self-exclusion rule for testing. It doesn't matter which group
  // name we pick, any group in the cycle will end up with all the member
  // bodies.
  std::string test_pair_name = body_of(length - 1);
  resolver_.AddPair(diagnostic_policy_, test_pair_name, test_pair_name, r1);
  const CollisionFilterGroupsImpl<std::string> actual_groups =
      ConvertInstancedNamesToStrings(resolver_.Resolve(diagnostic_policy_),
                                     plant_);

  // Do an exhaustive check of the group output.
  CollisionFilterGroupsImpl<std::string> expected_groups;
  std::set<std::string> all_bodies;
  for (int k = 0; k < length; ++k) {
    all_bodies.insert(fmt::format("r1::{}", body_of(k)));
  }
  for (int k = 0; k < length; ++k) {
    expected_groups.AddGroup(fmt::format("r1::{}", body_of(k)), all_bodies);
  }
  expected_groups.AddExclusionPair({"r1::body99", "r1::body99"});
  EXPECT_EQ(actual_groups, expected_groups);

  // Spot-check the filtered collisions.
  EXPECT_TRUE(inspector_.CollisionFiltered(geoms.front(), geoms.back()));
  EXPECT_TRUE(
      inspector_.CollisionFiltered(geoms.front(), geoms[geoms.size() / 2]));
}

TEST_F(CollisionFilterGroupResolverTest, MemberGroupDeepNest) {
  ModelInstanceIndex r1 = plant_.AddModelInstance("r1");
  const int depth = 100;
  std::vector<GeometryId> geoms;
  auto body_of = [](int k) {
    return fmt::format("body{}", k);
  };
  for (int k = 0; k < depth; ++k) {
    std::string body_k = body_of(k);
    geoms.push_back(AddBody(body_k, r1));
    if (k + 1 >= depth) {
      resolver_.AddGroup(diagnostic_policy_, body_k, {body_k}, {}, r1);
    } else {
      resolver_.AddGroup(diagnostic_policy_, body_k, {body_k}, {body_of(k + 1)},
                         r1);
    }
  }
  // Create a self-exclusion rule for testing. The group containing body0 is
  // "top", and will contain all of the bodies and geometries.
  resolver_.AddPair(diagnostic_policy_, body_of(0), body_of(0), r1);
  const CollisionFilterGroupsImpl<std::string> actual_groups =
      ConvertInstancedNamesToStrings(resolver_.Resolve(diagnostic_policy_),
                                     plant_);

  // Do an exhaustive check of the group output.
  CollisionFilterGroupsImpl<std::string> expected_groups;
  for (int k = 0; k < depth; ++k) {
    std::set<std::string> bodies;
    for (int m = k; m < depth; ++m) {
      bodies.insert(fmt::format("r1::{}", body_of(m)));
    }
    expected_groups.AddGroup(fmt::format("r1::{}", body_of(k)), bodies);
  }
  expected_groups.AddExclusionPair({"r1::body0", "r1::body0"});
  EXPECT_EQ(actual_groups, expected_groups);

  // Spot-check the filtered collisions.
  EXPECT_TRUE(inspector_.CollisionFiltered(geoms.front(), geoms.back()));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
