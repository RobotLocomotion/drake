#include "drake/geometry/geometry_version.h"

#include <gtest/gtest.h>

#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
class GeometryVersionTest : public ::testing::Test {
 protected:
  void SetUp() {}
  GeometryVersion version_;

  void modify_proximity(GeometryVersion* v) { v->modify_proximity(); }
  void modify_perception(GeometryVersion* v) { v->modify_perception(); }
  void modify_illustration(GeometryVersion* v) { v->modify_illustration(); }

  GeometryVersion CreateGeometryVersion() const { return GeometryVersion(); }

  void VerifySameRole(const GeometryVersion& v1, const GeometryVersion& v2,
                      Role role) const {
    EXPECT_TRUE(v1.IsSameAs(v2, role));
    // Verify commutativity.
    EXPECT_TRUE(v2.IsSameAs(v1, role));
  }

  void VerifyDistinctRole(const GeometryVersion& v1, const GeometryVersion& v2,
                          Role role) const {
    EXPECT_FALSE(v1.IsSameAs(v2, role));
    // Verify commutativity.
    EXPECT_FALSE(v2.IsSameAs(v1, role));
  }

  void VerifyIdenticalVersions(const GeometryVersion& v1,
                               const GeometryVersion& v2) const {
    VerifySameRole(v1, v2, Role::kProximity);
    VerifySameRole(v1, v2, Role::kPerception);
    VerifySameRole(v1, v2, Role::kIllustration);
  }

  void VerifyDistinctVersions(const GeometryVersion& v1,
                              const GeometryVersion& v2) const {
    VerifyDistinctRole(v1, v2, Role::kProximity);
    VerifyDistinctRole(v1, v2, Role::kPerception);
    VerifyDistinctRole(v1, v2, Role::kIllustration);
  }
};

namespace {
TEST_F(GeometryVersionTest, Proximity) {
  GeometryVersion old_version = version_;
  modify_proximity(&version_);
  const auto& new_version = version_;
  VerifyDistinctRole(old_version, new_version, Role::kProximity);
  VerifySameRole(old_version, new_version, Role::kPerception);
  VerifySameRole(old_version, new_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Perception) {
  GeometryVersion old_version = version_;
  modify_perception(&version_);
  const auto& new_version = version_;
  VerifySameRole(old_version, new_version, Role::kProximity);
  VerifyDistinctRole(old_version, new_version, Role::kPerception);
  VerifySameRole(old_version, new_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Illustration) {
  GeometryVersion old_version = version_;
  modify_illustration(&version_);
  const auto& new_version = version_;
  VerifySameRole(old_version, new_version, Role::kProximity);
  VerifySameRole(old_version, new_version, Role::kPerception);
  VerifyDistinctRole(old_version, new_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Unassigned) {
  GeometryVersion old_version = version_;
  const auto& new_version = version_;
  EXPECT_THROW(old_version.IsSameAs(new_version, Role::kUnassigned),
               std::logic_error);
}

TEST_F(GeometryVersionTest, Assignment) {
  GeometryVersion copied_version = version_;
  VerifyIdenticalVersions(copied_version, version_);
  modify_illustration(&version_);
  // The version numbers in the copy move independently of the original ones.
  VerifySameRole(version_, copied_version, Role::kProximity);
  VerifySameRole(version_, copied_version, Role::kPerception);
  VerifyDistinctRole(version_, copied_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, CopyConstruct) {
  // Copy constructed version should be identical to the original version.
  GeometryVersion copied_version(version_);
  VerifyIdenticalVersions(copied_version, version_);
  modify_illustration(&version_);
  // The version numbers in the copy move independently of the original ones.
  VerifySameRole(version_, copied_version, Role::kProximity);
  VerifySameRole(version_, copied_version, Role::kPerception);
  VerifyDistinctRole(version_, copied_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, DefaultConstruct) {
  // Each default constructed version is distinct from each other.
  GeometryVersion new_version = CreateGeometryVersion();
  VerifyDistinctVersions(new_version, version_);
}

TEST_F(GeometryVersionTest, Siblings) {
  // Each version constructed with the same parent is identical to each other.
  GeometryVersion v1(version_);
  GeometryVersion v2(version_);
  VerifyIdenticalVersions(v1, v2);
  // They then evolve independently for each role.
  modify_perception(&v1);
  modify_proximity(&v2);
  VerifyDistinctRole(v1, v2, Role::kProximity);
  VerifyDistinctRole(v1, v2, Role::kPerception);
  // The version values of the roles that remain unmodified since the two
  // versions were constructed from the parent remain equal to each other.
  VerifySameRole(v1, v2, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Parent) {
  // Child should be identical to parent at time when it's created.
  GeometryVersion v1(version_);
  VerifyIdenticalVersions(v1, version_);
  // They then evolve independently for each role.
  modify_perception(&version_);
  modify_proximity(&v1);
  VerifyDistinctRole(v1, version_, Role::kProximity);
  VerifyDistinctRole(v1, version_, Role::kPerception);
  // The version values of the roles that remain unmodified since the child
  // was constructed from the parent remain equal to those of the parent.
  VerifySameRole(v1, version_, Role::kIllustration);
}

TEST_F(GeometryVersionTest, IndependentInstance) {
  // v1 and v2 are equivalent to version_.
  GeometryVersion v1(version_);
  GeometryVersion v2(version_);
  // Modifying v1 does not change equivalence between v2 and version_.
  modify_perception(&v1);
  modify_proximity(&v1);
  modify_illustration(&v1);
  VerifyIdenticalVersions(v2, version_);
}
}  // namespace
}  // namespace geometry
}  // namespace drake
