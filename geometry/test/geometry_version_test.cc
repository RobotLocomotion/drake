#include "drake/geometry/geometry_version.h"

#include <gtest/gtest.h>

#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
class GeometryVersionTest : public ::testing::Test {
 protected:
  void SetUp() {}
  GeometryVersion version_;

  void increment_proximity(GeometryVersion* v) { v->increment_proximity(); }
  void increment_perception(GeometryVersion* v) { v->increment_perception(); }
  void increment_illustration(GeometryVersion* v) {
    v->increment_illustration();
  }

  GeometryVersion CreateGeometryVersion() const { return GeometryVersion(); }

  void VerifySameRole(const GeometryVersion& v1, const GeometryVersion& v2,
                      Role role) const {
    EXPECT_TRUE(v1.SameVersionAs(v2, role));
    // Verify commutativity.
    EXPECT_TRUE(v2.SameVersionAs(v1, role));
  }

  void VerifyDistinctRole(const GeometryVersion& v1, const GeometryVersion& v2,
                          Role role) const {
    EXPECT_FALSE(v1.SameVersionAs(v2, role));
    // Verify commutativity.
    EXPECT_FALSE(v2.SameVersionAs(v1, role));
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
  increment_proximity(&version_);
  const auto& new_version = version_;
  VerifyDistinctRole(old_version, new_version, Role::kProximity);
  VerifySameRole(old_version, new_version, Role::kPerception);
  VerifySameRole(old_version, new_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Perception) {
  GeometryVersion old_version = version_;
  increment_perception(&version_);
  const auto& new_version = version_;
  VerifySameRole(old_version, new_version, Role::kProximity);
  VerifyDistinctRole(old_version, new_version, Role::kPerception);
  VerifySameRole(old_version, new_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Illustration) {
  GeometryVersion old_version = version_;
  increment_illustration(&version_);
  const auto& new_version = version_;
  VerifySameRole(old_version, new_version, Role::kProximity);
  VerifySameRole(old_version, new_version, Role::kPerception);
  VerifyDistinctRole(old_version, new_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Unassigned) {
  GeometryVersion old_version = version_;
  const auto& new_version = version_;
  EXPECT_THROW(old_version.SameVersionAs(new_version, Role::kUnassigned),
               std::logic_error);
}

TEST_F(GeometryVersionTest, Assignment) {
  GeometryVersion copied_version = version_;
  VerifyIdenticalVersions(copied_version, version_);
  increment_illustration(&version_);
  // The version numbers in the copy move independently of the original ones.
  VerifySameRole(version_, copied_version, Role::kProximity);
  VerifySameRole(version_, copied_version, Role::kPerception);
  VerifyDistinctRole(version_, copied_version, Role::kIllustration);
}

TEST_F(GeometryVersionTest, CopyConstruct) {
  // Copy constructed version should be identical to the original version.
  GeometryVersion copied_version(version_);
  VerifyIdenticalVersions(copied_version, version_);
  increment_illustration(&version_);
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
  increment_perception(&v1);
  increment_proximity(&v2);
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
  increment_perception(&version_);
  increment_proximity(&v1);
  VerifyDistinctRole(v1, version_, Role::kProximity);
  VerifyDistinctRole(v1, version_, Role::kPerception);
  // The version values of the roles that remain unmodified since the child
  // was constructed from the parent remain equal to those of the parent.
  VerifySameRole(v1, version_, Role::kIllustration);
}

// Repeat the Parent Test with a Parent that has been modified at the time
// child is created.
TEST_F(GeometryVersionTest, ModifiedParent) {
  increment_proximity(&version_);
  increment_perception(&version_);
  increment_illustration(&version_);
  // Child should be identical to parent at time when it's created.
  GeometryVersion v1(version_);
  VerifyIdenticalVersions(v1, version_);
  // They then evolve independently for each role.
  increment_perception(&version_);
  increment_proximity(&v1);
  VerifyDistinctRole(v1, version_, Role::kProximity);
  VerifyDistinctRole(v1, version_, Role::kPerception);
  // The version values of the roles that remain unmodified since the child
  // was constructed from the parent remain equal to those of the parent.
  VerifySameRole(v1, version_, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Grandparent) {
  // Child is identical to its grandparent if its parent was unmodified when
  // the child was created.
  GeometryVersion v1(version_);
  GeometryVersion v2(v1);
  VerifyIdenticalVersions(v2, version_);
  // Child remains identical to its grandparent if its parent is modified
  // after the child was created.
  increment_proximity(&v1);
  VerifyIdenticalVersions(v2, version_);

  // Child and grandparent then evolve independently for each role.
  increment_perception(&version_);
  increment_proximity(&v2);
  VerifyDistinctRole(v2, version_, Role::kProximity);
  VerifyDistinctRole(v2, version_, Role::kPerception);
  // The version values of the roles that remain unmodified since the child
  // was created remain equal to those of the grandparent.
  VerifySameRole(v2, version_, Role::kIllustration);
}

// Repeat the Grandparent Test with a Grandparent that has been modified at
// the time parent and child are created.
TEST_F(GeometryVersionTest, ModifiedGrandparent) {
  increment_proximity(&version_);
  increment_perception(&version_);
  increment_illustration(&version_);
  // Child is identical to its grandparent if its parent was unmodified when
  // the child was created.
  GeometryVersion v1(version_);
  GeometryVersion v2(v1);
  VerifyIdenticalVersions(v2, version_);
  // Child remains identical to its grandparent if its parent is modified
  // after the child was created.
  increment_proximity(&v1);
  VerifyIdenticalVersions(v2, version_);

  // Child and grandparent then evolve independently for each role.
  increment_perception(&version_);
  increment_proximity(&v2);
  VerifyDistinctRole(v2, version_, Role::kProximity);
  VerifyDistinctRole(v2, version_, Role::kPerception);
  // The version values of the roles that remain unmodified since the child
  // was created remain equal to those of the grandparent.
  VerifySameRole(v2, version_, Role::kIllustration);
}

TEST_F(GeometryVersionTest, Cousins) {
  // Cousins are identical to each other if their parent haven't been modified
  // at the time they (the cousins) are created.
  GeometryVersion v1(version_);
  GeometryVersion v2(v1);
  GeometryVersion v3(version_);
  GeometryVersion v4(v3);
  VerifyIdenticalVersions(v2, v4);
  // Cousins remain identical to each other if their parents are modified
  // after they (the cousins) were created.
  increment_proximity(&v1);
  increment_perception(&v3);
  VerifyIdenticalVersions(v2, v4);

  // Cousins then evolve independently for each role.
  increment_perception(&v2);
  increment_proximity(&v4);
  VerifyDistinctRole(v2, v4, Role::kProximity);
  VerifyDistinctRole(v2, v4, Role::kPerception);
  // The version values of the roles that remain unmodified since the cousins
  // were created remain equal.
  VerifySameRole(v2, v4, Role::kIllustration);
}
}  // namespace
}  // namespace geometry
}  // namespace drake
