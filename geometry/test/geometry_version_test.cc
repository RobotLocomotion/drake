#include "drake/geometry/geometry_version.h"

#include <gtest/gtest.h>

#include "drake/geometry/geometry_roles.h"

namespace drake {
namespace geometry {
/* GeometryVersionTest tests the following properties of GeometryVersion:
  1. Role versions within a GeometryVersion are independent from each other.
  2. Copies of GeometryVersion are equivalent immediately after the copy.
  3. The equivalence in the second property is transitive upon copying.
  4. Copies of GeometryVersion then are modified independently after the copy.
  5. Each GeometryVersion created from scratch is unique. */
class GeometryVersionTest : public ::testing::Test {
 protected:
  void SetUp() {}

  void modify_proximity(GeometryVersion* v) { v->modify_proximity(); }
  void modify_perception(GeometryVersion* v) { v->modify_perception(); }
  void modify_illustration(GeometryVersion* v) { v->modify_illustration(); }

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

  GeometryVersion version_;
};

namespace {
// Tests property 1.
TEST_F(GeometryVersionTest, Proximity) {
  GeometryVersion old_version = version_;
  modify_proximity(&version_);
  const auto& new_version = version_;
  VerifyDistinctRole(old_version, new_version, Role::kProximity);
  VerifySameRole(old_version, new_version, Role::kPerception);
  VerifySameRole(old_version, new_version, Role::kIllustration);
}

// Tests property 1.
TEST_F(GeometryVersionTest, Perception) {
  GeometryVersion old_version = version_;
  modify_perception(&version_);
  const auto& new_version = version_;
  VerifySameRole(old_version, new_version, Role::kProximity);
  VerifyDistinctRole(old_version, new_version, Role::kPerception);
  VerifySameRole(old_version, new_version, Role::kIllustration);
}

// Tests property 1.
TEST_F(GeometryVersionTest, Illustration) {
  GeometryVersion old_version = version_;
  modify_illustration(&version_);
  const auto& new_version = version_;
  VerifySameRole(old_version, new_version, Role::kProximity);
  VerifySameRole(old_version, new_version, Role::kPerception);
  VerifyDistinctRole(old_version, new_version, Role::kIllustration);
}

// Test for unsupported role.
TEST_F(GeometryVersionTest, Unassigned) {
  GeometryVersion old_version = version_;
  const auto& new_version = version_;
  EXPECT_THROW(old_version.IsSameAs(new_version, Role::kUnassigned),
               std::logic_error);
}

// Tests properties 2 and 4 for assignment.
TEST_F(GeometryVersionTest, Assignment) {
  GeometryVersion copied_version = version_;
  VerifyIdenticalVersions(copied_version, version_);
  modify_illustration(&version_);
  // The role versions in the copy move independently of the original ones.
  VerifySameRole(version_, copied_version, Role::kProximity);
  VerifySameRole(version_, copied_version, Role::kPerception);
  VerifyDistinctRole(version_, copied_version, Role::kIllustration);
}

// Tests properties 2 and 4 for copy constructor.
TEST_F(GeometryVersionTest, CopyConstruct) {
  // Copy constructed version should be equivalent to the original version.
  GeometryVersion copied_version(version_);
  VerifyIdenticalVersions(copied_version, version_);
  modify_illustration(&version_);
  // The role versions in the copy move independently of the original ones.
  VerifySameRole(version_, copied_version, Role::kProximity);
  VerifySameRole(version_, copied_version, Role::kPerception);
  VerifyDistinctRole(version_, copied_version, Role::kIllustration);
}

// Tests properties 3 and 4.
TEST_F(GeometryVersionTest, Transitivity) {
  GeometryVersion v1(version_);
  GeometryVersion v2(version_);
  // The equivalence is transitive at the time of the copy.
  VerifyIdenticalVersions(v1, v2);
  // The copies then evolve independently for each role.
  modify_perception(&v1);
  modify_proximity(&v2);
  VerifyDistinctRole(v1, v2, Role::kProximity);
  VerifyDistinctRole(v1, v2, Role::kPerception);
  // The illustration role version remains unmodified since the two
  // versions were copied from the parent.
  VerifySameRole(v1, v2, Role::kIllustration);
}

// Tests property 5.
TEST_F(GeometryVersionTest, DefaultConstruct) {
  // Each default constructed version is unique.
  GeometryVersion new_version;
  VerifyDistinctVersions(new_version, version_);
}
}  // namespace
}  // namespace geometry
}  // namespace drake
