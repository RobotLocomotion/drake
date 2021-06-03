#include "drake/geometry/proximity/collision_filter.h"

#include <tuple>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {

/* Use GeometrySetTester's friend status with GeometrySet to leak its geometry
 ids to support the tests below. */
class GeometrySetTester {
 public:
  static std::unordered_set<GeometryId> geometries(const GeometrySet& s) {
    return s.geometries();
  }
};

namespace internal {

using std::vector;

/* For all of these tests, we use GeometrySets that only consist of GeometryIds.
 This way we don't need a *real* implementation of CollisionFilter::ExtractIds.
 For our purposes, it's simply a case of extracting the geometries. In the real
 world, we'd receive a different callback and we rely on unit tests of that
 callback to be tested elsewhere.  */
class CollisionFilterTest : public ::testing::Test {
 protected:
  /* Populate the given `filter` with three ids, and then return the ids. */
  static std::tuple<GeometryId, GeometryId, GeometryId> InitIds(
      CollisionFilter* filter) {
    vector<GeometryId> ids{GeometryId::get_new_id(), GeometryId::get_new_id(),
                           GeometryId::get_new_id()};
    for (auto id : ids) {
      filter->AddGeometry(id);
      EXPECT_TRUE(HasGeometry(*filter, id));
    }
    return std::make_tuple(ids[0], ids[1], ids[2]);
  }

  /* Apply collision filters between geometries in the list. */
  static void FilterAllPairs(CollisionFilter* filter,
                             std::initializer_list<GeometryId> ids) {
    filter->Apply(CollisionFilterDeclaration().ExcludeWithin(GeometrySet(ids)),
                  extract_ids());

    for (GeometryId id_A : ids) {
      for (GeometryId id_B : ids) {
        /* We'll do the test twice; confirm order doesn't matter. */
        ASSERT_FALSE(filter->CanCollideWith(id_A, id_B));
        ASSERT_FALSE(filter->CanCollideWith(id_B, id_A));
      }
    }
  }

  /* Reports if `filter` has had the given `id` added to it. */
  static bool HasGeometry(const CollisionFilter& filter, GeometryId id) {
    return filter.HasGeometry(id);
  }

  /* Returns a value for the collision filter id extraction functor. */
  static CollisionFilter::ExtractIds extract_ids() {
    return &GeometrySetTester::geometries;
  }
};

/* Tests that declaration statements that exclude collisions between geometries
 in different sets are properly handled. */
TEST_F(CollisionFilterTest, ExcludeBetween) {
  CollisionFilter filters;
  auto [id_A, id_B, id_C] = this->InitIds(&filters);

  /* Initial condition: everything can collide. */
  EXPECT_TRUE(filters.CanCollideWith(id_A, id_B));
  EXPECT_TRUE(filters.CanCollideWith(id_A, id_C));
  EXPECT_TRUE(filters.CanCollideWith(id_B, id_C));

  filters.Apply(CollisionFilterDeclaration().ExcludeBetween(
                    GeometrySet(id_A), GeometrySet({id_B, id_C})),
                this->extract_ids());

  EXPECT_FALSE(filters.CanCollideWith(id_A, id_B));
  EXPECT_FALSE(filters.CanCollideWith(id_A, id_C));
  EXPECT_TRUE(filters.CanCollideWith(id_B, id_C));
}

/* Tests that declaration statements that exclude collisions between geometries
 in a single set are properly handled. */
TEST_F(CollisionFilterTest, ExcludeWithin) {
  CollisionFilter filters;
  auto [id_A, id_B, id_C] = this->InitIds(&filters);

  /* Initial condition: everything can collide. */
  EXPECT_TRUE(filters.CanCollideWith(id_A, id_B));
  EXPECT_TRUE(filters.CanCollideWith(id_A, id_C));
  EXPECT_TRUE(filters.CanCollideWith(id_B, id_C));

  filters.Apply(CollisionFilterDeclaration()
                    .ExcludeWithin(GeometrySet({id_A, id_B}))
                    .ExcludeWithin(GeometrySet({id_A, id_C})),
                this->extract_ids());

  EXPECT_FALSE(filters.CanCollideWith(id_A, id_B));
  EXPECT_FALSE(filters.CanCollideWith(id_A, id_C));
  EXPECT_TRUE(filters.CanCollideWith(id_B, id_C));
}

/* In this test, we're confirming the logic of operator= and operator!=. As
 such, we explicitly call those operators so we don't rely on gtest's
 implementation details. */
TEST_F(CollisionFilterTest, Equality) {
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();
  const GeometryId id_C = GeometryId::get_new_id();

  CollisionFilter filters1;
  CollisionFilter filters2;

  /* Filters with no registered geometries are equal. */
  EXPECT_EQ(filters1, filters2);

  /* Filters with different registered geometries are unequal. */
  /* Disjoint sets of geometries. */
  filters1.AddGeometry(id_A);
  filters2.AddGeometry(id_B);
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
  /* Sets have non-zero intersection, but are still not equal. */
  filters1.AddGeometry(id_C);
  filters2.AddGeometry(id_C);
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
  /* Sets have equal, non-empty sets of registered geometry and *no* filters. */
  filters1.AddGeometry(id_B);
  filters2.AddGeometry(id_A);
  EXPECT_TRUE(filters1 == filters2);
  EXPECT_FALSE(filters1 != filters2);

  /* Various forms of matched/mismatched filters. The correctness of this test
   depends on the order; do not re-order these operations. */
  filters1.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_A, id_B})),
      this->extract_ids());
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
  filters2.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_A, id_B})),
      this->extract_ids());
  EXPECT_TRUE(filters1 == filters2);
  EXPECT_FALSE(filters1 != filters2);
  filters1.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_A, id_C})),
      this->extract_ids());
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
  filters2.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_B, id_C})),
      this->extract_ids());
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
  filters1.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_B, id_C})),
      this->extract_ids());
  filters2.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_A, id_C})),
      this->extract_ids());
  EXPECT_TRUE(filters1 == filters2);
  EXPECT_FALSE(filters1 != filters2);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
