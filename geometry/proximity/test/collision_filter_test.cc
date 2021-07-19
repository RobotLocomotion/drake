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

constexpr bool kCanCollide = true;
/* Helper function to test the invariance of CollisionFilter::CanCollideWith()
 to the geometry id order. */
::testing::AssertionResult ExpectCanCollide(const CollisionFilter& filter,
                                            GeometryId id_A, GeometryId id_B,
                                            bool expect_collidable) {
  const bool result1 = filter.CanCollideWith(id_A, id_B);
  const bool result2 = filter.CanCollideWith(id_B, id_A);
  if (result1 != result2) {
    return ::testing::AssertionFailure()
           << "The collision filter state for " << id_A << " and " << id_B
           << " changed based on ordering";
  }
  if (result1 != expect_collidable) {
    return ::testing::AssertionFailure()
           << "For pair (" << id_A << ", " << id_B
           << "), our expectation for can-collide was " << expect_collidable
           << ", but the filter reported " << result1;
  }
  return ::testing::AssertionSuccess();
}

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
      EXPECT_TRUE(filter->HasGeometry(id));
    }
    return std::make_tuple(ids[0], ids[1], ids[2]);
  }

  /* Apply collision filters between geometries in the list. */
  static void FilterAllPairs(CollisionFilter* filter,
                             std::initializer_list<GeometryId> ids,
                             bool is_permanent) {
    filter->Apply(CollisionFilterDeclaration().ExcludeWithin(GeometrySet(ids)),
                  get_extract_ids_functor(), is_permanent);

    for (GeometryId id_A : ids) {
      for (GeometryId id_B : ids) {
        ASSERT_TRUE(ExpectCanCollide(*filter, id_A, id_B, !kCanCollide));
      }
    }
  }

  /* Returns a value for the collision filter id extraction functor. */
  static CollisionFilter::ExtractIds get_extract_ids_functor() {
    return &GeometrySetTester::geometries;
  }
};

/* Tests that declaration statements that allow collisions between geometries
 in different sets are properly handled. */
TEST_F(CollisionFilterTest, AllowBetween) {
  for (bool is_permanent : {true, false}) {
    CollisionFilter filters;
    auto [id_A, id_B, id_C] = this->InitIds(&filters);

    /* To test Allowing, we have to start with filters. Filter everything. */
    this->FilterAllPairs(&filters, {id_A, id_B, id_C}, is_permanent);

    filters.Apply(CollisionFilterDeclaration().AllowBetween(
                      GeometrySet(id_A), GeometrySet({id_B, id_C})),
                  this->get_extract_ids_functor());
    /* Our ability to remove the filter depends on whether it was permanent when
     added. */
    EXPECT_EQ(filters.CanCollideWith(id_A, id_B), !is_permanent);
    EXPECT_EQ(filters.CanCollideWith(id_A, id_C), !is_permanent);
    EXPECT_FALSE(filters.CanCollideWith(id_B, id_C));
  }
}

/* Tests that declaration statements that allow collisions between geometries
 in a single set are properly handled. */
TEST_F(CollisionFilterTest, AllowWithin) {
  for (bool is_permanent : {true, false}) {
    CollisionFilter filters;
    auto [id_A, id_B, id_C] = this->InitIds(&filters);

    /* To test Allowing, we have to start with filters. Filter everything. */
    this->FilterAllPairs(&filters, {id_A, id_B, id_C}, is_permanent);

    filters.Apply(CollisionFilterDeclaration()
                      .AllowWithin(GeometrySet({id_A, id_B}))
                      .AllowWithin(GeometrySet({id_A, id_C})),
                  this->get_extract_ids_functor());
    /* Our ability to remove the filter depends on whether it was permanent when
     added. */
    EXPECT_EQ(filters.CanCollideWith(id_A, id_B), !is_permanent);
    EXPECT_EQ(filters.CanCollideWith(id_A, id_C), !is_permanent);
    EXPECT_FALSE(filters.CanCollideWith(id_B, id_C));
  }
}

/* Tests that declaration statements that exclude collisions between geometries
 in different sets are properly handled. */
TEST_F(CollisionFilterTest, ExcludeBetween) {
  CollisionFilter filters;
  auto [id_A, id_B, id_C] = this->InitIds(&filters);

  /* Initial condition: everything can collide. */
  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_B, kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_C, kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_B, id_C, kCanCollide));

  filters.Apply(CollisionFilterDeclaration().ExcludeBetween(
                    GeometrySet(id_A), GeometrySet({id_B, id_C})),
                this->get_extract_ids_functor());

  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_B, !kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_C, !kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_B, id_C, kCanCollide));
}

/* Tests that declaration statements that exclude collisions between geometries
 in a single set are properly handled. */
TEST_F(CollisionFilterTest, ExcludeWithin) {
  CollisionFilter filters;
  auto [id_A, id_B, id_C] = this->InitIds(&filters);
  const GeometryId id_D = GeometryId::get_new_id();
  filters.AddGeometry(id_D);

  /* Initial condition: everything can collide. */
  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_B, kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_C, kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_D, kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_B, id_C, kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_B, id_D, kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_C, id_D, kCanCollide));

  filters.Apply(CollisionFilterDeclaration()
                    .ExcludeWithin(GeometrySet({id_A, id_B, id_D}))
                    .ExcludeWithin(GeometrySet({id_A, id_C})),
                this->get_extract_ids_functor());

  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_B, !kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_C, !kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_A, id_D, !kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_B, id_C, kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_B, id_D, !kCanCollide));
  EXPECT_TRUE(ExpectCanCollide(filters, id_C, id_D, kCanCollide));
}

/* Simply confirms that regardless of efforts, a geometry will never report that
 it can collide with itself. */
TEST_F(CollisionFilterTest, NoSelfCollision) {
  CollisionFilter filters;
  auto [id_A, id_B, id_C] = this->InitIds(&filters);

  /* Initial condition: no self collision. */
  EXPECT_FALSE(filters.CanCollideWith(id_A, id_A));
  EXPECT_FALSE(filters.CanCollideWith(id_B, id_B));
  EXPECT_FALSE(filters.CanCollideWith(id_C, id_C));

  // TODO(SeanCurtis-TRI): When we add filter *removal* confirm that attempts to
  // add pair (A, A) back into candidate set C doesn't work.
}

/* In this test, we're confirming the logic of operator== and operator!=. As
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
      this->get_extract_ids_functor());
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
  filters2.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_A, id_B})),
      this->get_extract_ids_functor());
  EXPECT_TRUE(filters1 == filters2);
  EXPECT_FALSE(filters1 != filters2);
  filters1.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_A, id_C})),
      this->get_extract_ids_functor());
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
  filters2.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_B, id_C})),
      this->get_extract_ids_functor());
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
  filters1.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_B, id_C})),
      this->get_extract_ids_functor());
  filters2.Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet({id_A, id_C})),
      this->get_extract_ids_functor());
  EXPECT_TRUE(filters1 == filters2);
  EXPECT_FALSE(filters1 != filters2);
  filters1.Apply(
      CollisionFilterDeclaration().AllowWithin(GeometrySet({id_A, id_C})),
      this->get_extract_ids_functor());
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);

  /* Neither set has filters between (A, *). The first simply doesn't have A,
   the second has A, but no filters. They should *not* be considered equal. */
  filters1.RemoveGeometry(id_A);
  filters2.Apply(CollisionFilterDeclaration().AllowBetween(
                     GeometrySet(id_A), GeometrySet({id_B, id_C})),
                 this->get_extract_ids_functor());
  EXPECT_FALSE(filters1 == filters2);
  EXPECT_TRUE(filters1 != filters2);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
