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

using std::make_pair;
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
                             bool is_invariant) {
    filter->Apply(CollisionFilterDeclaration().ExcludeWithin(GeometrySet(ids)),
                  get_extract_ids_functor(), is_invariant);

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

  static CollisionFilter ClearCopy(const CollisionFilter& filter) {
    return filter.MakeClearCopy();
  }
};

/* Tests that declaration statements that allow collisions between geometries
 in different sets are properly handled. */
TEST_F(CollisionFilterTest, AllowBetween) {
  for (bool is_invariant : {true, false}) {
    CollisionFilter filters;
    auto [id_A, id_B, id_C] = this->InitIds(&filters);

    /* To test Allowing, we have to start with filters. Filter everything. */
    this->FilterAllPairs(&filters, {id_A, id_B, id_C}, is_invariant);

    filters.Apply(CollisionFilterDeclaration().AllowBetween(
                      GeometrySet(id_A), GeometrySet({id_B, id_C})),
                  this->get_extract_ids_functor());
    /* Our ability to remove the filter depends on whether it was invariant when
     added. */
    EXPECT_EQ(filters.CanCollideWith(id_A, id_B), !is_invariant);
    EXPECT_EQ(filters.CanCollideWith(id_A, id_C), !is_invariant);
    EXPECT_FALSE(filters.CanCollideWith(id_B, id_C));
  }
}

/* Tests that declaration statements that allow collisions between geometries
 in a single set are properly handled. */
TEST_F(CollisionFilterTest, AllowWithin) {
  for (bool is_invariant : {true, false}) {
    CollisionFilter filters;
    auto [id_A, id_B, id_C] = this->InitIds(&filters);

    /* To test Allowing, we have to start with filters. Filter everything. */
    this->FilterAllPairs(&filters, {id_A, id_B, id_C}, is_invariant);

    filters.Apply(CollisionFilterDeclaration()
                      .AllowWithin(GeometrySet({id_A, id_B}))
                      .AllowWithin(GeometrySet({id_A, id_C})),
                  this->get_extract_ids_functor());
    /* Our ability to remove the filter depends on whether it was invariant when
     added. */
    EXPECT_EQ(filters.CanCollideWith(id_A, id_B), !is_invariant);
    EXPECT_EQ(filters.CanCollideWith(id_A, id_C), !is_invariant);
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

/* Exercise transient declaration API. This entails five different methods that
 are all deeply entangled, so we'll have to test them together. For each
 function, we want to test certain properties (as listed below):
  A ApplyTransient()
    1 Can't remove invariant filters.
    2 Can add filters.
    3 Can remove filters.
  B has_transient_history()
    1 No history reports false.
    2 History reports true.
  C IsActive()
    1 No active history implies no ids report as active.
    2 Ids in active set report true.
    3 Ids not in active set report false.
  D RemoveDeclaration()
    1 Removing an invalid id is a no-op.
    2 Removing the last declaration returns us to the previous state.
    3 Removing an intermediate declaration has playback.
  E Flatten()
    1 Flattening an "only-persitent" filter state makes no difference.
    2 Any previous ids given are no longer valid.
    3 Reports no active history.
    4 Flatten preserves resultant filter state (i.e., the set of filtered
      pairs is same before and after flattening).

  In addition, to those *specific* methods, there are aspects of the other
  methods that interact with transient history:

   F Add geometry to system with transient history.
   G Remove geometry from system with transient history.
   H Two collision filters with the same *end* result are equal, even if their
     history is different.

  The testing methodology is a sequence of operations that will give us the
  chance to test all of the properties above. Comments will reference which
  property is under test. */
TEST_F(CollisionFilterTest, TransientDeclarations) {
  CollisionFilter filter;

  /* Prop. C.1: No history -> not active. */
  EXPECT_FALSE(filter.IsActive(FilterId::get_new_id()));
  /* Prop. B.1: No history -> reports false. */
  EXPECT_FALSE(filter.has_transient_history());

  /* Five geometry ids give us ten possible pairs to play with. */
  auto new_id = [&filter]() {
    const GeometryId id = GeometryId::get_new_id();
    filter.AddGeometry(id);
    return id;
  };
  const GeometryId A = new_id();
  const GeometryId B = new_id();
  const GeometryId C = new_id();
  const GeometryId D = new_id();
  const GeometryId E = new_id();

  const bool invariant = true;

  /* We'll track the expected pairs by adding them and removing them from this
   collection. */
  using Pair = std::pair<GeometryId, GeometryId>;
  using Decl = CollisionFilterDeclaration;

  /* Prop. H: This test encodes expected filters into persistent base
   configuration and compares it against a filter with arbitrary history.
   The parameter `expected` should be *all* the geometry pairs that the filter
   `f` would report as filtered. */
  auto is_expected =
      [this](const CollisionFilter& f,
             const std::set<Pair>& expected) -> ::testing::AssertionResult {
    CollisionFilter temp_filter = this->ClearCopy(f);
    for (const auto& pair : expected) {
      temp_filter.Apply(
          Decl().ExcludeWithin(GeometrySet({pair.first, pair.second})),
          this->get_extract_ids_functor(), !invariant);
    }
    if (temp_filter != f) return ::testing::AssertionFailure();
    return ::testing::AssertionSuccess();
  };

  std::set<Pair> expected_filter_pairs;
  const CollisionFilter::ExtractIds& extract = this->get_extract_ids_functor();

  /* Set up the persistent base with pairs
    P(D, E), (B, C), (B, D), (B, E)
    * P(i, j) denotes an *invariant* pair between i and j. */
  filter.Apply(Decl().ExcludeWithin(GeometrySet{D, E}), extract, invariant);
  filter.Apply(Decl().ExcludeBetween(GeometrySet{B}, GeometrySet{C, D, E}),
               extract, !invariant);
  expected_filter_pairs.emplace(make_pair(D, E));
  expected_filter_pairs.emplace(make_pair(B, C));
  expected_filter_pairs.emplace(make_pair(B, D));
  expected_filter_pairs.emplace(make_pair(B, E));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop. H: Add a redundant declaration, so now we *know* we're testing a
   filter *with* history against one without. Filtered pairs are still:
    P(D, E), (B, C), (B, D), (B, E) */
  const FilterId id0 =
      filter.ApplyTransient(CollisionFilterDeclaration().ExcludeBetween(
                                GeometrySet{B}, GeometrySet{{C, D, E}}),
                            this->get_extract_ids_functor());
  EXPECT_TRUE(is_expected(filter, expected_filter_pairs));
  /* Prop. B.2.: history -> true */
  EXPECT_TRUE(filter.has_transient_history());
  /* Prop. C.2: Has history, query valid id -> active. */
  EXPECT_TRUE(filter.IsActive(id0));
  /* Prop. C.3: Has history, query invalid id -> not active. */
  EXPECT_FALSE(filter.IsActive(FilterId::get_new_id()));

  /* Prop. A.2 and A.3: Add new filter (C, D), remove old, non-invariant
   filter (B, D). Filtered pairs become:
    P(D, E), (B, C), (B, E), (C, D) */
  const FilterId id1 = filter.ApplyTransient(
      Decl().ExcludeWithin(GeometrySet{C, D}).AllowWithin(GeometrySet{B, D}),
      extract);
  /* The new declaration is active. */
  EXPECT_TRUE(filter.IsActive(id1));
  /* The old declaration, although supplanted, is also active. */
  EXPECT_TRUE(filter.IsActive(id0));
  expected_filter_pairs.emplace(make_pair(C, D));
  expected_filter_pairs.erase(expected_filter_pairs.find(make_pair(B, D)));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* This declaration will be used twice. We'll add and then remove it (with
   this as the last declaration to test D.2), and then use it again to test D.3.
   */
  /* We'll save the state at id 1 as we'll be returning to it. */
  const CollisionFilter filter_as_of_id1(filter);
  Decl add_2_clear_2;
  add_2_clear_2.ExcludeBetween(GeometrySet(A), GeometrySet{B, C})
      .AllowBetween(GeometrySet(B), GeometrySet{C, E});

  /* Prop. C.2 and C.3 again: adding and removing filters. More complex. The
   state becomes:
    P(D, E), (A, B), (A, C), (C, D) */
  const FilterId id2 = filter.ApplyTransient(add_2_clear_2, extract);
  EXPECT_TRUE(filter.IsActive(id2));
  expected_filter_pairs.emplace(make_pair(A, B));
  expected_filter_pairs.emplace(make_pair(A, C));
  expected_filter_pairs.erase(expected_filter_pairs.find(make_pair(B, C)));
  expected_filter_pairs.erase(expected_filter_pairs.find(make_pair(B, E)));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop. D.1: Removing invalid id does nothing. */
  EXPECT_FALSE(filter.RemoveDeclaration(FilterId::get_new_id()));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop. D.2: Removing last declaration returns to previous state:
    P(D, E), (B, C), (B, E), (C, D) */
  EXPECT_TRUE(filter.RemoveDeclaration(id2));
  ASSERT_TRUE(filter == filter_as_of_id1);

  /* Re-apply the declaration (add_2_clear_2) to become:
    P(D, E), (A, B), (A, C), (C, D) */
  const FilterId id3 = filter.ApplyTransient(add_2_clear_2, extract);
  EXPECT_TRUE(filter.IsActive(id3));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Add a further declaration that is *partially* redundant w.r.t. the previous
   declaration. We'll continue to filter (A, B) and continue to allow (B, E):
    P(D, E), (A, B), (A, C), (C, D) */
  const FilterId id4 = filter.ApplyTransient(
      Decl().ExcludeWithin(GeometrySet{A, B}).AllowWithin(GeometrySet{B, E}),
      extract);
  EXPECT_TRUE(filter.IsActive(id4));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop. D.3: Removing id 3 will only half undo its work. id 4 keeps one
   filter and one removal:
    P(D, E), (A, B), (B, C), (C, D) */
  EXPECT_TRUE(filter.RemoveDeclaration(id3));
  expected_filter_pairs.erase(expected_filter_pairs.find(make_pair(A, C)));
  expected_filter_pairs.emplace(make_pair(B, C));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop. A.1: Attempting to remove invariant filter makes no difference:
    P(D, E), (A, B), (B, C), (C, D) */
  const FilterId id5 =
      filter.ApplyTransient(Decl().AllowWithin(GeometrySet{D, E}), extract);
  EXPECT_TRUE(filter.IsActive(id5));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop. F: Adding a geometry to filter with history is allowed and preserves
   filters:
    P(D, E), (A, B), (B, C), (C, D) */
  const GeometryId F = GeometryId::get_new_id();
  filter.AddGeometry(F);
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop. G: Removing a geometry does the right thing. We'll remove a geometry
   with filters to see the impact:
    P(D, E), (B, C), (C, D) */
  filter.RemoveGeometry(A);
  expected_filter_pairs.erase(expected_filter_pairs.find(make_pair(A, B)));
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop E: flattening history. Before flattening, we'll confirm valid filter
   ids and active history. */
  ASSERT_TRUE(filter.has_transient_history());
  for (FilterId id : {id0, id1, id4, id5}) ASSERT_TRUE(filter.IsActive(id));

  filter.Flatten();

  /* Prop. E.1: Flatten -> unchanged resultant filter state. */
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));

  /* Prop. E.2: Flatten -> reports no active history. */
  EXPECT_FALSE(filter.has_transient_history());

  /* Prop. E.3: Flatten -> Invalid ids. */
  for (FilterId id : {id0, id1, id4, id5}) EXPECT_FALSE(filter.IsActive(id));

  /* Prop. E.4: Flatten flat history makes no difference. */
  EXPECT_NO_THROW(filter.Flatten());
  ASSERT_TRUE(is_expected(filter, expected_filter_pairs));
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
