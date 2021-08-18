#include "drake/multibody/plant/contact_results.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {

using std::move;
using std::vector;

namespace internal {
namespace {

/* The class we provide must a) be default constructible and b) default
 constructible in order to use with OwnershipVector. */
class CopyConstructible {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CopyConstructible)

  CopyConstructible() = default;
  explicit CopyConstructible(int value) : value_(value) {}

  int value() const { return value_; }

 private:
  int value_{};
};

bool operator==(const CopyConstructible& a, const CopyConstructible& b) {
  return a.value() == b.value();
}

std::ostream& operator<<(std::ostream& out, const CopyConstructible& c) {
    out << c.value();
    return out;
}

// TODO(SeanCurtis-TRI) Refactor this more intelligibly. As is, there is a
//  great deal to slog through in a single test.
GTEST_TEST(OwnershipVectorTest, EverythingUnderTheSun) {
  const vector<CopyConstructible> source_data{
      CopyConstructible(10), CopyConstructible(20), CopyConstructible(30)};
  const CopyConstructible extra(40);

  /* Create a new copy of OwnershipVector with unowned references to all of the
   entries in source_data. */
  auto make_fresh = [&source_data]() {
    OwnershipVector<CopyConstructible> instances;
    for (const auto& instance : source_data) {
      instances.AddUnowned(&instance);
    }
    return instances;
  };

  /* Confirms that the given OwnershipVector has all of the data in source_data,
   but it doesn't own anything. */
  auto confirm_unowned =
      [&source_data](const OwnershipVector<CopyConstructible>& dut) {
        EXPECT_EQ(dut.size(), static_cast<int>(source_data.size()));
        /* The data is still at the source_data address.. */
        for (int i = 0; i < dut.size(); ++i) {
          EXPECT_EQ(&dut.get(i), &source_data[i]);
        }
      };

  /* Confirms that the OwnershipVector is a unique copy of `source_data`. */
  auto confirm_owned_copy = [&source_data,
                             &extra](OwnershipVector<CopyConstructible>* dut) {
    EXPECT_EQ(dut->size(), static_cast<int>(source_data.size()));
    /* The copy has the same values but different addresses. */
    for (int i = 0; i < dut->size(); ++i) {
      EXPECT_EQ(dut->get(i), source_data[i]);
      EXPECT_NE(&dut->get(i), &source_data[i]);
    }
    /* It can't add any more fields. */
    EXPECT_THROW(dut->AddUnowned(&extra), std::exception);
  };

  auto confirm_unique_copy = [](const OwnershipVector<CopyConstructible>& a,
                                const OwnershipVector<CopyConstructible>& b) {
    EXPECT_EQ(a.size(), b.size());
    /* The copies have the same values but different addresses. */
    for (int i = 0; i < a.size(); ++i) {
      EXPECT_EQ(a.get(i), b.get(i));
      EXPECT_NE(&a.get(i), &b.get(i));
    }
  };

  /* Construct and add unowned instances. */
  OwnershipVector<CopyConstructible> instances = make_fresh();
  confirm_unowned(instances);

  /* Copy construction -- creates unique copy which cannot be extended. */
  OwnershipVector<CopyConstructible> copy_constructed(instances);
  confirm_owned_copy(&copy_constructed);

  OwnershipVector<CopyConstructible> copy_constructed_from_copy(
      copy_constructed);
  confirm_owned_copy(&copy_constructed_from_copy);
  confirm_unique_copy(copy_constructed_from_copy, copy_constructed);

  /* Copy assignment -- old values are lost, creates a unique copy which
   cannot be extended. */
  OwnershipVector<CopyConstructible> copy_assigned;
  copy_assigned.AddUnowned(&extra);
  EXPECT_EQ(copy_assigned.size(), 1);
  EXPECT_EQ(&copy_assigned.get(0), &extra);

  copy_assigned = instances;
  confirm_owned_copy(&copy_assigned);

  OwnershipVector<CopyConstructible> copy_assigned_from_copy;
  copy_assigned_from_copy = copy_constructed;
  confirm_owned_copy(&copy_assigned_from_copy);
  confirm_unique_copy(copy_assigned_from_copy, copy_constructed);

  /* Move construction -- the result has the same semantics as the original. */

  OwnershipVector<CopyConstructible> move_constructed(make_fresh());
  confirm_unowned(move_constructed);
  EXPECT_NO_THROW(move_constructed.AddUnowned(&extra));
  EXPECT_EQ(move_constructed.size(), static_cast<int>(source_data.size()) + 1);

  OwnershipVector<CopyConstructible> move_constructed_from_copy(
      move(copy_constructed));
  confirm_owned_copy(&move_constructed_from_copy);

  /* Move assignment. */
  OwnershipVector<CopyConstructible> move_assigned;
  move_assigned.AddUnowned(&extra);
  EXPECT_EQ(move_assigned.size(), 1);
  EXPECT_EQ(&move_assigned.get(0), &extra);

  move_assigned = make_fresh();

  confirm_unowned(move_assigned);
  EXPECT_NO_THROW(move_assigned.AddUnowned(&extra));
  EXPECT_EQ(move_assigned.size(), static_cast<int>(source_data.size()) + 1);

  OwnershipVector<CopyConstructible> move_assigned_from_copy;
  move_assigned_from_copy.AddUnowned(&extra);
  EXPECT_EQ(move_assigned_from_copy.size(), 1);
  EXPECT_EQ(&move_assigned_from_copy.get(0), &extra);

  move_assigned_from_copy = move(copy_assigned);

  confirm_owned_copy(&move_assigned_from_copy);

  /* Clear - restores it to zero size and addable. */
  OwnershipVector<CopyConstructible> non_owner(make_fresh());
  EXPECT_NO_THROW(non_owner.AddUnowned(&extra));  // Proof it doesn't own.
  EXPECT_GT(non_owner.size(), 0);
  non_owner.clear();
  EXPECT_EQ(non_owner.size(), 0);
  EXPECT_NO_THROW(non_owner.AddUnowned(&extra));  // Proof it doesn't own.

  OwnershipVector<CopyConstructible> owner(copy_assigned_from_copy);
  EXPECT_THROW(owner.AddUnowned(&extra), std::exception);  // Proof is owner.
  EXPECT_GT(owner.size(), 0);
  owner.clear();
  EXPECT_EQ(owner.size(), 0);
  EXPECT_NO_THROW(owner.AddUnowned(&extra));  // Proof it doesn't own.
}

}  // namespace
}  // namespace internal
namespace {

// TODO(SeanCurtis-TRI) We need to test the following:
//  - It initially doesn't own any of the hydroelastic components.
//  - Copies work and are proper, stand-alone copies.
GTEST_TEST(ContactResultsTest, Construction) {}
}  // namespace
}  // namespace multibody
}  // namespace drake
