#include "drake/multibody/tree/element_collection.h"

#include <set>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/model_instance.h"

namespace drake {
namespace multibody {
namespace internal {

// For this unit test, we arbitrarily pick one of the valid element types that
// is fully supported by ElementCollection. To avoid spamming this arbitrary
// choice too widely, we'll introduce shorthand for it.
template <typename T>
using Element = ModelInstance<T>;
using Index = ModelInstanceIndex;
using Collection = ElementCollection<double, ModelInstance, Index>;

GTEST_TEST(ElementCollectionTest, Empty) {
  const Collection empty;
  EXPECT_EQ(empty.num_elements(), 0);
  EXPECT_EQ(empty.elements().size(), 0);
  EXPECT_EQ(empty.indices().size(), 0);
  EXPECT_EQ(empty.names_map().size(), 0);
  EXPECT_EQ(empty.next_index(), Index{0});
}

// Adding elements with no removals and no holes.
GTEST_TEST(ElementCollectionTest, AddDense) {
  Element<double> one(Index{1}, "one");

  Collection dut;
  dut.Add(std::make_unique<Element<double>>(Index{0}, "zero"));
  dut.AddBorrowed(&one);

  EXPECT_EQ(dut.num_elements(), 2);
  EXPECT_EQ(dut.elements().size(), 2);
  EXPECT_EQ(dut.elements().at(0)->name(), "zero");
  EXPECT_EQ(dut.elements().at(1)->name(), "one");

  EXPECT_TRUE(dut.has_element(Index{0}));
  EXPECT_TRUE(dut.has_element(Index{1}));
  EXPECT_EQ(dut.get_element(Index{0}).name(), "zero");
  EXPECT_EQ(dut.get_element(Index{1}).name(), "one");
  EXPECT_EQ(dut.get_mutable_element(Index{0}).name(), "zero");
  EXPECT_EQ(dut.get_mutable_element(Index{1}).name(), "one");
  EXPECT_EQ(dut.get_element_unchecked(Index{0}).name(), "zero");
  EXPECT_EQ(dut.get_element_unchecked(Index{1}).name(), "one");
  EXPECT_EQ(dut.get_element_unchecked(Index{1}).name(), "one");
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_element(Index{}),
                              ".*default-constructed ModelInstanceIndex.*");
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_mutable_element(Index{}),
                              ".*default-constructed ModelInstanceIndex.*");
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_element(Index{22}),
                              ".*ModelInstanceIndex.*22.*bounds.*");
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_mutable_element(Index{22}),
                              ".*ModelInstanceIndex.*22.*bounds.*");

  EXPECT_EQ(dut.names_map().size(), 2);
  EXPECT_EQ(dut.names_map().count(std::string{"zero"}), 1);
  EXPECT_EQ(dut.names_map().count(std::string{"one"}), 1);

  EXPECT_THAT(dut.indices(), testing::ElementsAre(Index{0}, Index{1}));
  EXPECT_EQ(dut.next_index(), Index{2});

  dut.Add(std::make_unique<Element<double>>(Index{2}, "two"));
  EXPECT_EQ(dut.num_elements(), 3);
  EXPECT_EQ(dut.next_index(), Index{3});
}

// Adding elements with duplicate names.
GTEST_TEST(ElementCollectionTest, DuplicateNames) {
  Collection dut;
  dut.Add(std::make_unique<Element<double>>(Index{0}, "foo"));
  dut.Add(std::make_unique<Element<double>>(Index{1}, "bar"));
  dut.Add(std::make_unique<Element<double>>(Index{2}, "foo"));
  dut.Add(std::make_unique<Element<double>>(Index{3}, "bar"));
  dut.Add(std::make_unique<Element<double>>(Index{4}, "foo"));
  EXPECT_EQ(dut.names_map().size(), 5);
  EXPECT_EQ(dut.names_map().count(std::string{"foo"}), 3);
  EXPECT_EQ(dut.names_map().count(std::string{"bar"}), 2);
  std::set<Index> foo_indices;
  auto [lower, upper] = dut.names_map().equal_range(std::string{"foo"});
  for (auto iter = lower; iter != upper; ++iter) {
    const Index index = iter->second;
    foo_indices.insert(index);
  }
  EXPECT_THAT(foo_indices, testing::ElementsAre(Index{0}, Index{2}, Index{4}));
}

// Variously adding, removing, and renaming elements.
GTEST_TEST(ElementCollectionTest, Removals) {
  Element<double> one(Index{1}, "one");

  // Add 0, add 1, skip 2, add 3.
  // Overall we have indices 0,1,3 now.
  Collection dut;
  dut.Add(std::make_unique<Element<double>>(Index{0}, "zero"));
  dut.AddBorrowed(&one);
  dut.AppendNull();
  dut.Add(std::make_unique<Element<double>>(Index{3}, "three"));

  EXPECT_EQ(dut.num_elements(), 3);
  EXPECT_EQ(dut.elements().size(), 3);
  EXPECT_EQ(dut.elements().at(0)->name(), "zero");
  EXPECT_EQ(dut.elements().at(1)->name(), "one");
  EXPECT_EQ(dut.elements().at(2)->name(), "three");

  EXPECT_TRUE(dut.has_element(Index{0}));
  EXPECT_TRUE(dut.has_element(Index{1}));
  EXPECT_FALSE(dut.has_element(Index{2}));
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_element(Index{2}),
                              ".*ModelInstanceIndex.*2.*removed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_mutable_element(Index{2}),
                              ".*ModelInstanceIndex.*2.*removed.*");
  EXPECT_TRUE(dut.has_element(Index{3}));
  EXPECT_EQ(dut.get_element(Index{3}).name(), "three");
  EXPECT_EQ(dut.get_mutable_element(Index{3}).name(), "three");
  EXPECT_EQ(dut.get_element_unchecked(Index{3}).name(), "three");

  EXPECT_EQ(dut.names_map().size(), 3);
  EXPECT_EQ(dut.names_map().count(std::string{"zero"}), 1);
  EXPECT_EQ(dut.names_map().count(std::string{"one"}), 1);
  EXPECT_EQ(dut.names_map().count(std::string{"three"}), 1);

  EXPECT_THAT(dut.indices(),
              testing::ElementsAre(Index{0}, Index{1}, Index{3}));
  EXPECT_EQ(dut.next_index(), Index{4});

  // Add 4.
  // Overall we have indices 0,1,3,4 now.
  dut.Add(std::make_unique<Element<double>>(Index{4}, "four"));
  EXPECT_EQ(dut.num_elements(), 4);
  EXPECT_EQ(dut.next_index(), Index{5});

  // Remove 1 then 0. Add 5. Rename 3.
  // Overall we have indices 3,4,5 now (with a new name for 3).
  dut.Remove(Index{1});
  dut.Remove(Index{0});
  dut.Add(std::make_unique<Element<double>>(Index{5}, "five"));
  dut.Rename(Index{3}, "tres");

  EXPECT_EQ(dut.num_elements(), 3);
  EXPECT_EQ(dut.elements().size(), 3);
  EXPECT_EQ(dut.elements().at(0)->name(), "tres");
  EXPECT_EQ(dut.elements().at(1)->name(), "four");
  EXPECT_EQ(dut.elements().at(2)->name(), "five");

  EXPECT_FALSE(dut.has_element(Index{0}));
  EXPECT_FALSE(dut.has_element(Index{1}));
  EXPECT_FALSE(dut.has_element(Index{2}));
  EXPECT_TRUE(dut.has_element(Index{3}));
  EXPECT_TRUE(dut.has_element(Index{4}));
  EXPECT_TRUE(dut.has_element(Index{5}));
  EXPECT_EQ(dut.get_element(Index{5}).name(), "five");
  EXPECT_EQ(dut.get_mutable_element(Index{5}).name(), "five");
  EXPECT_EQ(dut.get_element_unchecked(Index{5}).name(), "five");

  EXPECT_EQ(dut.names_map().size(), 3);
  EXPECT_EQ(dut.names_map().count(std::string{"tres"}), 1);
  EXPECT_EQ(dut.names_map().count(std::string{"four"}), 1);
  EXPECT_EQ(dut.names_map().count(std::string{"five"}), 1);

  EXPECT_THAT(dut.indices(),
              testing::ElementsAre(Index{3}, Index{4}, Index{5}));
  EXPECT_EQ(dut.next_index(), Index{6});
}

// Removing items with duplicate names respects the index.
GTEST_TEST(ElementCollectionTest, RemovalWithDuplicates) {
  const std::string name("foo");
  Element<double> one(Index{1}, name);
  Collection dut;
  dut.Add(std::make_unique<Element<double>>(Index{0}, name));
  dut.AddBorrowed(&one);
  EXPECT_EQ(dut.names_map().count(name), 2);

  dut.Remove(Index{0});
  EXPECT_THAT(dut.indices(), testing::ElementsAre(Index{1}));
  ASSERT_EQ(dut.names_map().count(name), 1);
  EXPECT_EQ(dut.names_map().find(name)->second, Index{1});
}

// Allocate a range of indices and then populate them out-of-order.
// This matches how MbT::Clone currently works.
GTEST_TEST(ElementCollectionTest, AllocateThenAdd) {
  ElementCollection<AutoDiffXd, ModelInstance, Index> other;
  for (int i = 0; i < 5; ++i) {
    other.AppendNull();
  }

  Collection dut;
  dut.Add(std::make_unique<Element<double>>(Index{0}, "zero"));
  dut.Add(std::make_unique<Element<double>>(Index{1}, "one"));
  dut.ResizeToMatch(other);
  dut.Add(std::make_unique<Element<double>>(Index{4}, "four"));
  dut.Add(std::make_unique<Element<double>>(Index{2}, "two"));
  dut.Add(std::make_unique<Element<double>>(Index{3}, "three"));
  EXPECT_EQ(dut.next_index(), Index{5});

  const std::vector<std::string> expected_names{
      std::string{"zero"}, std::string{"one"}, std::string{"two"},
      std::string{"three"}, std::string{"four"}};
  EXPECT_EQ(dut.num_elements(), 5);
  EXPECT_EQ(dut.elements().size(), 5);
  EXPECT_EQ(dut.names_map().size(), 5);
  EXPECT_EQ(dut.indices().size(), 5);
  for (Index i{0}; i < 5; ++i) {
    const std::string& name = expected_names.at(i);
    EXPECT_EQ(dut.elements().at(i)->name(), name);
    EXPECT_TRUE(dut.has_element(i));
    EXPECT_EQ(dut.get_element(i).name(), name);
    EXPECT_EQ(dut.get_mutable_element(i).name(), name);
    EXPECT_EQ(dut.get_element_unchecked(i).name(), name);
    EXPECT_EQ(dut.names_map().count(name), 1);
    EXPECT_EQ(dut.indices().at(i), i);
  }
}

// A simple sanity check that the class can be instantiated.
GTEST_TEST(ElementCollectionTest, Autodiff) {
  ElementCollection<AutoDiffXd, ModelInstance, Index> autodiff;
}

// A simple sanity check that the class can be instantiated.
GTEST_TEST(ElementCollectionTest, Symbolic) {
  ElementCollection<symbolic::Expression, ModelInstance, Index> symbolic;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
