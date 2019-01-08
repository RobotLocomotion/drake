#include "drake/geometry/render/render_label.h"

#include <set>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace render {

// Utility function for generating new labels.
class RenderLabelTester {
 public:
  // Allocates a render label from a finite pool.
  RenderLabel new_label() { return RenderLabel(next_value_++); }

  // Allocates a render label with the given value.
  RenderLabel new_label(RenderLabel::ValueType value) {
    return RenderLabel(value);
  }

 private:
  RenderLabel::ValueType next_value_{0};
};

namespace  {

class RenderLabelTests : public ::testing::Test {
 protected:
  // Each test has the same labels drawn from an independent pool.
  void SetUp() override {
    for (int i = 0; i < kLabelCount; ++i) {
      labels_.push_back(tester.new_label());
      // This does two things:
      //  1. Confirms the labels are what the rest of the tests think they are,
      //  2. Tests comparisons between labels and ints.
      ASSERT_EQ(labels_[i], i);
    }
  }
  RenderLabelTester tester;
  static constexpr int kLabelCount = 4;
  std::vector<RenderLabel> labels_;
};

// Confirms that default labels are unclassified labels.
TEST_F(RenderLabelTests, DefaultConstructor) {
  EXPECT_EQ(RenderLabel(), RenderLabel::unclassified_label());
}

// Confirms that the reserved labels report correctly, and that generated labels
// do not report as reserved.
TEST_F(RenderLabelTests, TestForReservedLabels) {
  for (RenderLabel label : labels_) {
    EXPECT_FALSE(label.is_empty());
    EXPECT_FALSE(label.is_ignored());
    EXPECT_FALSE(label.is_unclassified());
  }
  EXPECT_TRUE(RenderLabel::empty_label().is_empty());
  EXPECT_FALSE(RenderLabel::empty_label().is_ignored());
  EXPECT_FALSE(RenderLabel::empty_label().is_unclassified());

  EXPECT_FALSE(RenderLabel::ignored_label().is_empty());
  EXPECT_TRUE(RenderLabel::ignored_label().is_ignored());
  EXPECT_FALSE(RenderLabel::ignored_label().is_unclassified());

  EXPECT_FALSE(RenderLabel::unclassified_label().is_empty());
  EXPECT_FALSE(RenderLabel::unclassified_label().is_ignored());
  EXPECT_TRUE(RenderLabel::unclassified_label().is_unclassified());
}

// Confirms that assignment behaves correctly. This also implicitly tests
// equality and inequality.
TEST_F(RenderLabelTests, AssignmentAndComparison) {
  EXPECT_TRUE(labels_[0] != labels_[1]);
  RenderLabel temp = labels_[1];
  EXPECT_TRUE(temp == labels_[1]);
  temp = labels_[2];
  EXPECT_TRUE(temp == labels_[2]);
  // This exploits the knowledge that labels are allocated in a monotonically
  // increasing order. Also, all allocated labels are smaller than the special
  // values.
  EXPECT_TRUE(labels_[0] < labels_[1]);
  EXPECT_TRUE(labels_[0] < RenderLabel::empty_label());
  EXPECT_TRUE(labels_[0] < RenderLabel::ignored_label());
  EXPECT_TRUE(labels_[0] < RenderLabel::unclassified_label());
}

// Confirms that RenderLabels can be used as hashable entries in STL containers
// (e.g., unordered_set and unordered_map).
TEST_F(RenderLabelTests, ServeAsMapKey) {
  std::unordered_set<RenderLabel> label_set;

  // This is a *different* label with the *same* value as labels_[0]. It should
  // *not* introduce a new value to the set.
  RenderLabel temp = labels_[0];

  EXPECT_EQ(label_set.size(), 0);
  label_set.insert(labels_[0]);
  EXPECT_NE(label_set.find(labels_[0]), label_set.end());
  EXPECT_NE(label_set.find(temp), label_set.end());

  EXPECT_EQ(label_set.size(), 1);
  label_set.insert(labels_[1]);
  EXPECT_EQ(label_set.size(), 2);

  label_set.insert(temp);
  EXPECT_EQ(label_set.size(), 2);

  EXPECT_EQ(label_set.find(labels_[2]), label_set.end());
}

// Confirms that RenderLabels can be used as orderable entries in STL containers
// (e.g., set and map).
TEST_F(RenderLabelTests, ServeAsSetMember) {
  std::set<RenderLabel> label_set;

  // This is a *different* label with the *same* value as labels_[0]. It should
  // *not* introduce a new value to the set.
  RenderLabel temp = labels_[0];

  EXPECT_EQ(label_set.size(), 0);
  label_set.insert(labels_[0]);
  EXPECT_NE(label_set.find(labels_[0]), label_set.end());
  EXPECT_NE(label_set.find(temp), label_set.end());

  EXPECT_EQ(label_set.size(), 1);
  label_set.insert(labels_[1]);
  EXPECT_EQ(label_set.size(), 2);

  label_set.insert(temp);
  EXPECT_EQ(label_set.size(), 2);

  EXPECT_EQ(label_set.find(labels_[2]), label_set.end());
}

// Attempting to construct a negative-valued render label explodes.
TEST_F(RenderLabelTests, InvalidValueDeathTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      {tester.new_label(-1);},
      "abort: .*render_label.h:.+ in RenderLabel.+"
      "'value >= 0' failed.");
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
