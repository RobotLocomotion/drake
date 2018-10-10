#include "drake/geometry/dev/render/render_label.h"

#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace dev {
namespace render {
namespace  {

class RenderLabelTests : public ::testing::Test {
 protected:
  // Because the label counter is global, to get some *fixed* labels, we
  // need to grab a number before any tests run.
  static void SetUpTestCase() {
    for (int i = 0; i < kLabelCount; ++i) {
      labels_[i] = RenderLabel::new_label();
      // This does two things:
      //  1. Confirms the labels are what the rest of the tests think they are
      //  2. Tests comparisons between labels and ints.
      ASSERT_EQ(labels_[i], i);
      ASSERT_EQ(i, labels_[i]);
    }
  }
  static constexpr int kLabelCount = 4;
  static std::vector<RenderLabel> labels_;
};

std::vector<RenderLabel> RenderLabelTests::labels_{kLabelCount};

// Confirms that default labels are empty labels.
TEST_F(RenderLabelTests, DefaultConstructor) {
  EXPECT_EQ(RenderLabel(), RenderLabel::empty_label());
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
  EXPECT_TRUE(labels_[0] < RenderLabel::terrain_label());
}

// Confirms that when I've acquired T - 1 labels (where T is the value of the
// terrain label) that subsequent efforts to request a label will fail.
TEST_F(RenderLabelTests, ConsumeAllLabels) {
  // NOTE: This will require counting up to 2 billion.
  EXPECT_THROW(while (true) RenderLabel::new_label(), std::runtime_error);
}

// Confirms that labels are configured to serve as unique keys in
// STL containers.
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

}  // namespace
}  // namespace render
}  // namespace dev
}  // namespace geometry
}  // namespace drake
