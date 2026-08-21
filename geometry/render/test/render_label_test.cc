#include "drake/geometry/render/render_label.h"

#include <set>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace {

class RenderLabelTests : public ::testing::Test {
 protected:
  // Each test has the same labels drawn from an independent pool.
  void SetUp() override {
    for (RenderLabel::ValueType i = 0; i < kLabelCount; ++i) {
      labels_.emplace_back(i);
      // This does two things:
      //  1. Confirms the labels are what the rest of the tests think they are,
      //  2. Tests comparisons between labels and ints.
      ASSERT_EQ(labels_[i], static_cast<int>(i));
    }
  }
  static constexpr int kLabelCount = 4;
  std::vector<RenderLabel> labels_;
};

// Confirms that default labels are unclassified labels.
TEST_F(RenderLabelTests, Construction) {
  // Defaults to unclassified.
  EXPECT_EQ(RenderLabel(), RenderLabel::kUnspecified);

  // Bad values throw an exception.
  using ValueType = RenderLabel::ValueType;

  std::vector<ValueType> bad_values{
      -1, RenderLabel::kUnspecified, RenderLabel::kDoNotRender,
      RenderLabel::kDontCare, RenderLabel::kEmpty};
  for (ValueType value : bad_values) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        RenderLabel{value},
        "Invalid construction of RenderLabel with invalid value: .+");
  }

  // Range of good values.
  DRAKE_EXPECT_NO_THROW(RenderLabel{0});
  DRAKE_EXPECT_NO_THROW(RenderLabel{RenderLabel::kMaxUnreserved});
}

// Confirms that the reserved labels report correctly, and that generated labels
// do not report as reserved.
TEST_F(RenderLabelTests, TestForReservedLabels) {
  for (RenderLabel label : labels_) {
    EXPECT_FALSE(label.is_reserved());
  }
  EXPECT_TRUE(RenderLabel::kEmpty.is_reserved());
  EXPECT_EQ(RenderLabel::kEmpty, RenderLabel::kEmpty);
  EXPECT_TRUE(RenderLabel::kDontCare.is_reserved());
  EXPECT_EQ(RenderLabel::kDontCare, RenderLabel::kDontCare);
  EXPECT_TRUE(RenderLabel::kDoNotRender.is_reserved());
  EXPECT_EQ(RenderLabel::kDoNotRender, RenderLabel::kDoNotRender);
  EXPECT_TRUE(RenderLabel::kUnspecified.is_reserved());
  EXPECT_EQ(RenderLabel::kUnspecified, RenderLabel::kUnspecified);

  // Note: This assumes that the A != B is commutative.

  EXPECT_NE(RenderLabel::kEmpty, RenderLabel::kDontCare);
  EXPECT_NE(RenderLabel::kEmpty, RenderLabel::kDoNotRender);
  EXPECT_NE(RenderLabel::kEmpty, RenderLabel::kUnspecified);
  EXPECT_NE(RenderLabel::kDontCare, RenderLabel::kDoNotRender);
  EXPECT_NE(RenderLabel::kDontCare, RenderLabel::kUnspecified);
  EXPECT_NE(RenderLabel::kDoNotRender, RenderLabel::kUnspecified);
}

// Confirms that assignment behaves correctly. This also implicitly tests
// equality and inequality.
TEST_F(RenderLabelTests, AssignmentAndComparison) {
  EXPECT_TRUE(labels_[0] != labels_[1]);
  RenderLabel temp = labels_[1];
  EXPECT_EQ(temp, labels_[1]);
  temp = labels_[2];
  EXPECT_EQ(temp, labels_[2]);
  // This exploits the knowledge that labels were allocated in a monotonically
  // increasing order, while the reserved labels occupy the highest label
  // values.
  EXPECT_TRUE(labels_[0] < labels_[1]);
  EXPECT_TRUE(labels_[0] < RenderLabel::kEmpty);
  EXPECT_TRUE(labels_[0] < RenderLabel::kDoNotRender);
  EXPECT_TRUE(labels_[0] < RenderLabel::kUnspecified);

  // Comparison of RenderLabel with values of underlying type.
  for (RenderLabel::ValueType i(0); i < kLabelCount; ++i) {
    EXPECT_TRUE(labels_[i] == i);
  }
}

// Confirms that RenderLabels can be used as hashable entries in STL containers
// (e.g., unordered_set and unordered_map).
TEST_F(RenderLabelTests, ServeAsMapKey) {
  std::unordered_set<RenderLabel> label_set;

  // This is a *different* label with the *same* value as labels_[0]. It should
  // *not* introduce a new value to the set.
  RenderLabel same_as_label0 = labels_[0];

  EXPECT_EQ(label_set.size(), 0);
  label_set.insert(labels_[0]);
  EXPECT_NE(label_set.find(labels_[0]), label_set.end());
  EXPECT_NE(label_set.find(same_as_label0), label_set.end());

  EXPECT_EQ(label_set.size(), 1);
  label_set.insert(labels_[1]);
  EXPECT_EQ(label_set.size(), 2);

  label_set.insert(same_as_label0);
  EXPECT_EQ(label_set.size(), 2);

  EXPECT_EQ(label_set.find(labels_[2]), label_set.end());
}

// Confirms that RenderLabels can be used as orderable entries in STL containers
// (e.g., set and map).
TEST_F(RenderLabelTests, ServeAsSetMember) {
  std::set<RenderLabel> label_set;

  // This is a *different* label with the *same* value as labels_[0]. It should
  // *not* introduce a new value to the set.
  RenderLabel same_as_label0 = labels_[0];

  EXPECT_EQ(label_set.size(), 0);
  label_set.insert(labels_[0]);
  EXPECT_NE(label_set.find(labels_[0]), label_set.end());
  EXPECT_NE(label_set.find(same_as_label0), label_set.end());

  EXPECT_EQ(label_set.size(), 1);
  label_set.insert(labels_[1]);
  EXPECT_EQ(label_set.size(), 2);

  label_set.insert(same_as_label0);
  EXPECT_EQ(label_set.size(), 2);

  EXPECT_EQ(label_set.find(labels_[2]), label_set.end());
}

GTEST_TEST(RenderLabelTest, ToString) {
  const RenderLabel dut = RenderLabel::kUnspecified;
  EXPECT_EQ(dut.to_string(), "32767");
  EXPECT_EQ(fmt::to_string(dut), "32767");
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
