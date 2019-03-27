#include "drake/geometry/render/render_label.h"

#include <set>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render {
namespace  {

using systems::sensors::ColorI;
using systems::sensors::ColorD;

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
        RenderLabel{value}, std::logic_error,
        "Invalid construction of RenderLabel with invalid value: .+");
  }

  // Range of good values.
  EXPECT_NO_THROW(RenderLabel{0});
  EXPECT_NO_THROW(RenderLabel{RenderLabel::kMaxUnreserved});
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

TEST_F(RenderLabelTests, ColorLabelConversion) {
  // Explicitly testing labels at *both* ends of the reserved space -- this
  // assumes that the reserved labels are at the top end; if that changes, we'll
  // need a different mechanism to get a large-valued label.
  RenderLabel label1 = RenderLabel(0);
  RenderLabel label2 = RenderLabel(RenderLabel::kMaxUnreserved - 1);
  RenderLabel label3 = RenderLabel::kEmpty;

  // A ColorI should be invertible back to the original label.
  ColorI color1 = label1.GetColorI();
  ColorI color2 = label2.GetColorI();
  ColorI color3 = label3.GetColorI();
  EXPECT_EQ(label1, RenderLabel::LabelFromColor(color1));
  EXPECT_EQ(label2, RenderLabel::LabelFromColor(color2));
  EXPECT_EQ(label3, RenderLabel::LabelFromColor(color3));

  // Different labels should produce different colors.
  ASSERT_NE(label1, label2);
  ASSERT_NE(label2, label3);
  ASSERT_NE(label1, label3);
  auto same_colors = [](const auto& expected, const auto& test) {
    if (expected.r != test.r || expected.g != test.g || expected.b != test.b) {
      return ::testing::AssertionFailure()
          << "Expected color " << expected << ", found " << test;
    }
    return ::testing::AssertionSuccess();
  };

  EXPECT_FALSE(same_colors(color1, color2));
  EXPECT_FALSE(same_colors(color2, color3));
  EXPECT_FALSE(same_colors(color1, color3));

  // Different labels should also produce different Normalized colors.
  ColorD color1_d = label1.GetColorD();
  ColorD color2_d = label2.GetColorD();
  ColorD color3_d = label3.GetColorD();
  EXPECT_FALSE(same_colors(color1_d, color2_d));
  EXPECT_FALSE(same_colors(color1_d, color3_d));
  EXPECT_FALSE(same_colors(color2_d, color3_d));

  // THe normalized color should simply be the integer color divided by 255.
  ColorD color1_d_by_hand{color1.r / 255., color1.g / 255., color1.b / 255.};
  ColorD color2_d_by_hand{color2.r / 255., color2.g / 255., color2.b / 255.};
  ColorD color3_d_by_hand{color3.r / 255., color3.g / 255., color3.b / 255.};

  EXPECT_TRUE(same_colors(color1_d, color1_d_by_hand));
  EXPECT_TRUE(same_colors(color2_d, color2_d_by_hand));
  EXPECT_TRUE(same_colors(color3_d, color3_d_by_hand));
}

}  // namespace
}  // namespace render
}  // namespace geometry
}  // namespace drake
