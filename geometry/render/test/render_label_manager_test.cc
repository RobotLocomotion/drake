#include "drake/geometry/render/render_label_manager.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/value.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

// Simple class with friend access to RenderLabelManager for testing the
// internal state in tests.
class RenderLabelManagerTester {
 public:
  RenderLabelManagerTester() = default;

  // Allocates a label manager with the source id but limits the allocatable
  // number of labels to the given `max_value`.
  RenderLabelManager make_manager(SourceId source_id,
                                  RenderLabel::ValueType max_value) {
    RenderLabelManager manager(source_id);
    manager.maximum_value_ = max_value;
    return manager;
  }

  // Reports the manager's maximum valid value.
  RenderLabel::ValueType maximum_value(
      const RenderLabelManager& manager) const {
    return manager.maximum_value_;
  }

  // Consistency check; confirms that every render label associated with a name
  // *stores* that name.
  ::testing::AssertionResult HasConsistentNames(
      const RenderLabelManager& manager) const {
    for (auto it = manager.name_label_map_.begin();
         it != manager.name_label_map_.end(); ++it) {
      const std::string& label_name = it->first;
      const RenderLabelClass& label_class = it->second;
      if (label_name != label_class.name) {
        return ::testing::AssertionFailure()
            << "Render label class has name '" << label_class.name
            << "' stored in the collection named '" << label_name << "'";
      }
    }
    return ::testing::AssertionSuccess();
  }

  // Confirms that the given render label class is in the manager.
  ::testing::AssertionResult HasRenderLabelClass(
      const RenderLabelManager& manager,
      const RenderLabelClass label_class) const {
    if (manager.name_label_map_.count(label_class.name) == 0) {
      return ::testing::AssertionFailure()
             << "No labels allocated with the name '" << label_class.name
             << "'";
    }
    auto range = manager.name_label_map_.equal_range(label_class.name);
    for (auto it = range.first; it != range.second; ++it) {
      const RenderLabelClass& test_class = it->second;
      if (test_class.source_id == label_class.source_id) {
        if (test_class.label == label_class.label) {
          // We don't need to check names; we'll already have tested for name
          // consistency.
          return ::testing::AssertionSuccess();
        } else {
          return ::testing::AssertionFailure()
                 << "Found render label class with the name '"
                 << label_class.name << "' and matching source id "
                 << label_class.source_id
                 << ", but the label value doesn't match. Expected "
                 << label_class.label << ", found " << test_class.label;
        }
      }
    }
    return ::testing::AssertionFailure()
           << "Unable to find a render label class with name '"
           << label_class.name << "' and source id " << label_class.source_id;
  }

  // Reports the number of labels registered in `manager`.
  int LabelCount(const RenderLabelManager& manager) const {
    return static_cast<int>(manager.name_label_map_.size());
  }

  // Confirms that the reserved labels are in the manager (and recorded
  // correctly). In the case of multiple points of failure, only the first
  // detected is reported.
  ::testing::AssertionResult HasReservedLabels(
      const RenderLabelManager& manager, SourceId source_id) const {
    ::testing::AssertionResult result = HasRenderLabelClass(
        manager, RenderLabelClass("empty", source_id, RenderLabel::kEmpty));
    if (result) {
      result = HasRenderLabelClass(
          manager, RenderLabelClass("do_not_render", source_id,
                                    RenderLabel::kDoNotRender));
      if (result) {
        result = HasRenderLabelClass(
            manager, RenderLabelClass("unspecified", source_id,
                                      RenderLabel::kUnspecified));
        if (result) {
          result = HasRenderLabelClass(
              manager,
              RenderLabelClass("dont_care", source_id, RenderLabel::kDontCare));
        }
      }
    }
    return result;
  }
};

namespace {

using ValueType = RenderLabel::ValueType;

// Simple test confirming that the construction is consistent and coherent.
GTEST_TEST(RenderLabelManagerTest, Constructor) {
  RenderLabelManagerTester tester;
  SourceId source_id = SourceId::get_new_id();
  RenderLabelManager manager(source_id);
  EXPECT_EQ(4, tester.LabelCount(manager));
  EXPECT_TRUE(tester.HasConsistentNames(manager));
  EXPECT_TRUE(tester.HasReservedLabels(manager, source_id));
  EXPECT_EQ(RenderLabel::kMaxUnreserved, tester.maximum_value(manager));

  // Copy constructor.
  RenderLabelManager copy(manager);
  EXPECT_EQ(4, tester.LabelCount(copy));
  EXPECT_TRUE(tester.HasConsistentNames(copy));
  EXPECT_TRUE(tester.HasReservedLabels(copy, source_id));
  EXPECT_EQ(RenderLabel::kMaxUnreserved, tester.maximum_value(copy));
}

GTEST_TEST(RenderLabelManagerTest, GetRenderLabel) {
  RenderLabelManagerTester tester;
  SourceId source_id = SourceId::get_new_id();
  RenderLabelManager manager(source_id);

  // Case: Request with new source (which means implicitly new name) -->
  // produces new label.
  SourceId new_source = SourceId::get_new_id();
  RenderLabel label1;
  EXPECT_NO_THROW({ label1 = manager.GetRenderLabel(new_source, "case1"); });
  EXPECT_TRUE(tester.HasConsistentNames(manager));

  // Case: Request with previously used source and new name --> produces new
  // label.
  RenderLabel label2;
  EXPECT_NO_THROW({ label2 = manager.GetRenderLabel(new_source, "case2"); });
  EXPECT_TRUE(tester.HasConsistentNames(manager));
  EXPECT_NE(label1, label2);

  // Case: Request with previously used source and name --> produces previous
  // label.
  RenderLabel label3;
  EXPECT_NO_THROW({ label3 = manager.GetRenderLabel(new_source, "case2"); });
  EXPECT_TRUE(tester.HasConsistentNames(manager));
  EXPECT_EQ(label2, label3);

  // Case: Exhaust labels.
  // Confirm ability to set maximum value works to facilitate label exhaustion.
  const ValueType max_value = 5;
  RenderLabelManager small_manager = tester.make_manager(source_id, max_value);
  EXPECT_EQ(max_value, tester.maximum_value(small_manager));

  for (int i = 0; i <= max_value; ++i) {
    small_manager.GetRenderLabel(new_source, std::to_string(i));
    EXPECT_TRUE(tester.HasConsistentNames(small_manager));
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      small_manager.GetRenderLabel(new_source, "too many"), std::logic_error,
      "All \\d+ render labels have been allocated");
}

GTEST_TEST(RenderLabelManagerTest, GetRenderLabelClasses) {
  SourceId manager_id = SourceId::get_new_id();
  SourceId source1 = SourceId::get_new_id();
  SourceId source2 = SourceId::get_new_id();

  RenderLabelManager manager(manager_id);
  std::vector<RenderLabelClass> expected_classes;
  auto add_label = [&expected_classes, &manager](const std::string& name,
                                                 SourceId source_id) {
    expected_classes.emplace_back(name, source_id,
                                  manager.GetRenderLabel(source_id, name));
  };
  // Reserved labels.
  expected_classes.emplace_back("empty", manager_id, RenderLabel::kEmpty);
  expected_classes.emplace_back("unspecified", manager_id,
                                RenderLabel::kUnspecified);
  expected_classes.emplace_back("dont_care", manager_id,
                                RenderLabel::kDontCare);
  expected_classes.emplace_back("do_not_render", manager_id,
                                RenderLabel::kDoNotRender);
  add_label("common", source1);
  add_label("common", source2);
  add_label("unique1", source1);
  add_label("unique2", source2);

  std::vector<RenderLabelClass> classes = manager.GetRenderLabelClasses();
  EXPECT_EQ(classes.size(), expected_classes.size());

  for (const auto& label_class : classes) {
    bool found = false;
    for (const auto& expected_class : expected_classes) {
      if (label_class == expected_class) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Reported label class not found in expected classes: "
                       << label_class;
  }
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
