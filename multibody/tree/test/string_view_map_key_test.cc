#include "drake/multibody/tree/string_view_map_key.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

class StringViewMapKeyTest : public ::testing::Test {
 public:
  void ValidateStorageKey(const StringViewMapKey& key) {
    // Validate that the string_view is associated with the storage when the
    // optional value is not set.
    ASSERT_EQ(key.storage().has_value(), true);
    EXPECT_EQ(key.storage().value().c_str(), key.view().data());
  }

  void ValidateReferenceKey(const StringViewMapKey& key) {
    ASSERT_EQ(key.storage().has_value(), false);
    EXPECT_EQ(key.view().data(), kReferenceView.data());
  }

 protected:
  void SetUp() {}

  // Short string for validation of SSO (short-string optimization) strings.
  const std::string kShortString = "abcde";

  // Long string for validation of non-SSO strings.
  const std::string kLongString = "abcdefghijklmnopqrstuvwxyz";

  // For "reference" keys.
  const std::string_view kReferenceView = "reference";

  StringViewMapKey kShortKey{kShortString};
  StringViewMapKey kLongKey{kLongString};
  StringViewMapKey kReferenceKey{kReferenceView};
};

// Validate constructed objects.
TEST_F(StringViewMapKeyTest, Construction) {
  ValidateStorageKey(kShortKey);
  ValidateStorageKey(kLongKey);
  ValidateReferenceKey(kReferenceKey);
}

// Validate copy constructed objects.
TEST_F(StringViewMapKeyTest, CopyConstruction) {
  ValidateStorageKey(StringViewMapKey(kShortKey));
  ValidateStorageKey(StringViewMapKey(kLongKey));
  ValidateReferenceKey(StringViewMapKey(kReferenceKey));
}

// Validate move constructed objects.
TEST_F(StringViewMapKeyTest, MoveConstruction) {
  ValidateStorageKey(StringViewMapKey(std::move(kShortKey)));
  ValidateStorageKey(StringViewMapKey(std::move(kLongKey)));
  ValidateReferenceKey(StringViewMapKey(std::move(kReferenceKey)));
}

// Validate copy assigned objects.
TEST_F(StringViewMapKeyTest, CopyAssignment) {
  StringViewMapKey short_copy;
  StringViewMapKey long_copy;
  StringViewMapKey reference_copy;
  ValidateStorageKey(short_copy = kShortKey);
  ValidateStorageKey(long_copy = kLongKey);
  ValidateReferenceKey(reference_copy = kReferenceKey);
}

// Validate move assigned objects.
TEST_F(StringViewMapKeyTest, MoveAssignment) {
  StringViewMapKey short_move;
  StringViewMapKey long_move;
  StringViewMapKey reference_move;
  ValidateStorageKey(short_move = std::move(kShortKey));
  ValidateStorageKey(long_move = std::move(kLongKey));
  ValidateReferenceKey(reference_move = std::move(kReferenceKey));
}


}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
