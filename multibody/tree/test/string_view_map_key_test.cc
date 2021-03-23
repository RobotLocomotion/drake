#include "drake/multibody/tree/string_view_map_key.h"

#include <unordered_map>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace internal {
namespace {

class StringViewMapKeyTest : public ::testing::Test {
 public:
  void ValidateStorageKey(const StringViewMapKey& key) {
    // Validate that the string_view is associated with the storage when the
    // optional value is set.
    ASSERT_EQ(key.storage().has_value(), true);
    EXPECT_EQ(key.storage().value().c_str(), key.view().data());
  }

  void ValidateReferenceKey(const StringViewMapKey& key) {
    ASSERT_EQ(key.storage().has_value(), false);
    EXPECT_EQ(key.view().data(), kReferenceView.data());
  }

 protected:
  // Short string for validation of SSO (short-string optimization) strings.
  const std::string kShortString = "abcde";

  // Long string for validation of non-SSO strings.
  const std::string kLongString = "abcdefghijklmnopqrstuvwxyz";

  // For "reference" keys.
  const std::string_view kReferenceView = "reference";

  StringViewMapKey short_key{kShortString};
  StringViewMapKey long_key{kLongString};
  StringViewMapKey reference_key{kReferenceView};
};

// Confirm the class can actually serve its published purpose: as a key in an
// unordered map that allows string and string_view based queries.
TEST_F(StringViewMapKeyTest, AsUnorderedMapKey) {
  std::unordered_map<StringViewMapKey, int> values;
  const std::string key_value{"one"};
  const std::string_view key_view(key_value);
  values.emplace(StringViewMapKey(key_value), 1);
  EXPECT_EQ(values.count(key_value), 1);
  EXPECT_EQ(values.count(key_view), 1);
}

// Validate constructed objects.
TEST_F(StringViewMapKeyTest, Construction) {
  ValidateStorageKey(short_key);
  ValidateStorageKey(long_key);
  ValidateReferenceKey(reference_key);
}

// Validate copy constructed objects.
TEST_F(StringViewMapKeyTest, CopyConstruction) {
  const StringViewMapKey short_copy(short_key);
  const StringViewMapKey long_copy(long_key);
  const StringViewMapKey reference_copy(reference_key);
  ValidateStorageKey(short_copy);
  ValidateStorageKey(long_copy);
  ValidateReferenceKey(reference_copy);

  // Confirm that copies of owning keys own unique values and that non-owning
  // copies share the string with the source.
  EXPECT_NE(short_copy.view().data(), short_key.view().data());
  EXPECT_NE(long_copy.view().data(), long_key.view().data());
  EXPECT_EQ(reference_copy.view().data(), reference_key.view().data());
}

// Validate move constructed objects.
TEST_F(StringViewMapKeyTest, MoveConstruction) {
  ValidateStorageKey(StringViewMapKey(std::move(short_key)));
  ValidateStorageKey(StringViewMapKey(std::move(long_key)));
  ValidateReferenceKey(StringViewMapKey(std::move(reference_key)));
}

// Validate copy assigned objects.
TEST_F(StringViewMapKeyTest, CopyAssignment) {
  StringViewMapKey short_copy;
  StringViewMapKey long_copy;
  StringViewMapKey reference_copy;
  ValidateStorageKey(short_copy = short_key);
  ValidateStorageKey(long_copy = long_key);
  ValidateReferenceKey(reference_copy = reference_key);

  // Confirm that copies of owning keys own unique values and that non-owning
  // copies share the string with the source.
  EXPECT_NE(short_copy.view().data(), short_key.view().data());
  EXPECT_NE(long_copy.view().data(), long_key.view().data());
  EXPECT_EQ(reference_copy.view().data(), reference_key.view().data());
}

// Validate move assigned objects.
TEST_F(StringViewMapKeyTest, MoveAssignment) {
  StringViewMapKey short_move;
  StringViewMapKey long_move;
  StringViewMapKey reference_move;
  ValidateStorageKey(short_move = std::move(short_key));
  ValidateStorageKey(long_move = std::move(long_key));
  ValidateReferenceKey(reference_move = std::move(reference_key));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
