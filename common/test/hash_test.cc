#include "drake/common/hash.h"

#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace {

// A hasher which simply records all of its invocations for later inspection
class MockHasher {
 public:
  void operator()(const void* data, size_t length) noexcept {
    const uint8_t* const begin = static_cast<const uint8_t*>(data);
    const uint8_t* const end = begin + length;

    std::vector<uint8_t> bytes;
    for (const uint8_t* iter = begin; iter < end; ++iter) {
      bytes.push_back(*iter);
    }
    record_.push_back(bytes);
  }

  std::vector<std::vector<uint8_t>> record() const { return record_; }

 private:
  std::vector<std::vector<uint8_t>> record_;
};


GTEST_TEST(HashTest, HashAppendOptional) {
  // Test basic functionality:  ensure two equal values get hashed the same way,
  // and that an empty value and non-empty value are hashed differently
  // (regardless of whether or not the hashes turn out the same).
  drake::optional<int> nonempty1(99);
  drake::optional<int> nonempty2(99);
  MockHasher hne1;
  MockHasher hne2;
  hash_append(hne1, nonempty1);
  hash_append(hne2, nonempty2);
  EXPECT_EQ(hne1.record(), hne2.record());
  EXPECT_EQ(hne1.record().size(), 2);

  drake::optional<int> empty1;
  drake::optional<int> empty2;
  MockHasher he1;
  MockHasher he2;
  hash_append(he1, empty1);
  hash_append(he2, empty2);
  EXPECT_EQ(he1.record(), he2.record());
  EXPECT_EQ(he1.record().size(), 1);

  EXPECT_NE(he1.record(), hne1.record());
}

}  // namespace
}  // namespace drake
