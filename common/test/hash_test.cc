#include "drake/common/hash.h"

#include <utility>
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
    record_.emplace_back(begin, end);
  }

  std::vector<std::vector<uint8_t>> record() const { return record_; }

 private:
  std::vector<std::vector<uint8_t>> record_;
};

GTEST_TEST(HashTest, HashAppendOptional) {
  // Test basic functionality:  ensure two equal values get hashed the same way,
  // and that an empty value and non-empty value are hashed differently
  // (regardless of whether or not the hashes turn out the same).
  std::optional<int> nonempty1(99);
  std::optional<int> nonempty2(99);
  MockHasher hash_nonempty1;
  MockHasher hash_nonempty2;
  hash_append(hash_nonempty1, nonempty1);
  hash_append(hash_nonempty2, nonempty2);
  EXPECT_EQ(hash_nonempty1.record(), hash_nonempty2.record());
  EXPECT_EQ(hash_nonempty1.record().size(), 2);

  std::optional<int> empty1;
  std::optional<int> empty2;
  MockHasher hash_empty1;
  MockHasher hash_empty2;
  hash_append(hash_empty1, empty1);
  hash_append(hash_empty2, empty2);
  EXPECT_EQ(hash_empty1.record(), hash_empty2.record());
  EXPECT_EQ(hash_empty1.record().size(), 1);

  // We specifically want to ensure that the hasher is called in a different
  // way for the empty and non-empty cases.  Given that `hash_append` for
  // `int` and `bool` each invoke the hasher once, the following expectation
  // on total invocation counts reliably tests this:
  EXPECT_NE(hash_empty1.record().size(), hash_nonempty1.record().size());
}

// This is mostly just a compilation test. If the declarations vs definitions
// in the header file are incorrectly ordered, this would fail to compile.
GTEST_TEST(HashTest, HashAppendPairOptionals) {
  const std::pair<std::optional<int>, std::optional<int>> foo{1, 2};
  MockHasher foo_hash;
  hash_append(foo_hash, foo);
  EXPECT_EQ(foo_hash.record().size(), 4);
}

GTEST_TEST(HashTest, HashAppendPointer) {
  const std::pair<int, const int*> foo{22, nullptr};
  MockHasher foo_hash;
  hash_append(foo_hash, foo);

  MockHasher expected;
  hash_append(expected, 22);
  hash_append(expected, std::uintptr_t{});

  EXPECT_EQ(foo_hash.record(), expected.record());
}

}  // namespace
}  // namespace drake
