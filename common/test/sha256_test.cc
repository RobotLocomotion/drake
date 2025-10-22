#include "drake/common/sha256.h"

#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/temp_directory.h"

namespace drake {
namespace {

// Our default constructor produces a dummy hash.
GTEST_TEST(Sha256Test, Default) {
  EXPECT_EQ(Sha256{}.to_string(), std::string(64, '0'));
}

// Since we're just a tiny wrapper around picosha, we don't need much testing of
// the actual checksum math. We'll just spot-test a well-known value: one of the
// standard test vectors for the SHA-2 family.
GTEST_TEST(Sha256Test, WellKnownValue) {
  const std::string data = "abc";
  const std::string expected =
      "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
  EXPECT_EQ(Sha256::Checksum(data).to_string(), expected);
  EXPECT_EQ(Sha256::Parse(expected), Sha256::Checksum(data));
}

// Hex strings can be uppercase and/or lowercase.
GTEST_TEST(Sha256Test, ParseIgnoresCase) {
  EXPECT_EQ(Sha256::Parse(std::string(64, 'c')),
            Sha256::Parse(std::string(64, 'C')));
}

// A valid parse requires a 64-character hex string.
GTEST_TEST(Sha256Test, ParseErrors) {
  // Wrong number of characters.
  EXPECT_FALSE(Sha256::Parse(std::string(60, '0')));
  EXPECT_FALSE(Sha256::Parse(std::string(70, '0')));
  // Non-hex letter.
  EXPECT_FALSE(Sha256::Parse(std::string(64, 'z')));
}

// Here, we're cross-checking the exact implementation of our hash function
// ("glass-box testing"). If we ever change to a different implementation,
// we'll need to adjust this test to allow for it.
GTEST_TEST(Sha256Test, Hash) {
  const std::hash<Sha256> hasher;
  EXPECT_EQ(hasher(Sha256{}), 0);
  EXPECT_EQ(hasher(Sha256::Checksum("abc")),
            // In our hash, the four words are directly XOR'd together. The
            // to_string() prints the low-order byte first, but literals in C
            // code give the high-order byte first, therefore the constants here
            // are "reversed" versions of the words from the to_string().
            0xeacf018fbf1678baULL ^      // ba7816bf8f01cfea, byte-reversed
                0x2322ae5dde404141ULL ^  // 414140de5dae2223, byte-reversed
                0x9c7a1796a36103b0ULL ^  // b00361a396177a9c, byte-reversed
                0xad1500f261ff10b4ULL ^  // b410ff61f20015ad, byte-reversed
                0);
}

GTEST_TEST(Sha256Test, Comparisons) {
  auto zero = Sha256();
  auto abc = Sha256::Checksum("abc");

  EXPECT_TRUE(zero == zero);
  EXPECT_TRUE(abc == abc);
  EXPECT_FALSE(zero == abc);
  EXPECT_FALSE(abc == zero);

  EXPECT_FALSE(zero != zero);
  EXPECT_FALSE(abc != abc);
  EXPECT_TRUE(zero != abc);
  EXPECT_TRUE(abc != zero);

  EXPECT_FALSE(zero < zero);
  EXPECT_FALSE(abc < abc);
  EXPECT_TRUE(zero < abc);
  EXPECT_FALSE(abc < zero);
}

// Reading from an empty file.
GTEST_TEST(Sha256Test, EmptyFileTest) {
  std::ifstream devnull("/dev/null");
  EXPECT_EQ(Sha256::Checksum(&devnull), Sha256::Checksum(""));
}

// Reading from a non-empty file.
GTEST_TEST(Sha256Test, RealFileTest) {
  const std::string data = "abc";
  const std::string temp_filename = temp_directory() + "/abc.txt";
  {
    std::ofstream temp_write(temp_filename);
    temp_write << data;
    DRAKE_THROW_UNLESS(temp_write.good());
  }
  std::ifstream temp_read(temp_filename);
  EXPECT_EQ(Sha256::Checksum(&temp_read), Sha256::Checksum(data));
}

}  // namespace
}  // namespace drake
