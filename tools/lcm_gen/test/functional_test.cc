#include <gmock/gmock.h>
#include <gtest/gtest.h>

// The "papa" (i.e, "probed") messages are generated by our lcm_gen tool.
// These are the "classes under test" for this file.

// The "romeo" (i.e., "reference") messages are generated by the upstream
// lcm-gen tool. Here, we use those as an "oracle" to compare against.

// clang-format off
#include "papa/lima.h"
#include "papa/mike.h"
#include "romeo/lima.hpp"
#include "romeo/mike.hpp"
// clang-format on

#include "drake/lcm/lcm_messages.h"

namespace {

// We use vector<int> here so that googletest doesn't print the bytes as
// `char`s -- we want to see them data as numbers, not ASCII goop.
template <typename Message>
std::vector<int> Encode(const Message& message) {
  std::vector<uint8_t> chars = drake::lcm::EncodeLcmMessage(message);
  return std::vector<int>(chars.begin(), chars.end());
}

template <typename ActualMessage, typename ExpectedMessage>
bool CompareEncode(const ActualMessage& actual,
                   const ExpectedMessage& expected) {
  const auto actual_bytes = Encode(actual);
  const auto expected_bytes = Encode(expected);

  // As a convenience, provide a specific wrong-size failure message.
  // The default output for EXPECT_EQ on the two arrays is a bit hard to grok.
  EXPECT_EQ(actual_bytes.size(), expected_bytes.size());

  // As a convenience, provide an acute summary of the first differing byte.
  // The default output for EXPECT_EQ on the two arrays is a bit hard to grok.
  for (size_t i = 0;; ++i) {
    if (i >= actual_bytes.size()) {
      break;
    }
    if (i >= expected_bytes.size()) {
      break;
    }
    EXPECT_EQ(actual_bytes[i], expected_bytes[i]) << i << "th message byte";
    if (actual_bytes[i] != expected_bytes[i]) {
      break;
    }
  }

  // Compare the full arrays. If this fails, the both array's entire contents
  // will be printed.
  EXPECT_EQ(actual_bytes, expected_bytes);

  return actual_bytes == expected_bytes;
}

// Check that our encoded 'lima' message is identical to upstream.
GTEST_TEST(RomeoTest, LimaEncode) {
  ASSERT_EQ(papa::lima::getHash(), romeo::lima::getHash());

  papa::lima papa{};
  romeo::lima romeo{};
  ASSERT_TRUE(CompareEncode(papa, romeo));

  // We'll change the struct fields one at a time, and keep checking that both
  // papa and romeo encode identically as we so do. This is the list of changes.
  const auto check_one = [&](const auto& mutation) {
    mutation(&papa);
    mutation(&romeo);
    ASSERT_TRUE(CompareEncode(papa, romeo));
  };
  std::apply(
      [&](const auto&... mutation) {
        (check_one(mutation), ...);
      },
      std::make_tuple(
          [&](auto* message) {
            message->golf = true;
          },
          [&](auto* message) {
            message->bravo = 22;
          },
          [&](auto* message) {
            message->delta = 2.25;
          },
          [&](auto* message) {
            message->foxtrot = 22.125;
          },
          [&](auto* message) {
            message->india8 = 22;
          },
          [&](auto* message) {
            message->india16 = 22222;
          },
          [&](auto* message) {
            message->india32 = 22222222;
          },
          [&](auto* message) {
            message->india64 = 222222222222222222;
          }));
}

// Check that our encoded 'mike' message is identical to upstream.
GTEST_TEST(RomeoTest, MikeEncode) {
  ASSERT_EQ(papa::mike::getHash(), romeo::mike::getHash());

  papa::mike papa{};
  romeo::mike romeo{};
  ASSERT_TRUE(CompareEncode(papa, romeo));

  // We'll change the struct fields one at a time, and keep checking that both
  // papa and romeo encode identically as we so do. This is the list of changes.
  const auto check_one = [&](const auto& mutation) {
    mutation(&papa);
    mutation(&romeo);
    ASSERT_TRUE(CompareEncode(papa, romeo));
  };
  std::apply(
      [&](const auto&... mutation) {
        (check_one(mutation), ...);
      },
      std::make_tuple(
          [&](auto* message) {
            message->delta[0] = 2.5;
          },
          [&](auto* message) {
            message->foxtrot[3][4] = 1.25;
          }));
}

// Checks that our generated code for `struct lima` can send and receive data.
GTEST_TEST(PapaTest, LimaRoundTrip) {
  papa::lima send{};
  send.golf = true;
  send.bravo = 22;
  send.delta = 2.25;
  send.foxtrot = 22.125;
  send.india8 = 22;
  send.india16 = 22222;
  send.india32 = 22222222;
  send.india64 = 222222222222222222;
  auto data = drake::lcm::EncodeLcmMessage(send);

  auto receive = drake::lcm::DecodeLcmMessage<papa::lima>(data);
  EXPECT_EQ(receive.golf, true);
  EXPECT_EQ(receive.bravo, 22);
  EXPECT_EQ(receive.delta, 2.25);
  EXPECT_EQ(receive.foxtrot, 22.125);
  EXPECT_EQ(receive.india8, 22);
  EXPECT_EQ(receive.india16, 22222);
  EXPECT_EQ(receive.india32, 22222222);
  EXPECT_EQ(receive.india64, 222222222222222222);
}

// Checks that our generated code for `struct mike` can send and receive data.
GTEST_TEST(PapaTest, MikeRoundTrip) {
  papa::mike send{};
  send.delta = {2.5, 22.25, 222.125};
  send.foxtrot[3] = {2222};
  send.alpha.india8 = 22;
  send.rows = 2;
  send.cols = 3;
  send.bravo.resize(send.rows);
  send.india8.resize(send.rows);
  for (auto& item : send.india8) {
    item.resize(send.cols);
  }
  for (auto& item : send.india16) {
    item.resize(send.cols);
  }
  send.india32.resize(send.rows);
  send.xray[1].india8 = 22;
  send.yankee.resize(send.rows);
  send.yankee[1].india16 = 2222;
  send.zulu.resize(send.rows);
  send.zulu[1][0].india32 = 22222;
  auto data = drake::lcm::EncodeLcmMessage(send);

  auto receive = drake::lcm::DecodeLcmMessage<papa::mike>(data);
  EXPECT_EQ(receive.delta.at(2), 222.125);
  EXPECT_EQ(receive.foxtrot.at(3).at(0), 2222);
  EXPECT_EQ(receive.alpha.india8, 22);
  EXPECT_EQ(receive.rows, 2);
  EXPECT_EQ(receive.cols, 3);
  EXPECT_EQ(receive.xray.at(1).india8, 22);
  EXPECT_EQ(receive.yankee.at(1).india16, 2222);
  EXPECT_EQ(receive.zulu.at(1).at(0).india32, 22222);
}

}  // namespace
