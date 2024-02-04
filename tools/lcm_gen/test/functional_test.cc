#include <gmock/gmock.h>
#include <gtest/gtest.h>

// The "papa" (i.e, "probed") messages are generated by our lcm_gen tool.
// These are the "classes under test" for this file.

// The "romeo" (i.e., "reference") messages are generated by the upstream
// lcm-gen tool. Here, we use those as an "oracle" to compare against.

// clang-format off
#include "papa/lima.h"
#include "papa/november.h"
#include "romeo/lima.hpp"
#include "romeo/november.hpp"
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

// Compares the encoded bytes for two messages. The messages are same type in
// the sense that they are the same LCM message definition, but the C++ types
// will actually be different -- one for our own lcm_gen tool and one for the
// upstream lcm_gen tool.
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
  for (size_t i = 0; i < actual_bytes.size() && i < expected_bytes.size();
       ++i) {
    if (actual_bytes[i] != expected_bytes[i]) {
      EXPECT_EQ(actual_bytes[i], expected_bytes[i]) << i << "th message byte";
      break;
    }
  }

  // If this fails, the both array's entire contents will be printed.
  EXPECT_EQ(actual_bytes, expected_bytes);

  return actual_bytes == expected_bytes;
}

// Check that our encoded 'lima' message is identical to upstream.
GTEST_TEST(RomeoTest, LimaEncode) {
  // The hash is the same.
  ASSERT_EQ(papa::lima::getHash(), romeo::lima::getHash());

  // Default-constructed messages encode the same.
  papa::lima papa{};
  romeo::lima romeo{};
  ASSERT_TRUE(CompareEncode(papa, romeo));

  // We'll change the struct fields one at a time, and keep checking that both
  // papa and romeo encode the same as we so do.
  // clang-format off
  auto mutations = std::make_tuple(
      [](auto* message) { message->golf = true; },
      [](auto* message) { message->bravo = 22; },
      [](auto* message) { message->delta = 2.25; },
      [](auto* message) { message->foxtrot = 22.125; },
      [](auto* message) { message->india8 = 22; },
      [](auto* message) { message->india16 = 22222; },
      [](auto* message) { message->india32 = 22222222; },
      [](auto* message) { message->india64 = 222222222222222222; });
  // clang-format on
  const auto check_one = [&](const auto& mutation) {
    EXPECT_NO_THROW(mutation(&papa));
    EXPECT_NO_THROW(mutation(&romeo));
    ASSERT_TRUE(CompareEncode(papa, romeo));
  };
  std::apply(
      [&](const auto&... mutation) {
        (check_one(mutation), ...);
      },
      mutations);
}

// Check that our generated code for `struct lima` can send and receive data.
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

  // When the received data is cut short, the message detects the error.
  data.resize(data.size() - 1);
  EXPECT_EQ(receive.decode(data.data(), 0, data.size()), -1);
  // When the send buffer isn't large enough, the message detects the error.
  EXPECT_EQ(send.encode(data.data(), 0, data.size()), -1);
}

// Check that our encoded 'november' message is identical to upstream.
GTEST_TEST(RomeoTest, NovemberEncode) {
  // The hash is the same.
  ASSERT_EQ(papa::november::getHash(), romeo::november::getHash());

  // Default-constructed messages encode the same.
  papa::november papa{};
  romeo::november romeo{};
  ASSERT_TRUE(CompareEncode(papa, romeo));

  // We'll change the struct fields one at a time, and keep checking that both
  // papa and romeo encode the same as we so do.
  // clang-format off
  auto mutations = std::make_tuple(
      [](auto* message) { message->alpha.india8 = 22; },
      [](auto* message) { message->bravo.india16 = 2222; },
      [](auto* message) { message->charlie = 22222222; });
  // clang-format on
  const auto check_one = [&](const auto& mutation) {
    EXPECT_NO_THROW(mutation(&papa));
    EXPECT_NO_THROW(mutation(&romeo));
    ASSERT_TRUE(CompareEncode(papa, romeo));
  };
  std::apply(
      [&](const auto&... mutation) {
        (check_one(mutation), ...);
      },
      mutations);
}

// Check that our generated code for `struct november` can send and receive
// data.
GTEST_TEST(PapaTest, NovemberRoundTrip) {
  papa::november send{};
  send.alpha.india8 = 22;
  send.bravo.india16 = 2222;
  send.charlie = 22222222;
  auto data = drake::lcm::EncodeLcmMessage(send);

  auto receive = drake::lcm::DecodeLcmMessage<papa::november>(data);
  EXPECT_EQ(receive.alpha.india8, 22);
  EXPECT_EQ(receive.bravo.india16, 2222);
  EXPECT_EQ(receive.charlie, 22222222);

  // When the received data is cut short, the message detects the error.
  data.resize(data.size() - 1);
  EXPECT_EQ(receive.decode(data.data(), 0, data.size()), -1);
  // When the send buffer isn't large enough, the message detects the error.
  EXPECT_EQ(send.encode(data.data(), 0, data.size()), -1);
}

// TODO(jwnimmer-tri) We're currently missing unit test coverage for messages
// that can nest inside themselves (i.e., recursive message definitions). See
// "We avoid recursions by setting C([list]) = 0 if [list] contains C." from
// https://lcm-proj.github.io/lcm/content/lcm-type-ref.html#fingerprint-computation
// Our codegen probably does this correctly already, but we should test it.

}  // namespace
