#include "drake/common/fmt_ostream.h"

#include <gtest/gtest.h>

// Sample code that shows how to shim Drake code that defines an operator<< to
// interface with fmt >= 9 semantics. The ostream_formatter is intended to be a
// porting shim for #17742 and will probably be deleted eventually, once all
// of Drake is using fmt pervasively.
namespace sample {
class UnportedStreamable {
 public:
  friend std::ostream& operator<<(std::ostream& os, const UnportedStreamable&) {
    return os << "US";
  }
};
}  // namespace sample
namespace fmt {
template <>
struct formatter<sample::UnportedStreamable> : drake::ostream_formatter {};
}  // namespace fmt

// Sample code that shows how to print a third-party type that only supports
// operator<< and not fmt::formatter. We plan to keep this sugar indefinitely,
// since many third-party types will never know about fmt::formatter.
namespace sample {
class ThirdPartyStreamable {
 public:
  ThirdPartyStreamable() = default;

  // Non-copyable.
  ThirdPartyStreamable(const ThirdPartyStreamable&) = delete;

  friend std::ostream& operator<<(std::ostream& os,
                                  const ThirdPartyStreamable&) {
    return os << "TPS";
  }
};
}  // namespace sample

namespace drake {
namespace {

GTEST_TEST(FmtOstreamTest, UnportedOstreamShim) {
  const sample::UnportedStreamable value;
  EXPECT_EQ(fmt::format("{}", value), "US");
  EXPECT_EQ(fmt::format("{:4}", value), "US  ");
  EXPECT_EQ(fmt::format("{:>4}", value), "  US");
}

GTEST_TEST(FmtOstreamTest, ThirdPartyOstreamShim) {
  const sample::ThirdPartyStreamable value;
  EXPECT_EQ(fmt::format("{}", fmt_streamed(value)), "TPS");
  EXPECT_EQ(fmt::format("{:4}", fmt_streamed(value)), "TPS ");
  EXPECT_EQ(fmt::format("{:>4}", fmt_streamed(value)), " TPS");
}

}  // namespace
}  // namespace drake
