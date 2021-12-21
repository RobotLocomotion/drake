#include "drake/common/name_deprecator.h"

#include <thread>

#include <gtest/gtest.h>

namespace drake {
namespace internal {
namespace {

GTEST_TEST(NameDeprecatorTest, SmokeTest) {
  NameDeprecator dut;
  std::set<std::string> correct_names({"ca", "cb"});
  EXPECT_FALSE(dut.HasScope());
  dut.DeclareScope(
      "test",
      [&correct_names](const std::string& x) {
        return correct_names.count(x);
      });
  EXPECT_TRUE(dut.HasScope());
  dut.DeclareDeprecatedName("2453-11-11", "da", "ca");
  // Multiple aliases to one target is ok.
  dut.DeclareDeprecatedName("2453-11-11", "dda", "ca");
  dut.DeclareDeprecatedName("2453-11-11", "db", "cb");

  // First requests for deprecated names warn.
  EXPECT_EQ(dut.MaybeTranslate("da"), "ca");
  EXPECT_EQ(dut.MaybeTranslate("dda"), "ca");
  EXPECT_EQ(dut.MaybeTranslate("db"), "cb");

  // Subsequent requests for deprecated names are silent; check the console
  // output for this test to see.
  EXPECT_EQ(dut.MaybeTranslate("da"), "ca");
  EXPECT_EQ(dut.MaybeTranslate("dda"), "ca");
  EXPECT_EQ(dut.MaybeTranslate("db"), "cb");

  // Correct names pass through.
  EXPECT_EQ(dut.MaybeTranslate("cb"), "cb");

  // Random strings pass through.
  EXPECT_EQ(dut.MaybeTranslate("xq"), "xq");
}

GTEST_TEST(NameDeprecatorTest, OncePerProcessTest) {
  // Give multiple threads identical deprecators and see if they emit more than
  // one message. Thread sanitizer builds will be able to detect races; manual
  // inspection of test output can be used to detect leaked duplicates.

  static constexpr int kThreads = 2;
  std::vector<std::thread> threads;
  for (int k = 0; k < kThreads; k++) {
    threads.push_back(
        std::thread([]() {
            std::set<std::string> correct_names({"ca", "cb"});
            NameDeprecator dut;
            dut.DeclareScope(
                "thread_test",
                [&correct_names](const std::string& x) {
                  return correct_names.count(x);
                });
            dut.DeclareDeprecatedName("5432-11-11", "da", "ca");

            static constexpr int kTranslateAttempts = 100;
            for (int n = 0; n < kTranslateAttempts; n++) {
              // Sleep to allow thread actions to interleave.
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
              dut.MaybeTranslate("da");
            }
          }));
  }
    for (auto& thread : threads) {
      thread.join();
    }
}

}  // namespace
}  // namespace internal
}  // namespace drake

