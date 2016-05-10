#include <iostream>
#include <sstream>
#include <map>

#include "gtest/gtest.h"

#include "drake/util/Polynomial.h"
#include "drake/util/TrigPoly.h"

using namespace Eigen;
using namespace std;

namespace drake {
namespace util {
namespace {

void test_serialization(TrigPolyd dut, std::string expect) {
  std::stringstream test_stream;
  test_stream << dut;
  EXPECT_EQ(test_stream.str(), expect);
}

TEST(TrigPolyTest, SmokeTest) {
  // Confirm that these conversions compile okay.
  TrigPolyd x(1.0);
  TrigPolyd y = 2.0;
  TrigPolyd z = 3;

  // Test something else.
  Polynomiald q("q");
  Polynomiald s("s");
  Polynomiald c("c");

  TrigPolyd p(q, s, c);

  test_serialization(p, "q1");
  test_serialization(sin(p), "s1");
  test_serialization(cos(p), "c1");
  test_serialization(sin(p) * p * p + cos(p), "s1*q1^2+c1");
  test_serialization(sin(p + p), "s1*c1+c1*s1");
}

}  // anonymous namespace
}  // namespace test
}  // namespace drake
