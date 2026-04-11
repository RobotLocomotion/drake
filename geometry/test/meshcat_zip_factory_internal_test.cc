#include "drake/geometry/meshcat_zip_factory_internal.h"

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/unused.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

GTEST_TEST(MeshcatZipFactoryInternalTest, Basic) {
  MeshcatZipFactory dut;
  dut.AddFile("README.txt", "Hello, world!");
  const std::string result = dut.Build();
  EXPECT_GT(result.size(), 0);
  EXPECT_THAT(result, testing::HasSubstr("Hello"));
}

GTEST_TEST(MeshcatZipFactoryInternalTest, Empty) {
  const MeshcatZipFactory dut;
  EXPECT_THROW(unused(dut.Build()), std::exception);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
