#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace yaml {
namespace test {
namespace {

struct Float32Struct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  float value{};
};

// Test that the convert<> deprecation for YamlReadArchive still
// works as intended.  On 2022-03-01 when deprecated yaml code is
// removed, this entire test file can be removed as well.
GTEST_TEST(YamlReadArchiveDeprecatedTest, LoadCustomScalar) {
  const std::string data = "value: 2.0";
  const auto result = LoadYamlString<Float32Struct>(data);
  EXPECT_EQ(result.value, 2.0f);
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
