#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/name_value.h"
#include "drake/common/yaml/test/example_structs.h"
#include "drake/common/yaml/yaml_write_archive.h"

namespace drake {
namespace yaml {
namespace test {
namespace {

// A test fixture with common helpers.
class YamlWriteSubtractTest : public ::testing::Test {
 public:
  template <typename Serializable>
  static std::string SaveSub(
        const Serializable& data,
        const Serializable& defaults) {
    YamlWriteArchive defaults_archive;
    defaults_archive.Accept(defaults);
    YamlWriteArchive archive;
    archive.Accept(data);
    archive.SubtractDefaults(defaults_archive);
    return archive.EmitString("doc");
  }
};

TEST_F(YamlWriteSubtractTest, Basic) {
  OuterStruct y;
  y.outer_value = 1.0;
  y.inner_struct.inner_value = 2.0;

  OuterStruct x = y;
  x.outer_value = 3.0;

  EXPECT_EQ(SaveSub(x, y), "doc:\n  outer_value: 3.0\n");
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
