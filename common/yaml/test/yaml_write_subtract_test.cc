#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/yaml/test/example_structs.h"
#include "drake/common/yaml/yaml_write_archive.h"

namespace drake {
namespace yaml {
namespace test {
namespace {

// A test fixture with common helpers.
class YamlWriteSubtractTest : public ::testing::Test {
 public:
  template <typename DataSerializable, typename DefaultsSerializable>
  static std::string SaveSub(
        const DataSerializable& data,
        const DefaultsSerializable& defaults) {
    YamlWriteArchive defaults_archive;
    defaults_archive.Accept(defaults);
    YamlWriteArchive archive;
    archive.Accept(data);
    archive.EraseMatchingMaps(defaults_archive);
    return archive.EmitString("doc");
  }
};

// Shows the typical use -- that only the novel data is output.
// The inner_struct is the same for both x and y, so is not output.
TEST_F(YamlWriteSubtractTest, BasicExample1) {
  OuterStruct defaults;
  defaults.outer_value = 1.0;
  defaults.inner_struct.inner_value = 2.0;

  OuterStruct data = defaults;
  data.outer_value = 3.0;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  outer_value: 3.0
)R");
}

// Shows the typical use -- that only the novel data is output.
// The outer_value is the same for both x and y, so is not output.
TEST_F(YamlWriteSubtractTest, BasicExample2) {
  OuterStruct defaults;
  defaults.outer_value = 1.0;
  defaults.inner_struct.inner_value = 2.0;

  OuterStruct data = defaults;
  data.inner_struct.inner_value = 3.0;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  inner_struct:
    inner_value: 3.0
)R");
}

// Same as the BasicExample1 from above, except that the map order of the
// defaults vs data differs.  The defaults still take effect.
TEST_F(YamlWriteSubtractTest, DifferentMapOrder1) {
  OuterStructOpposite defaults;
  defaults.inner_struct.inner_value = 1.0;
  defaults.outer_value = 2.0;

  OuterStruct data;
  data.outer_value = 3.0;
  data.inner_struct.inner_value = defaults.inner_struct.inner_value;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  outer_value: 3.0
)R");
}

// Same as the BasicExample2 from above, except that the map order of the
// defaults vs data differs.  The defaults still take effect.
TEST_F(YamlWriteSubtractTest, DifferentMapOrder2) {
  OuterStructOpposite defaults;
  defaults.inner_struct.inner_value = 1.0;
  defaults.outer_value = 2.0;

  OuterStruct data;
  data.outer_value = defaults.outer_value;
  data.inner_struct.inner_value = 3.0;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  inner_struct:
    inner_value: 3.0
)R");
}

// YAML nulls are handled reasonably, without throwing.
TEST_F(YamlWriteSubtractTest, Nulls) {
  OptionalStruct data;
  data.value = std::nullopt;

  OptionalStruct defaults = data;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
)R");
}

// Arrays differing in their values are not subtracted.
TEST_F(YamlWriteSubtractTest, DifferentArrays) {
  ArrayStruct data;
  data.value = { 1.0, 2.0, 3.0 };

  ArrayStruct defaults;
  defaults.value = { 0.0, 0.0, 0.0 };

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  value: [1.0, 2.0, 3.0]
)R");
}

// Vectors differing in size (but sharing a prefix) are not subtracted.
TEST_F(YamlWriteSubtractTest, DifferentSizeVectors) {
  VectorStruct data;
  data.value = { 1.0, 2.0, 3.0 };

  VectorStruct defaults;
  defaults.value = { 1.0, 2.0 };

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  value: [1.0, 2.0, 3.0]
)R");
}

// Variants differing by tag are not subtracted.
TEST_F(YamlWriteSubtractTest, DifferentVariantTag) {
  VariantStruct data;
  data.value = DoubleStruct{1.0};

  VariantStruct defaults;
  defaults.value = StringStruct{"1.0"};

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  value: !DoubleStruct
    value: 1.0
)R");
}

// Maps differing in key only (same value) are not subtracted.
TEST_F(YamlWriteSubtractTest, DifferentMapKeys) {
  MapStruct data;
  data.value["a"] = 1.0;

  MapStruct defaults;
  defaults.value["b"] = 1.0;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  value:
    a: 1.0
)R");
}

// Maps differing in value only (same key) are not subtracted.
TEST_F(YamlWriteSubtractTest, DifferentMapValues) {
  MapStruct data;
  data.value["a"] = 1.0;

  MapStruct defaults;
  defaults.value["a"] = 2.0;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  value:
    a: 1.0
)R");
}

// In the ununsual case that the user provided two different schemas to
// subtract, we notice the discrepancy without throwing any exceptions.
// Here, we do DoubleStruct - ArrayStruct.
TEST_F(YamlWriteSubtractTest, DifferentSchemas) {
  DoubleStruct data;
  data.value = 1.0;

  ArrayStruct defaults;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  value: 1.0
)R");
}

// In the ununsual case that the user provided two different (but similar)
// schemas to subtract, we notice the discrepancy without throwing any
// exceptions.  Here, we do OuterStruct - OuterWithBlankInner.
TEST_F(YamlWriteSubtractTest, DifferentInnerSchemas) {
  OuterStruct data;
  data.outer_value = 1.0;
  data.inner_struct.inner_value = 2.0;

  OuterWithBlankInner defaults;

  EXPECT_EQ(SaveSub(data, defaults), R"R(doc:
  outer_value: 1.0
  inner_struct:
    inner_value: 2.0
)R");
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
