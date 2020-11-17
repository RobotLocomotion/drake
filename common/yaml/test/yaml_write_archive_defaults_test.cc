// This file tests the yaml_write_archive.h method EraseMatchingMaps(), which
// is used to write out YAML details without repeating the default values.

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
class YamlWriteArchiveDefaultsTest : public ::testing::Test {
 public:
  template <typename DataSerializable, typename DefaultsSerializable>
  static std::string SaveDataWithoutDefaults(
      const DataSerializable& data, const DefaultsSerializable& defaults,
      const std::string& key_name = "doc") {
    YamlWriteArchive defaults_archive;
    defaults_archive.Accept(defaults);
    YamlWriteArchive archive;
    archive.Accept(data);
    archive.EraseMatchingMaps(defaults_archive);
    return archive.EmitString(key_name);
  }
};

// Shows the typical use -- that only the novel data is output.
// The inner_struct is the same for both x and y, so is not output.
TEST_F(YamlWriteArchiveDefaultsTest, BasicExample1) {
  OuterStruct defaults;
  defaults.outer_value = 1.0;
  defaults.inner_struct.inner_value = 2.0;

  OuterStruct data = defaults;
  data.outer_value = 3.0;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  outer_value: 3.0
)""");
}

// Shows the typical use -- that only the novel data is output.
// The outer_value is the same for both x and y, so is not output.
TEST_F(YamlWriteArchiveDefaultsTest, BasicExample2) {
  OuterStruct defaults;
  defaults.outer_value = 1.0;
  defaults.inner_struct.inner_value = 2.0;

  OuterStruct data = defaults;
  data.inner_struct.inner_value = 3.0;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  inner_struct:
    inner_value: 3.0
)""");
}

// Shows the typical use -- emit the content with or without providing a
// root_name.
TEST_F(YamlWriteArchiveDefaultsTest, BasicExample3) {
  OuterStruct defaults;
  defaults.outer_value = 1.0;

  OuterStruct data;
  data.outer_value = 3.0;
  data.inner_struct.inner_value = defaults.inner_struct.inner_value;

  // Emit using the default "doc" root name.
  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  outer_value: 3.0
)""");

  // Emit using an empty root name.
  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults, ""), R"""(outer_value: 3.0
)""");

  // Emit with an empty root name without defaults.
  EXPECT_EQ(SaveDataWithoutDefaults(defaults, defaults, ""), R"""({}
)""");
}

// Same as the BasicExample1 from above, except that the map order of the
// defaults vs data differs.  The defaults still take effect.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentMapOrder1) {
  OuterStructOpposite defaults;
  defaults.inner_struct.inner_value = 1.0;
  defaults.outer_value = 2.0;

  OuterStruct data;
  data.outer_value = 3.0;
  data.inner_struct.inner_value = defaults.inner_struct.inner_value;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  outer_value: 3.0
)""");
}

// Same as the BasicExample2 from above, except that the map order of the
// defaults vs data differs.  The defaults still take effect.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentMapOrder2) {
  OuterStructOpposite defaults;
  defaults.inner_struct.inner_value = 1.0;
  defaults.outer_value = 2.0;

  OuterStruct data;
  data.outer_value = defaults.outer_value;
  data.inner_struct.inner_value = 3.0;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  inner_struct:
    inner_value: 3.0
)""");
}

// YAML nulls are handled reasonably, without throwing.
TEST_F(YamlWriteArchiveDefaultsTest, Nulls) {
  OptionalStruct data;
  data.value = std::nullopt;

  OptionalStruct defaults = data;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
)""");
}

// Arrays differing in their values are not erased.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentArrays) {
  ArrayStruct data;
  data.value = { 1.0, 2.0, 3.0 };

  ArrayStruct defaults;
  defaults.value = { 0.0, 0.0, 0.0 };

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  value: [1.0, 2.0, 3.0]
)""");
}

// Vectors differing in size (but sharing a prefix) are not erased.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentSizeVectors) {
  VectorStruct data;
  data.value = { 1.0, 2.0, 3.0 };

  VectorStruct defaults;
  defaults.value = { 1.0, 2.0 };

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  value: [1.0, 2.0, 3.0]
)""");
}

// Variants differing by tag are not erased.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentVariantTag) {
  VariantStruct data;
  data.value = DoubleStruct{1.0};

  VariantStruct defaults;
  defaults.value = EigenVecStruct();

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  value: !DoubleStruct
    value: 1.0
)""");
}

// Maps differing in key only (same value) are not erased.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentMapKeys) {
  MapStruct data;
  data.value["a"] = 1.0;

  MapStruct defaults;
  defaults.value["b"] = 1.0;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  value:
    a: 1.0
)""");
}

// Maps differing in value only (same key) are not erased.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentMapValues) {
  MapStruct data;
  data.value["a"] = 1.0;

  MapStruct defaults;
  defaults.value["a"] = 2.0;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  value:
    a: 1.0
)""");
}

// In the ununsual case that the user provided two different schemas to
// subtract, we notice the discrepancy without throwing any exceptions.
// This test case serves to help achieve code coverage; we do not intend
// for users to necessarily depend on this behavior.
// Here, we do DoubleStruct - ArrayStruct.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentSchemas) {
  DoubleStruct data;
  data.value = 1.0;

  ArrayStruct defaults;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  value: 1.0
)""");
}

// In the ununsual case that the user provided two different (but similar)
// schemas to subtract, we notice the discrepancy without throwing any
// exceptions.  This test case serves to help achieve code coverage; we do not
// intend for users to necessarily depend on this behavior.  Here, we do
// OuterStruct - OuterWithBlankInner.
TEST_F(YamlWriteArchiveDefaultsTest, DifferentInnerSchemas) {
  OuterStruct data;
  data.outer_value = 1.0;
  data.inner_struct.inner_value = 2.0;

  OuterWithBlankInner defaults;

  EXPECT_EQ(SaveDataWithoutDefaults(data, defaults), R"""(doc:
  outer_value: 1.0
  inner_struct:
    inner_value: 2.0
)""");
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
