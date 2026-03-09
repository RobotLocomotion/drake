#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/test/example_structs.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace yaml {
namespace test {
namespace {

GTEST_TEST(YamlJsonTest, WriteScalars) {
  AllScalarsStruct data{.include_bytes = false};
  EXPECT_EQ(SaveJsonString(data), R"""({"some_bool":false,)"""
                                  R"""("some_double":1.2345,)"""
                                  R"""("some_float":1.2345,)"""
                                  R"""("some_int32":12,)"""
                                  R"""("some_int64":14,)"""
                                  R"""("some_path":"/path/to/nowhere",)"""
                                  R"""("some_string":"kNominalString",)"""
                                  R"""("some_uint32":12,)"""
                                  R"""("some_uint64":15})""");
}

GTEST_TEST(YamlJsonTest, BinaryScalarThrows) {
  BytesStruct data;
  DRAKE_EXPECT_THROWS_MESSAGE(SaveJsonString(data),
                              ".*Cannot save.*scalar.*with.*binary.*");
}

GTEST_TEST(YamlJsonTest, StringEscaping) {
  StringStruct data;
  data.value = "foo\n\t\"bar";
  EXPECT_EQ(SaveJsonString(data),
            R"""({"value":"foo\u000a\u0009\u0022bar"})""");
}

GTEST_TEST(YamlJsonTest, NonFinite) {
  DoubleStruct data;

  data.value = std::numeric_limits<double>::infinity();
  EXPECT_EQ(SaveJsonString(data), R"""({"value":Infinity})""");

  data.value = -data.value;
  EXPECT_EQ(SaveJsonString(data), R"""({"value":-Infinity})""");

  data.value = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(SaveJsonString(data), R"""({"value":NaN})""");
}

GTEST_TEST(YamlJsonTest, WriteSequence) {
  VectorStruct data;
  data.value = {1.0, 2.0};
  EXPECT_EQ(SaveJsonString(data), R"""({"value":[1.0,2.0]})""");
}

GTEST_TEST(YamlJsonTest, WriteMapping) {
  MapStruct data;
  data.value.clear();
  data.value["a"] = 1.0;
  data.value["b"] = 2.0;
  EXPECT_EQ(SaveJsonString(data), R"""({"value":{"a":1.0,"b":2.0}})""");
}

GTEST_TEST(YamlJsonTest, WriteNested) {
  OuterStruct data;
  EXPECT_EQ(
      SaveJsonString(data),
      R"""({"inner_struct":{"inner_value":1.2345},"outer_value":1.2345})""");
}

GTEST_TEST(YamlJsonTest, WriteOptional) {
  OptionalStruct data;
  EXPECT_EQ(SaveJsonString(data), R"""({"value":1.2345})""");

  data.value.reset();
  EXPECT_EQ(SaveJsonString(data), "{}");
}

GTEST_TEST(YamlJsonTest, WriteVariant) {
  VariantStruct data;

  data.value = std::string{"foo"};
  EXPECT_EQ(SaveJsonString(data), R"""({"value":"foo"})""");

  data.value = 0.5;
  EXPECT_EQ(SaveJsonString(data), R"""({"value":0.5})""");

  // It would be plausible here to use the `_tag` convention to annotate
  // variant tags, matching what we do in our yaml.py conventions.
  data.value = DoubleStruct{};
  DRAKE_EXPECT_THROWS_MESSAGE(SaveJsonString(data),
                              ".*SaveJsonString.*mapping.*tag.*");
}

GTEST_TEST(YamlJsonTest, FileRoundTrip) {
  DoubleStruct data;
  data.value = 22.25;
  const std::string filename = temp_directory() + "/file_round_trip.json";
  SaveJsonFile(filename, data);
  const auto readback = LoadYamlFile<DoubleStruct>(filename);
  EXPECT_EQ(readback.value, 22.25);
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
