#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/test/example_structs.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace yaml {
namespace test {
namespace {

GTEST_TEST(YamlJsonTest, WriteScalars) {
  AllScalarsStruct data;
  constexpr char expected[] =
      R"""({"some_bool":false,"some_double":1.2345,"some_float":1.2345,"some_int32":12,"some_int64":14,"some_string":"kNominalString","some_uint32":12,"some_uint64":15})""";
  EXPECT_EQ(SaveJsonString(data), expected);
}

GTEST_TEST(YamlJsonTest, StringEscaping) {
  StringStruct data;
  data.value = "foo\n\"bar\"\t";
  EXPECT_EQ(SaveJsonString(data),
            R"""({"value":"foo\u000a\u0022bar\u0022\u0009"})""");
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

  // It would be plausible here to use the `_tag` convention to annotate
  // variant tags, matching what we do in our yaml.py conventions.
  data.value = DoubleStruct{};
  DRAKE_EXPECT_THROWS_MESSAGE(SaveJsonString(data),
                              ".*SaveJsonString.*mapping.*tag.*");
}

struct IntVariant {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  std::variant<int, uint64_t> value{0};
};

GTEST_TEST(YamlJsonTest, WriteVariantScalar) {
  IntVariant data;

  data.value.emplace<int>(1);
  EXPECT_EQ(SaveJsonString(data), R"""({"value":1})""");

  data.value.emplace<uint64_t>(22);
  DRAKE_EXPECT_THROWS_MESSAGE(SaveJsonString(data),
                              ".*SaveJsonString.*scalar.*22.*tag.*");
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
