#include "drake/common/yaml/yaml_io.h"

#include <fstream>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/test/example_structs.h"

// This unit test only covers the basics of the helper functions in yaml_io,
// as well as (indirectly) the helper functions on YamlReadArchive related
// to file document loading (LoadFileAsNode, LoadStringAsNode).
//
// The exact details of YAML/C++ parsing, matching, and error messages are
// tested in more focused unit tests of the Archive classes.

namespace drake {
namespace yaml {
namespace test {
namespace {

std::string ReadTestFileAsString(const std::string& filename) {
  std::ifstream input(filename, std::ios::binary);
  DRAKE_DEMAND(!input.fail());
  std::stringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

GTEST_TEST(YamlIoTest, LoadString) {
  const std::string data = R"""(
value:
  some_value
)""";
  const auto result = LoadYamlString<StringStruct>(data);
  EXPECT_EQ(result.value, "some_value");
}

GTEST_TEST(YamlIoTest, LoadStringChildName) {
  const std::string data = R"""(
some_child_name:
  value:
    some_value
)""";
  const std::string child_name = "some_child_name";
  const auto result = LoadYamlString<StringStruct>(data, child_name);
  EXPECT_EQ(result.value, "some_value");
  // When the requested child_name does not exist, that's an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadYamlString<StringStruct>(data, "wrong_child_name"),
      ".* no such .* map entry .*wrong_child_name.*");
}

GTEST_TEST(YamlIoTest, LoadStringDefaults) {
  const std::string data = R"""(
value:
  some_key: 1.0
)""";
  const std::optional<std::string> no_child_name;
  const MapStruct defaults;  // The defaults contains kNominalDouble already.
  const auto result = LoadYamlString<MapStruct>(data, no_child_name, defaults);
  EXPECT_EQ(result.value.at("some_key"), 1.0);
  EXPECT_EQ(result.value.at("kNominalDouble"), test::kNominalDouble);
  EXPECT_EQ(result.value.size(), 2);
}

GTEST_TEST(YamlIoTest, LoadStringOptions) {
  const std::string data = R"""(
value: some_value
extra_junk: will_be_ignored
)""";
  const std::optional<std::string> no_child_name;
  const std::optional<StringStruct> no_defaults;
  YamlReadArchive::Options options;
  options.allow_yaml_with_no_cpp = true;
  const auto result = LoadYamlString<StringStruct>(
      data, no_child_name, no_defaults, options);
  EXPECT_EQ(result.value, "some_value");
  // Cross-check that the options actually were important.
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadYamlString<StringStruct>(
          data, no_child_name, no_defaults, { /* no options */ }),
      ".*extra_junk.*");
}

GTEST_TEST(YamlIoTest, LoadStringDefaultsAndOptions) {
  const std::string data = R"""(
value:
  some_key: 1.0
extra_junk:
  will_be_ignored: 2.0
)""";
  const std::optional<std::string> no_child_name;
  const MapStruct defaults;  // The defaults contains kNominalDouble already.
  YamlReadArchive::Options options;
  options.allow_yaml_with_no_cpp = true;
  const auto result = LoadYamlString<MapStruct>(
      data, no_child_name, defaults, options);
  // When user options are provided, the implicit options that would otherwise
  // be used for defaults parsing are not in effect; thus, retain_map_defaults
  // will be false and kNominalDouble is not present in the result.
  EXPECT_EQ(result.value.at("some_key"), 1.0);
  EXPECT_EQ(result.value.size(), 1);
}

GTEST_TEST(YamlIoTest, LoadFile) {
  const std::string filename = FindResourceOrThrow(
      "drake/common/yaml/test/yaml_io_test_input_1.yaml");
  const auto result = LoadYamlFile<StringStruct>(filename);
  EXPECT_EQ(result.value, "some_value_1");
}

GTEST_TEST(YamlIoTest, LoadFileChildName) {
  const std::string filename = FindResourceOrThrow(
      "drake/common/yaml/test/yaml_io_test_input_2.yaml");
  const std::string child_name = "some_string_struct";
  const auto result = LoadYamlFile<StringStruct>(filename, child_name);
  EXPECT_EQ(result.value, "some_value_2");
}

GTEST_TEST(YamlIoTest, LoadFileChildNameBad) {
  const std::string filename = FindResourceOrThrow(
      "drake/common/yaml/test/yaml_io_test_input_2.yaml");
  const std::string child_name = "wrong_child_name";
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadYamlFile<StringStruct>(filename, child_name),
      ".*yaml_io_test_input_2.yaml.* no such .*wrong_child_name.*");
}

// Because we know that the LoadFile and LoadString implementations share a
// helper function that implements handling of defaults and options, these
// specific variations of File-based test cases are not needed because the
// LoadString cases already covered them:
//
// - LoadFileDefaults
// - LoadFileOptions
// - LoadFileDefaultsAndOptions

GTEST_TEST(YamlIoTest, SaveString) {
  const StringStruct data{.value = "save_string"};
  const auto result = SaveYamlString(data);
  EXPECT_EQ(result, "value: save_string\n");
}

GTEST_TEST(YamlIoTest, SaveStringChild) {
  const std::string child_name = "some_child";
  const StringStruct data{.value = "save_string_child"};
  const auto result = SaveYamlString(data, child_name);
  EXPECT_EQ(result, "some_child:\n  value: save_string_child\n");
}

GTEST_TEST(YamlIoTest, SaveStringDefaults) {
  const std::optional<std::string> no_child_name;
  const MapStruct defaults;
  MapStruct data;  // The data.value contains kNominalDouble already.
  data.value.insert({"save_string_defaults", 1.0});
  ASSERT_EQ(data.value.size(), 2);
  const auto result = SaveYamlString(data, no_child_name, {defaults});
  // Only the non-default map entry is saved.
  EXPECT_EQ(result, "value:\n  save_string_defaults: 1.0\n");
}

// The implemenation of SaveYamlFile calls SaveYamlString, so we only need
// to lightly test it (specifically the file-writing function).  We'll do
// one test case with minimal arguments (just the filename) and one test
// case with all arguments (to confirm that they are all forwarded).

GTEST_TEST(YamlIoTest, SaveFile) {
  const std::string filename = temp_directory() + "/SaveFile1.yaml";
  const StringStruct data{.value = "save_file_1"};
  SaveYamlFile(filename, data);
  const auto result = ReadTestFileAsString(filename);
  EXPECT_EQ(result, "value: save_file_1\n");
}

GTEST_TEST(YamlIoTest, SaveFileAllArgs) {
  const std::string filename = temp_directory() + "/SaveFile4.yaml";
  const std::string child_name = "some_child";
  const MapStruct defaults;
  MapStruct data;  // The data.value contains kNominalDouble already.
  data.value.insert({"save_file_4", 1.0});
  ASSERT_EQ(data.value.size(), 2);
  SaveYamlFile(filename, data, child_name, {defaults});
  const auto result = ReadTestFileAsString(filename);
  EXPECT_EQ(result, "some_child:\n  value:\n    save_file_4: 1.0\n");
}

GTEST_TEST(YamlIoTest, SaveFileBad) {
  const std::string filename = temp_directory() + "/no_such_dir/file.yaml";
  DRAKE_EXPECT_THROWS_MESSAGE(
      SaveYamlFile(filename, StringStruct{}),
      ".* could not open .*/no_such_dir/.*");
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
