#include "drake/common/yaml/yaml_io.h"

#include <fstream>

#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/test/example_structs.h"

// This unit test only covers the basics of the helper functions in yaml_io.
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
