#include "drake/common/yaml/yaml_io.h"

#include <fstream>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/test/example_structs.h"

// This unit test only covers the basics of the helper functions in yaml_io.
//
// The exact details of YAML/C++ parsing, matching, and error messages are
// tested in more focused unit tests.

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

GTEST_TEST(YamlIoTest, LoadString) {
  const std::string filename = FindResourceOrThrow(
      "drake/common/yaml/test/yaml_io_test_input_1.yaml");
  const std::string data = ReadTestFileAsString(filename);
  const auto result = LoadYamlString<StringStruct>(data);
  EXPECT_EQ(result.value, "some_value_1");
}

GTEST_TEST(YamlIoTest, LoadStringChildName) {
  const std::string filename = FindResourceOrThrow(
      "drake/common/yaml/test/yaml_io_test_input_2.yaml");
  const std::string data = ReadTestFileAsString(filename);
  const std::string child_name = "some_string_struct";
  const auto result = LoadYamlString<StringStruct>(data, child_name);
  EXPECT_EQ(result.value, "some_value_2");
}

GTEST_TEST(YamlIoTest, LoadStringChildNameBad) {
  const std::string filename = FindResourceOrThrow(
      "drake/common/yaml/test/yaml_io_test_input_2.yaml");
  const std::string data = ReadTestFileAsString(filename);
  const std::string child_name = "wrong_child_name";
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadYamlString<StringStruct>(data, child_name),
      ".* no such .*wrong_child_name.*");
}

GTEST_TEST(YamlIoTest, SaveFile) {
  const std::string filename = temp_directory() + "/SaveFile.yaml";
  const StringStruct data{.value = "save_file_test"};
  SaveYamlFile(filename, data);
  const auto result = ReadTestFileAsString(filename);
  EXPECT_EQ(result, "value: save_file_test\n");
}

GTEST_TEST(YamlIoTest, SaveFileChild) {
  const std::string filename = temp_directory() + "/SaveFileChild.yaml";
  const std::string child_name = "some_string_struct";
  const StringStruct data{.value = "save_file_test"};
  SaveYamlFile(filename, data, child_name);
  const auto result = ReadTestFileAsString(filename);
  EXPECT_EQ(result, "some_string_struct:\n  value: save_file_test\n");
}

GTEST_TEST(YamlIoTest, SaveString) {
  const StringStruct data{.value = "save_string_test"};
  const auto result = SaveYamlString(data);
  EXPECT_EQ(result, "value: save_string_test\n");
}

GTEST_TEST(YamlIoTest, SaveStringChild) {
  const std::string child_name = "some_string_struct";
  const StringStruct data{.value = "save_string_test"};
  const auto result = SaveYamlString(data, child_name);
  EXPECT_EQ(result, "some_string_struct:\n  value: save_string_test\n");
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
