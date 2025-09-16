#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/yaml/yaml_io.h"

// This is a unit test that exactly replicates the examples in yaml_doxygen.h,
// to ensure that they remain correct. Any changes to this file or the doxygen
// file must be mirrored into the other file.

namespace drake {
namespace yaml {
namespace test {
namespace {

// (This is a test helper, not part of the Doxygen header.)
std::string GetTempFilename() {
  return temp_directory() + "/filename.yaml";
}

// Write data to a scratch file and then return the temp's filename.
// (This is a test helper, not part of the Doxygen header.)
std::string WriteTemp(const std::string& data) {
  const std::string filename = GetTempFilename();
  std::ofstream output(filename);
  output << data;
  return filename;
}

// This is an example from the Doxygen header.
struct MyData {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(foo));
    a->Visit(DRAKE_NVP(bar));
  }
  double foo{0.0};
  std::vector<double> bar;
};

GTEST_TEST(YamlDoxygenTest, ExamplesLoading) {
  // This data is an example from the Doxygen header.
  const std::string input = R"""(
foo: 1.0
bar: [2.0, 3.0]
)""";
  auto input_filename = WriteTemp(input);
  std::stringstream std_cout;

  // This code is an example from the Doxygen header.
  const MyData data = LoadYamlFile<MyData>(input_filename);
  std_cout << fmt::format("foo = {:.1f}\n", data.foo);
  std_cout << fmt::format("bar = {:.1f}\n", fmt::join(data.bar, ", "));

  const std::string expected_output = R"""(
foo = 1.0
bar = 2.0, 3.0
)""";
  EXPECT_EQ(std_cout.str(), expected_output.substr(1));
}

GTEST_TEST(YamlDoxygenTest, ExamplesSaving) {
  const std::string output_filename = GetTempFilename();

  // This code is an example from the Doxygen header.
  MyData data{4.0, {5.0, 6.0}};
  SaveYamlFile(output_filename, data);

  // This data is an example from the Doxygen header.
  const std::string output = ReadFileOrThrow(output_filename);
  const std::string expected_output = R"""(
foo: 4.0
bar: [5.0, 6.0]
)""";
  EXPECT_EQ(output, expected_output.substr(1));
}

// This is an example from the Doxygen header.
struct MoreData {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(baz));
    a->Visit(DRAKE_NVP(quux));
  }

  std::string baz;
  std::map<std::string, MyData> quux;
};

// This is an example from the Doxygen header.
MyData LoadMyData(const std::string& filename) {
  return LoadYamlFile<MyData>(filename);
}

GTEST_TEST(YamlDoxygenTest, ReadingYamlFiles) {
  // This data is an example from the Doxygen header.
  const std::string input = R"""(
foo: 1.0
bar: [2.0, 3.0]
)""";
  auto input_filename = WriteTemp(input);
  std::stringstream std_cout;

  // This code is an example from the Doxygen header.
  const MyData data = LoadMyData(input_filename);
  std_cout << fmt::format("foo = {:.1f}\n", data.foo);
  std_cout << fmt::format("bar = {:.1f}\n", fmt::join(data.bar, ", "));

  // This data is an example from the Doxygen header.
  const std::string expected_output = R"""(
foo = 1.0
bar = 2.0, 3.0
)""";
  EXPECT_EQ(std_cout.str(), expected_output.substr(1));
}

// This is an example from the Doxygen header.
MyData LoadMyData2(const std::string& filename) {
  return LoadYamlFile<MyData>(filename, "data_2");
}

GTEST_TEST(YamlDoxygenTest, TopLevelChild) {
  // This data is an example from the Doxygen header.
  const std::string input = R"""(
data_1:
  foo: 1.0
  bar: [2.0, 3.0]
data_2:
  foo: 4.0
  bar: [5.0, 6.0]
)""";
  auto input_filename = WriteTemp(input);
  std::stringstream std_cout;

  const MyData data = LoadMyData2(input_filename);
  std_cout << fmt::format("foo = {:.1f}\n", data.foo);
  std_cout << fmt::format("bar = {:.1f}\n", fmt::join(data.bar, ", "));

  // This data is an example from the Doxygen header.
  const std::string expected_output = R"""(
foo = 4.0
bar = 5.0, 6.0
)""";
  EXPECT_EQ(std_cout.str(), expected_output.substr(1));
}

GTEST_TEST(YamlDoxygenTest, MergeKeys) {
  // This data is an example from the Doxygen header.
  const std::string input = R"""(
_template: &common_foo
  foo: 1.0
data_1:
  << : *common_foo
  bar: [2.0, 3.0]
data_2:
  << : *common_foo
  bar: [5.0, 6.0]
)""";
  auto input_filename = WriteTemp(input);
  const MyData data = LoadMyData2(input_filename);
  EXPECT_EQ(data.foo, 1.0);
  EXPECT_EQ(data.bar, std::vector({5.0, 6.0}));
}

GTEST_TEST(YamlDoxygenTest, WritingYamlFiles) {
  std::stringstream std_cout;

  // This code is an example from the Doxygen header.
  MyData data{1.0, {2.0, 3.0}};
  std_cout << SaveYamlString(data, "root");

  // This data is an example from the Doxygen header.
  const std::string expected_output = R"""(
root:
  foo: 1.0
  bar: [2.0, 3.0]
)""";
  EXPECT_EQ(std_cout.str(), expected_output.substr(1));
}

// This is an example from the Doxygen header.
std::map<std::string, MyData> LoadAllMyData(const std::string& filename) {
  return LoadYamlFile<std::map<std::string, MyData>>(filename);
}

GTEST_TEST(YamlDoxygenTest, DocumentRoot) {
  // This data is an example from the Doxygen header.
  const std::string input = R"""(
data_1:
  foo: 1.0
  bar: [2.0, 3.0]
data_2:
  foo: 4.0
  bar: [5.0, 6.0]
)""";
  auto input_filename = WriteTemp(input);
  auto data = LoadAllMyData(input_filename);
  EXPECT_EQ(data.size(), 2);
  EXPECT_EQ(data.at("data_2").foo, 4.0);
}

// This is an example from the Doxygen header.
struct Foo {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(data));
  }

  std::string data;
};

// This is an example from the Doxygen header.
struct Bar {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  std::variant<std::string, double, Foo> value;
};

GTEST_TEST(YamlDoxygenTest, NullableTypes) {
  // This data is an example from the Doxygen header.
  const std::string input = R"""(
bar:
  value: hello
bar2:
  value: !!str hello
bar3:
  value: !!float 1.0
bar4:
  value: !Foo
    data: hello
)""";

  auto bar = LoadYamlString<Bar>(input, "bar");
  ASSERT_EQ(bar.value.index(), 0);
  EXPECT_EQ(std::get<std::string>(bar.value), "hello");

  auto bar2 = LoadYamlString<Bar>(input, "bar2");
  ASSERT_EQ(bar2.value.index(), 0);
  EXPECT_EQ(std::get<std::string>(bar2.value), "hello");

  auto bar3 = LoadYamlString<Bar>(input, "bar3");
  ASSERT_EQ(bar3.value.index(), 1);
  EXPECT_EQ(std::get<double>(bar3.value), 1.0);

  auto bar4 = LoadYamlString<Bar>(input, "bar4");
  ASSERT_EQ(bar4.value.index(), 2);
  EXPECT_EQ(std::get<Foo>(bar4.value).data, "hello");
}

}  // namespace
}  // namespace test
}  // namespace yaml
}  // namespace drake
