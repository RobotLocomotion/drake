#include "drake/common/file_source.h"

#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/memory_file.h"
#include "drake/common/name_value.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace {

namespace fs = std::filesystem;

/* We want to lock down the idea that the default value is an empty path. */
GTEST_TEST(FileSourceTest, DefaultPath) {
  FileSource dut;
  EXPECT_TRUE(std::holds_alternative<std::filesystem::path>(dut));
}

GTEST_TEST(FileSourceTest, ToString) {
  EXPECT_EQ(to_string(FileSource("a/b/c")), "\"a/b/c\"");
  EXPECT_EQ(fmt::to_string(FileSource("a/b/c")), "\"a/b/c\"");

  const MemoryFile file("012345789", ".ext", "hint");
  EXPECT_EQ(to_string(FileSource(file)), file.to_string());
  EXPECT_EQ(fmt::to_string(FileSource(file)), file.to_string());
}

/* Quick and dirty struct that has a FileSource and can be serialized. */
struct HasFileSource {
  template <typename Archive>
  void Serialize(Archive* archive) {
    archive->Visit(DRAKE_NVP(source));
  }
  FileSource source;
};

/* The path value gets (de)serialized. */
GTEST_TEST(FileSourceTest, SerializePath) {
  const HasFileSource dut{.source = fs::path("/some/path")};
  const std::string y = yaml::SaveYamlString(dut);
  const auto decoded = yaml::LoadYamlString<HasFileSource>(y);
  ASSERT_TRUE(std::holds_alternative<fs::path>(decoded.source));
  EXPECT_EQ(std::get<fs::path>(dut.source), std::get<fs::path>(decoded.source));
}

/* The MemoryFile value gets (de)serialized. */
GTEST_TEST(FileSourceTest, SerializeMemoryFile) {
  const HasFileSource dut{.source = MemoryFile("stuff", ".ext", "hint")};
  const std::string y = yaml::SaveYamlString(dut);
  const auto decoded = yaml::LoadYamlString<HasFileSource>(y);
  ASSERT_TRUE(std::holds_alternative<MemoryFile>(decoded.source));
  EXPECT_EQ(std::get<MemoryFile>(dut.source).contents(),
            std::get<MemoryFile>(decoded.source).contents());
}

}  // namespace
}  // namespace drake
