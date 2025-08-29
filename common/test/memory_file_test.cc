#include "drake/common/memory_file.h"

#include <fstream>
#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace {

GTEST_TEST(MemoryFileTest, DefaultConstruct) {
  const MemoryFile contents;
  EXPECT_TRUE(contents.contents().empty());
  EXPECT_TRUE(contents.extension().empty());
  EXPECT_TRUE(contents.filename_hint().empty());
  EXPECT_EQ(contents.sha256(), Sha256::Checksum(""));
}

GTEST_TEST(MemoryFileTest, BasicApi) {
  const std::string content_str = R"""(Some arbitrary
                contents with newlines and whatnot.)""";
  const std::string filename = "Just some file";
  const std::string ext = ".FOo";
  const MemoryFile contents(content_str, ext, filename);

  EXPECT_EQ(contents.contents(), content_str);
  EXPECT_EQ(contents.extension(), ".foo");
  EXPECT_EQ(contents.filename_hint(), filename);
  EXPECT_EQ(contents.sha256(), Sha256::Checksum(content_str));

  EXPECT_THROW(MemoryFile("a", ".b", "hint\nnewline"), std::exception);
}

GTEST_TEST(MemoryFileTest, ContentShaMatch) {
  const std::string contents = "This is a test";
  MemoryFile file(contents, ".txt", "test contents");
  EXPECT_EQ(file.contents(), contents);
  EXPECT_EQ(file.sha256(), Sha256::Checksum(file.contents()));

  MemoryFile file2(std::move(file));
  EXPECT_EQ(file2.contents(), contents);
  EXPECT_EQ(file2.sha256(), Sha256::Checksum(file2.contents()));

  // We don't have any promises about what the contents or hash is after moving.
  // We do require that the hash of its contents matches its reported hash.
  EXPECT_EQ(file.sha256(), Sha256::Checksum(file.contents()));
}

GTEST_TEST(MemoryFileTest, Extension) {
  EXPECT_THROW(MemoryFile("contents", "no_dot", "hint"), std::exception);
  EXPECT_NO_THROW(MemoryFile("contents", "", "hint"));
}

GTEST_TEST(MemoryFileTest, StaticMethods) {
  const std::string path =
      FindResourceOrThrow("drake/common/test/find_resource_test_data.txt");
  const MemoryFile memory_file = MemoryFile::Make(path);

  const std::string contents = ReadFileOrThrow(path);
  const MemoryFile ref(contents, ".txt", path);

  EXPECT_EQ(memory_file.contents(), ref.contents());
  EXPECT_EQ(memory_file.extension(), ref.extension());
  EXPECT_EQ(memory_file.sha256(), ref.sha256());
  EXPECT_EQ(memory_file.filename_hint(), ref.filename_hint());

  const std::filesystem::path temp_dir = temp_directory();
  const std::filesystem::path temp_file = temp_dir / "no_extension";
  {
    std::ofstream f(temp_file);
    f << "content";
  }
  const MemoryFile no_ext = MemoryFile::Make(temp_file);
  EXPECT_EQ(no_ext.extension(), "");
  EXPECT_EQ(no_ext.contents(), "content");
}

GTEST_TEST(MemoryFileTest, MoveSemantics) {
  const MemoryFile empty;
  const MemoryFile original("contents", ".ext", "hint");
  MemoryFile moved_from(original);

  MemoryFile moved_to(std::move(moved_from));

  EXPECT_EQ(moved_to.contents(), original.contents());
  EXPECT_EQ(moved_to.extension(), original.extension());
  EXPECT_EQ(moved_to.sha256(), original.sha256());
  EXPECT_EQ(moved_to.filename_hint(), original.filename_hint());

  EXPECT_EQ(moved_from.contents(), empty.contents());
  EXPECT_EQ(moved_from.extension(), empty.extension());
  EXPECT_EQ(moved_from.sha256(), empty.sha256());
  EXPECT_EQ(moved_from.filename_hint(), empty.filename_hint());
  EXPECT_NE(moved_from.sha256(), original.sha256());
}

GTEST_TEST(MemoryFileTest, ToString) {
  const MemoryFile file("0123456789", ".ext", "hint");

  EXPECT_THAT(file.to_string(), testing::HasSubstr("\"0123456789\""));
  EXPECT_THAT(file.to_string(10), testing::HasSubstr("\"0123456789\""));
  EXPECT_THAT(file.to_string(0), testing::HasSubstr("\"0123456789\""));
  EXPECT_THAT(file.to_string(-10), testing::HasSubstr("\"0123456789\""));
  EXPECT_THAT(file.to_string(5), testing::HasSubstr("\"<01234...>\""));

  EXPECT_THAT(fmt::to_string(file), testing::HasSubstr("\"0123456789\""));
}

/** Confirm that this can be serialized appropriately. The serialization work
 (with all of its nuances) gets tested elsewhere. Here, we're simply testing
 that the fields get serialized and deserialized as expected. */
GTEST_TEST(MemoryFileTest, Serialization) {
  // This content is the text shown in the serialization documentation. Same for
  // the extension and filename hint as well.
  const std::string content("This is an example of memory file test contents.");
  const std::string content_b64(
      "VGhpcyBpcyBhbiBleGFtcGxlIG9mIG1lbW9yeSBmaWxlIHRlc3QgY29udGVudHMu");
  const MemoryFile dut(content, ".txt", "payload.txt");

  // Serialization.
  const std::string y = yaml::SaveYamlString(dut);
  EXPECT_EQ(y, fmt::format("contents: !!binary {}\nextension: "
                           ".txt\nfilename_hint: payload.txt\n",
                           content_b64));

  // Deserialization.
  const auto from_yaml = yaml::LoadYamlString<MemoryFile>(y);
  EXPECT_EQ(from_yaml.contents(), dut.contents());
  EXPECT_EQ(from_yaml.extension(), dut.extension());
  EXPECT_EQ(from_yaml.filename_hint(), dut.filename_hint());
  EXPECT_EQ(from_yaml.sha256(), dut.sha256());
}

}  // namespace
}  // namespace drake
