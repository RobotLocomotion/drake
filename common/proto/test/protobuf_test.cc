#include "drake/common/proto/protobuf.h"

#include <string>

#include <gtest/gtest.h>
#include "google/protobuf/io/coded_stream.h"

#include "drake/common/find_resource.h"

namespace drake {
namespace {

GTEST_TEST(ProtobufUtilsTest, MakeFileInputStreamSucceeds) {
  const std::string absolute_path =
      FindResourceOrThrow("drake/common/proto/test/test_string.txt");
  auto istream = MakeFileInputStreamOrThrow(absolute_path);
  google::protobuf::io::CodedInputStream coded_stream(istream.get());
  std::string expected("test string");
  std::string contents;
  EXPECT_TRUE(coded_stream.ReadString(&contents, expected.size()));
  EXPECT_EQ(contents, expected);
}

GTEST_TEST(ProtobufUtilsTest, MakeFileInputStreamFails) {
  EXPECT_THROW(MakeFileInputStreamOrThrow("bogus.txt"), std::runtime_error);
}

}  // namespace
}  // namespace drake
