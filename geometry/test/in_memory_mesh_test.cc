#include "drake/geometry/in_memory_mesh.h"

#include <string>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(InMemoryMeshTest, ToString) {
  const MemoryFile file("a", ".a", "aa");
  const std::string file_str = fmt::to_string(file);
  InMemoryMesh mesh{file};

  EXPECT_EQ(mesh.to_string(),
            fmt::format("InMemoryMesh(mesh_file={})", file_str));

  mesh.supporting_files.emplace("name", file);
  EXPECT_EQ(mesh.to_string(),
            fmt::format("InMemoryMesh(mesh_file={file}, "
                        "supporting_files={{\"name\": {file}}})",
                        fmt::arg("file", file_str)));

  mesh.supporting_files.emplace("name2", file);
  EXPECT_EQ(mesh.to_string(),
            fmt::format("InMemoryMesh(mesh_file={file}, "
                        "supporting_files={{\"name\": {file}, "
                        "\"name2\": {file}}})",
                        fmt::arg("file", file_str)));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
