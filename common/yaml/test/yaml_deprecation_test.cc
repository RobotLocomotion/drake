#include <gtest/gtest.h>

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_write_archive.h"

namespace {

GTEST_TEST(YamlDeprecationTest, Aliases) {
  auto root = drake::yaml::internal::Node::MakeNull();
  drake::yaml::YamlReadArchive::Options options;
  drake::yaml::YamlReadArchive read_archive(root, options);
  drake::yaml::YamlWriteArchive write_archive;
}

}  // namespace
