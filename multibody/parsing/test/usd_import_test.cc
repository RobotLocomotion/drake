#include <gtest/gtest.h>

#include "pxr/usd/usd/stage.h"

#include "drake/common/find_resource.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(UsdImportTest, BasicTest) {
    const std::string usd_name = FindResourceOrThrow(
        "drake/multibody/parsing/test/usd_parser_test/empty.usda");
    // TODO(hong-nvidia): Test file importing once the DefaultArResolver
    // issue is sorted out
}

}  // namespace
}  // namespace multibody
}  // namespace drake