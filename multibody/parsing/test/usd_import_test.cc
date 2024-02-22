#include <gtest/gtest.h>

#include "pxr/usd/usd/stage.h"

#include "drake/common/find_resource.h"

namespace drake {
namespace multibody {
namespace {

using drake_vendor_pxr::UsdStageRefPtr;
using drake_vendor_pxr::UsdStage;

GTEST_TEST(UsdImportTest, BasicTest) {
    const std::string usd_name = FindResourceOrThrow(
        "drake/multibody/parsing/test/usd_parser_test/empty.usda");
    // TODO(hong-nvidia): Test file importing once the DefaultArResolver
    // issue is sorted out
    
    // UsdStageRefPtr stage = UsdStage::CreateInMemory();
}

}  // namespace
}  // namespace multibody
}  // namespace drake