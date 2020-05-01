#include "drake/systems/framework/system_base.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

// TODO(sherm1) As SystemBase gains more functionality, move type-agnostic
// tests here from {system, diagram, leaf}_test.cc.
// Cache entry methods are tested in cache_entry_test.cc.

namespace drake {
namespace systems {

// Don't want anonymous here because the type name test would then depend on
// exactly how anonymous namespaces are rendered by NiceTypeName, which is
// a "don't care" here.
namespace system_base_test_internal {

// A minimal concrete ContextBase object suitable for some simple tests.
class MyContextBase final : public ContextBase {
 public:
  MyContextBase() = default;
  MyContextBase(const MyContextBase&) = default;

 private:
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::make_unique<MyContextBase>(*this);
  }
};

// A minimal concrete SystemBase object suitable for some simple tests.
class MySystemBase final : public SystemBase {
 public:
  MySystemBase() {}

 private:
  std::unique_ptr<ContextBase> DoAllocateContext() const final {
    auto context = std::make_unique<MyContextBase>();
    InitializeContextBase(&*context);
    return context;
  }

  std::function<void(const AbstractValue&)> MakeFixInputPortTypeChecker(
      InputPortIndex) const override {
    return {};
  }

  std::multimap<int, int> GetDirectFeedthroughs() const final {
    throw std::logic_error("GetDirectFeedthroughs is not implemented");
  }
};

// Verify that system name methods work properly. Can't fully test the
// pathname here since we can't build a diagram; that's tested in diagram_test
// instead. But we can test it works right for a single-node System.
GTEST_TEST(SystemBaseTest, NameAndMessageSupport) {
  MySystemBase system;
  EXPECT_EQ(system.get_name(), "");
  EXPECT_EQ(system.GetSystemName(), "_");  // Current dummy name.
  EXPECT_EQ(system.GetSystemPathname(), "::_");

  system.set_name("any_name_will_do");
  EXPECT_EQ(system.get_name(), "any_name_will_do");
  EXPECT_EQ(system.GetSystemName(), "any_name_will_do");
  EXPECT_EQ(system.GetSystemPathname(), "::any_name_will_do");

  EXPECT_EQ(system.GetSystemType(),
            "drake::systems::system_base_test_internal::MySystemBase");

  auto context = system.AllocateContext();
  DRAKE_EXPECT_NO_THROW(system.ValidateContext(*context));

  MySystemBase other_system;
  auto other_context = other_system.AllocateContext();
  DRAKE_EXPECT_THROWS_MESSAGE(system.ValidateContext(*other_context),
                              std::exception,
                              ".*Context.*was not created for.*");
}

}  // namespace system_base_test_internal
}  // namespace systems
}  // namespace drake
