#include "drake/systems/framework/system_base.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

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
  explicit MyContextBase(bool is_good) : is_good_{is_good} {}
  MyContextBase(const MyContextBase&) = default;

  bool is_good() const { return is_good_; }

 private:
  std::unique_ptr<ContextBase> DoCloneWithoutPointers() const final {
    return std::make_unique<MyContextBase>(*this);
  }

  // For testing error checking for bad contexts.
  bool is_good_{false};
};

// A minimal concrete SystemBase object suitable for some simple tests.
class MySystemBase final : public SystemBase {
 public:
  MySystemBase() {}

 private:
  std::unique_ptr<ContextBase> DoAllocateContext() const final {
    auto context = std::make_unique<MyContextBase>(true);  // A valid context.
    InitializeContextBase(&*context);
    return context;
  }

  void DoCheckValidContext(const ContextBase& context) const final {
    auto& my_context = dynamic_cast<const MyContextBase&>(context);
    if (my_context.is_good())
      return;
    throw std::logic_error("This Context is totally unacceptable!");
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
  EXPECT_NO_THROW(system.ThrowIfContextNotCompatible(*context));

  MyContextBase bad_context(false);
  DRAKE_EXPECT_THROWS_MESSAGE(system.ThrowIfContextNotCompatible(bad_context),
                              std::logic_error,
                              ".*Context.*unacceptable.*");
}

}  // namespace system_base_test_internal
}  // namespace systems
}  // namespace drake
