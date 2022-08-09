#include "drake/systems/framework/system_base.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/discrete_values.h"

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
// Objects of this class are created with no system id.
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

  // Approximate LeafSystem in the ability to allocate a non-Context framework
  // quantity (in this case, an empty set of DiscreteValues()).
  std::unique_ptr<DiscreteValues<double>> AllocateDiscreteState() const {
    auto values = std::make_unique<DiscreteValues<double>>();
    // Allocated entities have their owner system id set.
    values->set_system_id(this->get_system_id());
    return values;
  }

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
}

// This tests validation of contexts.
GTEST_TEST(SystemBaseTest, ValidatingContexts) {
  MySystemBase system;
  std::unique_ptr<ContextBase> context = system.AllocateContext();
  DRAKE_EXPECT_NO_THROW(system.ValidateContext(*context));
  DRAKE_EXPECT_NO_THROW(system.ValidateContext(context.get()));

  MySystemBase other_system;
  auto other_context = other_system.AllocateContext();
  // Note: ValidateContext() has special failure messages based on the
  // relationship between system and context. If either system or context is the
  // "root" system, the message specifically refers to the root diagram. In
  // this test, we only have single systems. By definition, the system is the
  // root of its own diagram. Therefore, we can only test:
  //   (root system, root context) --> no throw (tested above).
  //   (root system, non-root context) --> throw message about root system.
  // The tests for the following combinations are contained in diagram_test.cc.
  //   (leaf system, root context)
  //   (root system, leaf context)
  //   (leaf system A, leaf context B)
  DRAKE_EXPECT_THROWS_MESSAGE(system.ValidateContext(*other_context),
                              ".*root Diagram was passed the Context of some "
                              "other[^]*troubleshooting.html.+");
  DRAKE_EXPECT_THROWS_MESSAGE(system.ValidateContext(other_context.get()),
                              ".*root Diagram was passed the Context of some "
                              "other[^]*troubleshooting.html.+");

  // These methods check for null pointers even in Release builds but don't
  // generate fancy messages for them. We're happy as long as "nullptr" gets
  // mentioned.
  ContextBase* null_context = nullptr;
  DRAKE_EXPECT_THROWS_MESSAGE(system.ValidateContext(null_context),
                              ".*nullptr.*");

  // We don't test ValidateContext() against a context without a system id;
  // having a system id is a pre-requisite.
}

// This tests validation of objects that are *not* contexts, but are similarly
// associated with systems (state, parameters, etc.).
GTEST_TEST(SystemBaseTest, ValidatingFrameworkEntity) {
  MySystemBase system;
  system.set_name("any_name_will_do");
  MySystemBase other_system;

  // Calling ValidateCreatedForThisSystem() with some non-Context object gets
  // a different message than for Context.
  std::unique_ptr<DiscreteValues<double>> state =
      system.AllocateDiscreteState();
  DRAKE_EXPECT_NO_THROW(system.ValidateCreatedForThisSystem(state.get()));
  DRAKE_EXPECT_NO_THROW(system.ValidateCreatedForThisSystem(*state));
  std::unique_ptr<DiscreteValues<double>> other_state =
      other_system.AllocateDiscreteState();
  DRAKE_EXPECT_THROWS_MESSAGE(
      system.ValidateCreatedForThisSystem(other_state.get()),
      ".*DiscreteValues.*was not created .+MySystemBase.*any_name_will_do.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      system.ValidateCreatedForThisSystem(*other_state),
      ".*DiscreteValues.*was not created .+MySystemBase.*any_name_will_do.*");

  // These methods check for null pointers even in Release builds but don't
  // generate fancy messages for them. We're happy as long as "nullptr" gets
  // mentioned.
  DiscreteValues<double>* null_state = nullptr;
  DRAKE_EXPECT_THROWS_MESSAGE(system.ValidateCreatedForThisSystem(null_state),
                              ".*nullptr.*");

  // We *can* pass a context without system id to ValidateCreatedForThisSystem.
  // The result should be the same as passing any other "disconnected" framework
  // entity.
  DiscreteValues<double> disconnected_state;  // No system id.

  DRAKE_EXPECT_THROWS_MESSAGE(
      system.ValidateCreatedForThisSystem(disconnected_state),
      ".*DiscreteValues.*was not associated.*should have been "
      "created for.*MySystemBase.*any_name_will_do.*");
}


}  // namespace system_base_test_internal
}  // namespace systems
}  // namespace drake
