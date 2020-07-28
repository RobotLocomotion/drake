#include "drake/systems/framework/publish_initialization_events.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(PublishInitializationEvents, Test) {
  class InitializationTestSystem : public LeafSystem<double> {
   public:
    InitializationTestSystem() {
      PublishEvent<double> pub_event(
          TriggerType::kInitialization,
          std::bind(&InitializationTestSystem::InitPublish, this,
                    std::placeholders::_1, std::placeholders::_2));
      DeclareInitializationEvent(pub_event);
    }

    bool get_pub_init() const { return pub_init_; }

   private:
    void InitPublish(const Context<double>& context,
                     const PublishEvent<double>& event) const {
      EXPECT_EQ(context.get_time(), 0);
      EXPECT_EQ(event.get_trigger_type(),
                TriggerType::kInitialization);
      pub_init_ = true;
    }

    mutable bool pub_init_{false};
  };

  InitializationTestSystem system;
  auto context = system.CreateDefaultContext();
  PublishInitializationEvents(system, *context);

  EXPECT_TRUE(system.get_pub_init());
}

}  // namespace
}  // namespace systems
}  // namespace drake
