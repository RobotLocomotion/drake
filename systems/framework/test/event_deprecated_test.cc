#include <gtest/gtest.h>

#include "drake/systems/framework/event.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {
namespace {

class EventDeprecatedTest : public ::testing::Test {
 protected:
  const PassThrough<double> system_{1};
  std::unique_ptr<Context<double>> context_ptr_ =
      system_.CreateDefaultContext();
  const Context<double>& context_ = *context_ptr_;
};

TEST_F(EventDeprecatedTest, PublishCallback) {
  bool called = false;
  PublishEvent<double>::PublishCallback callback =
      [&called](const Context<double>&, const PublishEvent<double>&) {
        called = true;
      };
  PublishEvent<double> event(callback);
  EventStatus result = event.handle(system_, context_);
  EXPECT_EQ(result.severity(), EventStatus::Severity::kSucceeded);
  EXPECT_TRUE(called);
}

TEST_F(EventDeprecatedTest, PublishSystemCallback) {
  bool called = false;
  PublishEvent<double>::SystemCallback callback =
      [&called](const System<double>&, const Context<double>&,
                const PublishEvent<double>&) {
        called = true;
      };
  PublishEvent<double> event(callback);
  EventStatus result = event.handle(system_, context_);
  EXPECT_EQ(result.severity(), EventStatus::Severity::kSucceeded);
  EXPECT_TRUE(called);
}

TEST_F(EventDeprecatedTest, DiscreteUpdateCallback) {
  bool called = false;
  DiscreteUpdateEvent<double>::DiscreteUpdateCallback callback =
      [&called](const Context<double>&, const DiscreteUpdateEvent<double>&,
                DiscreteValues<double>*) {
        called = true;
      };
  DiscreteUpdateEvent<double> event(callback);
  DiscreteValues<double> out;
  EventStatus result = event.handle(system_, context_, &out);
  EXPECT_EQ(result.severity(), EventStatus::Severity::kSucceeded);
  EXPECT_TRUE(called);
}

TEST_F(EventDeprecatedTest, DiscreteUpdateSystemCallback) {
  bool called = false;
  DiscreteUpdateEvent<double>::SystemCallback callback =
      [&called](const System<double>&, const Context<double>&,
                const DiscreteUpdateEvent<double>&, DiscreteValues<double>*) {
        called = true;
      };
  DiscreteUpdateEvent<double> event(callback);
  DiscreteValues<double> out;
  EventStatus result = event.handle(system_, context_, &out);
  EXPECT_EQ(result.severity(), EventStatus::Severity::kSucceeded);
  EXPECT_TRUE(called);
}

TEST_F(EventDeprecatedTest, UnrestrictedUpdateCallback) {
  bool called = false;
  UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback =
      [&called](const Context<double>&, const UnrestrictedUpdateEvent<double>&,
                State<double>*) {
        called = true;
      };
  UnrestrictedUpdateEvent<double> event(callback);
  State<double> out;
  EventStatus result = event.handle(system_, context_, &out);
  EXPECT_EQ(result.severity(), EventStatus::Severity::kSucceeded);
  EXPECT_TRUE(called);
}

TEST_F(EventDeprecatedTest, UnrestrictedUpdateSystemCallback) {
  bool called = false;
  UnrestrictedUpdateEvent<double>::SystemCallback callback =
      [&called](const System<double>&, const Context<double>&,
                const UnrestrictedUpdateEvent<double>&,
                State<double>*) {
        called = true;
      };
  UnrestrictedUpdateEvent<double> event(callback);
  State<double> out;
  EventStatus result = event.handle(system_, context_, &out);
  EXPECT_EQ(result.severity(), EventStatus::Severity::kSucceeded);
  EXPECT_TRUE(called);
}

}  // namespace
}  // namespace systems
}  // namespace drake
