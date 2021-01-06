#include "drake/systems/lcm/connect_lcm_scope.h"

#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

// Scope a ConstantVectorSource and check the output.
GTEST_TEST(ScopeTest, PublishTest) {
  drake::lcm::DrakeLcm lcm;
  const std::string channel = "my_channel";

  Eigen::VectorXd vec = Eigen::VectorXd::LinSpaced(5, 1.0, 2.0);

  DiagramBuilder<double> builder;
  auto source = builder.AddSystem<ConstantVectorSource>(vec);
  ConnectLcmScope(source->get_output_port(), channel, &builder, &lcm);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  drake::lcm::Subscriber<lcmt_drake_signal> sub(&lcm, channel);
  diagram->Publish(*context);
  lcm.HandleSubscriptions(0);

  const auto& message = sub.message();
  ASSERT_EQ(message.dim, vec.size());
  for (int i = 0; i < vec.size(); i++) {
    EXPECT_EQ(message.val[i], vec[i]);
  }
}

// Check that publish_period defaults to zero, or can be set by the user.
GTEST_TEST(ScopeTest, PeriodTest) {
  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm;
  auto dummy = builder.AddSystem<PassThrough>(1);
  const auto* publisher0 = ConnectLcmScope(
      dummy->get_output_port(), "channel0", &builder, &lcm);
  const auto* publisher1 = ConnectLcmScope(
      dummy->get_output_port(), "channel1", &builder, &lcm, 0.01);
  ASSERT_NE(publisher0, nullptr);
  ASSERT_NE(publisher1, nullptr);
  EXPECT_EQ(publisher0->get_publish_period(), 0.0);
  EXPECT_EQ(publisher1->get_publish_period(), 0.01);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
