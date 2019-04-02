#include "drake/systems/lcm/connect_lcm_scope.h"

#include <gtest/gtest.h>

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

// Scope a ConstantVectorSource and check the output.
GTEST_TEST(ScopeTest, PublishTest) {
  drake::lcm::DrakeMockLcm lcm;
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

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
