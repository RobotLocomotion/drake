#include "drake/systems/sensors/beam_model.h"

#include <gtest/gtest.h>

#include "drake/common/proto/call_matlab.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/random_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/sensors/gen/beam_model_params.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

GTEST_TEST(BeamModelTest, TestSpecConstructor) {
  const double kMaxRange = 5.0;

  DepthSensorSpecification spec;
  spec.set_max_range(kMaxRange);
  spec.set_num_pitch_values(4);
  spec.set_num_yaw_values(10);

  BeamModel<double> model(spec);
  EXPECT_EQ(model.max_range(), kMaxRange);
  EXPECT_EQ(model.get_depth_input_port().size(), 40);
}

GTEST_TEST(BeamModelTest, TestInputPorts) {
  const int kNumReadings = 10;
  const double kMaxRange = 5.0;
  BeamModel<double> model(kNumReadings, kMaxRange);

  EXPECT_EQ(model.get_depth_input_port().size(), kNumReadings);
  EXPECT_FALSE(model.get_depth_input_port().is_random());

  EXPECT_EQ(model.get_event_random_input_port().size(), kNumReadings);
  EXPECT_EQ(model.get_event_random_input_port().get_random_type().value(),
            RandomDistribution::kUniform);

  EXPECT_EQ(model.get_hit_random_input_port().size(), kNumReadings);
  EXPECT_EQ(model.get_hit_random_input_port().get_random_type().value(),
            RandomDistribution::kGaussian);

  EXPECT_EQ(model.get_short_random_input_port().size(), kNumReadings);
  EXPECT_EQ(model.get_short_random_input_port().get_random_type().value(),
            RandomDistribution::kExponential);

  EXPECT_EQ(model.get_uniform_random_input_port().size(), kNumReadings);
  EXPECT_EQ(model.get_uniform_random_input_port().get_random_type().value(),
            RandomDistribution::kUniform);
}

// Compare random samples to an analytic form of the probability density
// function.
GTEST_TEST(BeamModelTest, TestProbabilityDensity) {
  systems::DiagramBuilder<double> builder;

  const double kDepthInput = 3.0;
  const double kMaxRange = 5.0;
  auto beam_model = builder.AddSystem<BeamModel>(1, kMaxRange);

  auto constant_depth =
      builder.AddSystem<ConstantVectorSource>(Vector1d(kDepthInput));
  builder.Connect(constant_depth->get_output_port(),
                  beam_model->get_depth_input_port());

  auto w_event = builder.AddSystem<UniformRandomSource>(1, 0.0025);
  builder.Connect(w_event->get_output_port(0),
                  beam_model->get_event_random_input_port());

  auto w_hit = builder.AddSystem<GaussianRandomSource>(1, 0.0025);
  builder.Connect(w_hit->get_output_port(0),
                  beam_model->get_hit_random_input_port());

  auto w_short = builder.AddSystem<ExponentialRandomSource>(1, 0.0025);
  builder.Connect(w_short->get_output_port(0),
                  beam_model->get_short_random_input_port());

  auto w_uniform = builder.AddSystem<UniformRandomSource>(1, 0.0025);
  builder.Connect(w_uniform->get_output_port(0),
                  beam_model->get_uniform_random_input_port());

  auto logger = LogOutput(beam_model->get_output_port(0), &builder);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  // Zero all initial state.
  for (int i = 0; i < simulator.get_context().get_num_discrete_state_groups();
       i++) {
    BasicVector<double>& state =
        simulator.get_mutable_context().get_mutable_discrete_state(0);
    for (int j = 0; j < state.size(); j++) {
      state.SetAtIndex(j, 0.0);
    }
  }

  auto& params =
      beam_model->get_mutable_parameters(&diagram->GetMutableSubsystemContext(
          *beam_model, &simulator.get_mutable_context()));

  // Set some testable beam model parameters.
  params.set_lambda_short(2.0);
  params.set_sigma_hit(0.25);
  params.set_probability_short(0.2);
  params.set_probability_miss(0.05);
  params.set_probability_uniform(0.05);

  double probability_hit = 1.0 - params.probability_uniform() -
                           params.probability_miss() -
                           params.probability_short();
  // Truncated tail of the exponential adds to "hit".
  probability_hit += std::exp(-params.lambda_short() * kDepthInput);

  auto probability_density_function = [&](double z) {
    DRAKE_DEMAND(z >= 0.0 && z < kMaxRange);  // Doesn't capture the delta
                                              // function (with height p_miss)
                                              // at kMaxRange.
    const double p_short =
        (z <= kDepthInput)
            ? params.lambda_short() * std::exp(-params.lambda_short() * z)
            : 0.0;

    const double sigma_sq = params.sigma_hit() * params.sigma_hit();
    return params.probability_uniform() / kMaxRange +
           params.probability_short() * p_short +
           probability_hit * std::exp(-0.5 * (z - kDepthInput) *
                                      (z - kDepthInput) / sigma_sq) /
               std::sqrt(2 * M_PI * sigma_sq);
  };

  simulator.Initialize();
  simulator.StepTo(50);

  const auto& x = logger->data();

  const int N = x.size();

  // All values are in [0.0, kMaxRange]
  EXPECT_TRUE((x.array() >= 0.0 && x.array() <= kMaxRange).all());

  // Matlab visual debugging:
  const Eigen::VectorXd depth =
      Eigen::VectorXd::LinSpaced(1000, 0.0, kMaxRange - 1e-6);
  Eigen::VectorXd pdf(depth.size());
  for (int i = 0; i < static_cast<int>(depth.size()); i++) {
    pdf[i] = probability_density_function(depth[i]);
  }
  using common::CallMatlab;
  CallMatlab("clf");
  CallMatlab("plot", depth, pdf);
  CallMatlab("xlabel", "depth (m)");
  CallMatlab("ylabel", "probability density");
  CallMatlab("hold", "on");

  const double h = 0.2;
  // Evaluate all subintervals [a,a+h] in [0,kMaxRange).
  for (double a = 0.0; a < kMaxRange; a += h) {
    // Counts the number of samples in (a,a+h).
    const double count = (x.array() >= a && x.array() < a + h - 1e-8)
                             .template cast<double>()
                             .matrix()
                             .sum();

    EXPECT_NEAR(count / N, probability_density_function(a + h / 2) * h, 1.5e-2);
    CallMatlab("plot", a + h / 2, count / N / h, "r.");
  }

  // Check the max returns.
  // Cumulative distribution function of the standard normal distribution.
  auto Phi = [](double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); };
  const double p_max =
      params.probability_miss() +
      probability_hit *
          Phi(-kDepthInput /
              params.sigma_hit())  // "hit" would have returned < 0.0.
      +
      probability_hit *
          Phi((kDepthInput - kMaxRange) /
              params.sigma_hit());  // "hit" would have returned > kMaxRange.
  EXPECT_NEAR(
      (x.array() == kMaxRange).template cast<double>().matrix().sum() / N,
      p_max, 3e-3);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
