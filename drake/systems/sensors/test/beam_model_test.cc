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
  DepthSensorSpecification spec;
  spec.set_max_range(5.0);
  spec.set_num_pitch_values(4);
  spec.set_num_yaw_values(10);

  BeamModel<double> model(spec);
  EXPECT_EQ(model.max_range(), 5.0);
  EXPECT_EQ(model.get_depth_input_port().size(), 40);
}

GTEST_TEST(BeamModelTest, TestInputPorts) {
  const int kNumReadings = 10;
  BeamModel<double> model(kNumReadings, 6.0);

  EXPECT_EQ(model.get_depth_input_port().size(), kNumReadings);
  EXPECT_FALSE(model.get_depth_input_port().is_random());

  EXPECT_EQ(model.get_uniform_random_input_port().size(), 2 * kNumReadings);
  EXPECT_EQ(model.get_uniform_random_input_port().get_random_type().value(),
            RandomDistribution::kUniform);

  EXPECT_EQ(model.get_gaussian_random_input_port().size(), kNumReadings);
  EXPECT_EQ(model.get_gaussian_random_input_port().get_random_type().value(),
            RandomDistribution::kGaussian);

  EXPECT_EQ(model.get_exponential_random_input_port().size(), kNumReadings);
  EXPECT_EQ(model.get_exponential_random_input_port().get_random_type().value(),
            RandomDistribution::kExponential);
}

// Compare random samples to an analytic form of the probability density
// function.
GTEST_TEST(BeamModelTest, TestProbabilityDensity) {
  systems::DiagramBuilder<double> builder;

  const double depth_input = 3.0;
  const double max_range = 5.0;
  auto beam_model = builder.AddSystem<BeamModel>(1, max_range);

  auto constant_depth =
      builder.AddSystem<ConstantVectorSource>(Vector1d(depth_input));
  builder.Connect(constant_depth->get_output_port(),
                  beam_model->get_depth_input_port());

  auto uniform_random = builder.AddSystem<UniformRandomSource>(2, 0.0025);
  builder.Connect(uniform_random->get_output_port(0),
                  beam_model->get_uniform_random_input_port());

  auto gaussian_random = builder.AddSystem<GaussianRandomSource>(1, 0.0025);
  builder.Connect(gaussian_random->get_output_port(0),
                  beam_model->get_gaussian_random_input_port());

  auto exponential_random =
      builder.AddSystem<ExponentialRandomSource>(1, 0.0025);
  builder.Connect(exponential_random->get_output_port(0),
                  beam_model->get_exponential_random_input_port());

  auto logger = LogOutput(beam_model->get_output_port(0), &builder);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  // Zero all initial state.
  for (int i = 0; i < simulator.get_context().get_num_discrete_state_groups();
       i++) {
    BasicVector<double>* state =
        simulator.get_mutable_context()->get_mutable_discrete_state(0);
    for (int j = 0; j < state->size(); j++) {
      state->SetAtIndex(j, 0.0);
    }
  }

  auto params = dynamic_cast<BeamModelParams<double>*>(
      simulator.get_mutable_context()->get_mutable_numeric_parameter(0));

  // Set some testable beam model parameters.
  params->set_lambda_short(2.0);
  params->set_sigma_hit(0.25);
  params->set_probability_short(0.2);
  params->set_probability_miss(0.05);
  params->set_probability_random(0.05);

  auto probability_density_function = [&](double z) {
    DRAKE_DEMAND(z >= 0.0 && z < max_range);  // Doesn't capture the delta
                                              // function (with height p_miss)
                                              // at max_range.
    const double p_short =
        (z <= depth_input)
            ? params->lambda_short() * std::exp(-params->lambda_short() * z)
            : 0.0;
    double p_hit = 1.0 - params->probability_random() -
                   params->probability_miss() - params->probability_short();
    p_hit += std::exp(
        -params->lambda_short() *
        depth_input);  // Truncated tail of the exponential adds to "hit".

    const double sigma_sq = params->sigma_hit() * params->sigma_hit();
    return params->probability_random() / max_range +
           params->probability_short() * p_short +
           p_hit * std::exp(-0.5 * (z - depth_input) * (z - depth_input) /
                            sigma_sq) /
               std::sqrt(2 * M_PI * sigma_sq);
  };

  simulator.Initialize();
  simulator.StepTo(50);

  const auto& x = logger->data();

  const int N = x.size();

  // All values are in [0.0, max_range]
  EXPECT_TRUE((x.array() >= 0.0 && x.array() <= max_range).all());

  // Matlab visual debugging:
  Eigen::VectorXd depth =
      Eigen::VectorXd::LinSpaced(1000, 0.0, max_range - 1e-6);
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
  // Evaluate all subintervals [a,a+h] in [0,max_range).
  for (double a = 0.0; a <= max_range - h; a += h) {
    // Counts the number of samples in (a,a+h).
    const double count = (x.array() >= a && x.array() < a + h)
                             .template cast<double>()
                             .matrix()
                             .sum();

    EXPECT_NEAR(count / N, probability_density_function(a + h / 2) * h, 1e-2);
    CallMatlab("plot", a + h / 2, count / N / h, "r.");
  }

  // Check the max returns.
  double p_hit = 1.0 - params->probability_random() -
                 params->probability_miss() - params->probability_short();
  p_hit += std::exp(
      -params->lambda_short() *
      depth_input);  // Truncated tail of the exponential adds to "hit".
  // Cumulative distribution function of the standard normal distribution.
  auto Phi = [](double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); };
  const double p_max =
      params->probability_miss() +
      p_hit * Phi(-depth_input /
                  params->sigma_hit())  // "hit" would have returned < 0.0.
      +
      p_hit *
          Phi((depth_input - max_range) /
              params->sigma_hit());  // "hit" would have returned > max_range.
  EXPECT_NEAR(
      (x.array() == max_range).template cast<double>().matrix().sum() / N,
      p_max, 1e-3);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
