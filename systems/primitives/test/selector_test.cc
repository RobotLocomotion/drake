#include "drake/systems/primitives/selector.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/selector_internal.h"

namespace drake {
namespace systems {
namespace {

class SelectorTest : public ::testing::Test {
 protected:
  using InputPortParams = SelectorParams::InputPortParams;
  using OutputPortParams = SelectorParams::OutputPortParams;
  using OutputSelection = SelectorParams::OutputSelection;

  void Initialize(
      const SelectorParams& params = MakeTrivialPassthroughParams()) {
    dut_ = std::make_unique<Selector<double>>(params);
    context_ = dut_->CreateDefaultContext();
  }

  // This gives y0 = u0 where each port is a 1-element vector.
  static SelectorParams MakeTrivialPassthroughParams() {
    return SelectorParams{
        .inputs{
            InputPortParams{.size = 1},
        },
        .outputs{
            OutputPortParams{
                .selections{
                    OutputSelection{
                        .input_port_index = 0,
                        .input_offset = 0,
                    },
                },
            },
        },
    };
  }

  std::unique_ptr<System<double>> dut_;
  std::unique_ptr<Context<double>> context_;
};

TEST_F(SelectorTest, Basic) {
  Initialize();

  // Check the shape.
  EXPECT_TRUE(context_->is_stateless());
  ASSERT_EQ(dut_->num_input_ports(), 1);
  ASSERT_EQ(dut_->num_output_ports(), 1);
  EXPECT_EQ(dut_->get_input_port().get_name(), "u0");
  EXPECT_EQ(dut_->get_input_port().size(), 1);
  EXPECT_EQ(dut_->get_output_port().get_name(), "y0");
  EXPECT_EQ(dut_->get_output_port().size(), 1);

  // Check the output.
  const Vector1<double> input(22.0);
  dut_->get_input_port().FixValue(context_.get(), input);
  EXPECT_EQ(dut_->get_output_port().Eval(*context_), input);
}

TEST_F(SelectorTest, NoPorts) {
  Initialize(SelectorParams{});
  ASSERT_EQ(dut_->num_input_ports(), 0);
  ASSERT_EQ(dut_->num_output_ports(), 0);
}

TEST_F(SelectorTest, NoOutputPort) {
  SelectorParams params = MakeTrivialPassthroughParams();
  params.outputs.clear();
  Initialize(params);
  ASSERT_EQ(dut_->num_input_ports(), 1);
  ASSERT_EQ(dut_->num_output_ports(), 0);
  EXPECT_EQ(dut_->get_input_port().size(), 1);
}

TEST_F(SelectorTest, ZeroSizedPorts) {
  const SelectorParams params{
      .inputs{InputPortParams{.size = 0}},
      .outputs{OutputPortParams{.selections{}}},
  };
  Initialize(params);
  ASSERT_EQ(dut_->num_input_ports(), 1);
  ASSERT_EQ(dut_->num_output_ports(), 1);
  EXPECT_EQ(dut_->get_input_port().size(), 0);
  EXPECT_EQ(dut_->get_output_port().size(), 0);
}

TEST_F(SelectorTest, ComplicatedSelection) {
  // Calculate the following selection recipe ...
  //  y0 = <u0[0], u1[0]>
  //  y1 = <u1[0], u1[2]>
  //  y2 = <u1[1], u0[1]>
  //  y3 = <u1[1], u1[0], u0[1], u0[0]>
  //  y4 = <>  (i.e., empty)
  // ... with u2 unused.
  const SelectorParams params{
      .inputs{
          InputPortParams{.size = 2},
          InputPortParams{.size = 3},
          InputPortParams{.size = 1},
      },
      .outputs{
          // y0 = <u0[0], u1[0]>
          OutputPortParams{
              .selections{
                  OutputSelection{
                      .input_port_index = 0,
                      .input_offset = 0,
                  },
                  OutputSelection{
                      .input_port_index = 1,
                      .input_offset = 0,
                  },
              },
          },
          //  y1 = <u1[0], u1[2]>
          OutputPortParams{
              .selections{
                  OutputSelection{
                      .input_port_index = 1,
                      .input_offset = 0,
                  },
                  OutputSelection{
                      .input_port_index = 1,
                      .input_offset = 2,
                  },
              },
          },
          // y2 = <u1[1], u0[1]>
          OutputPortParams{
              .selections{
                  OutputSelection{
                      .input_port_index = 1,
                      .input_offset = 1,
                  },
                  OutputSelection{
                      .input_port_index = 0,
                      .input_offset = 1,
                  },
              },
          },
          // y3 = <u1[1], u1[0], u0[1], u0[0]>
          OutputPortParams{
              .selections{
                  OutputSelection{
                      .input_port_index = 1,
                      .input_offset = 1,
                  },
                  OutputSelection{
                      .input_port_index = 1,
                      .input_offset = 0,
                  },
                  OutputSelection{
                      .input_port_index = 0,
                      .input_offset = 1,
                  },
                  OutputSelection{
                      .input_port_index = 0,
                      .input_offset = 0,
                  },
              },
          },
          // y4 = <>  (i.e., empty)
          OutputPortParams{
              .selections{},
          },
      },
  };
  Initialize(params);

  // Check the shape.
  ASSERT_EQ(dut_->num_input_ports(), 3);
  ASSERT_EQ(dut_->num_output_ports(), 5);
  EXPECT_EQ(dut_->get_input_port(0).get_name(), "u0");
  EXPECT_EQ(dut_->get_input_port(0).size(), 2);
  EXPECT_EQ(dut_->get_input_port(1).get_name(), "u1");
  EXPECT_EQ(dut_->get_input_port(1).size(), 3);
  EXPECT_EQ(dut_->get_input_port(2).get_name(), "u2");
  EXPECT_EQ(dut_->get_input_port(2).size(), 1);
  EXPECT_EQ(dut_->get_output_port(0).get_name(), "y0");
  EXPECT_EQ(dut_->get_output_port(0).size(), 2);
  EXPECT_EQ(dut_->get_output_port(1).get_name(), "y1");
  EXPECT_EQ(dut_->get_output_port(1).size(), 2);
  EXPECT_EQ(dut_->get_output_port(2).get_name(), "y2");
  EXPECT_EQ(dut_->get_output_port(2).size(), 2);
  EXPECT_EQ(dut_->get_output_port(3).get_name(), "y3");
  EXPECT_EQ(dut_->get_output_port(3).size(), 4);
  EXPECT_EQ(dut_->get_output_port(4).get_name(), "y4");
  EXPECT_EQ(dut_->get_output_port(4).size(), 0);

  // Check the output.
  const Vector2<double> u0(0.0, 1.0);
  const Vector3<double> u1(10.0, 11.0, 12.0);
  const Vector1<double> u2(20.0);
  dut_->get_input_port(0).FixValue(context_.get(), u0);
  dut_->get_input_port(1).FixValue(context_.get(), u1);
  dut_->get_input_port(2).FixValue(context_.get(), u2);
  const Vector2<double> expected_y0(0.0, 10.0);
  const Vector2<double> expected_y1(10.0, 12.0);
  const Vector2<double> expected_y2(11.0, 1.0);
  const Vector4<double> expected_y3(11.0, 10.0, 1.0, 0.0);
  const VectorX<double> expected_y4;
  EXPECT_EQ(dut_->get_output_port(0).Eval(*context_), expected_y0);
  EXPECT_EQ(dut_->get_output_port(1).Eval(*context_), expected_y1);
  EXPECT_EQ(dut_->get_output_port(2).Eval(*context_), expected_y2);
  EXPECT_EQ(dut_->get_output_port(3).Eval(*context_), expected_y3);
  EXPECT_EQ(dut_->get_output_port(4).Eval(*context_), expected_y4);
}

TEST_F(SelectorTest, CustomNames) {
  SelectorParams params = MakeTrivialPassthroughParams();
  params.inputs.at(0).name = "my_input";
  params.outputs.at(0).name = "my_output";
  Initialize(params);
  ASSERT_EQ(dut_->num_input_ports(), 1);
  ASSERT_EQ(dut_->num_output_ports(), 1);
  EXPECT_EQ(dut_->get_input_port().get_name(), "my_input");
  EXPECT_EQ(dut_->get_output_port().get_name(), "my_output");
}

TEST_F(SelectorTest, BadInputSize) {
  // Try to request an invalid input size.
  SelectorParams params = MakeTrivialPassthroughParams();
  params.inputs.at(0).size = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(Selector<double>(params), ".*input.size.*");
}

TEST_F(SelectorTest, BadOutputInvalidInputPort) {
  // Try to cite an input port that doesn't exist.
  SelectorParams params = MakeTrivialPassthroughParams();
  params.outputs.at(0).selections.at(0).input_port_index = 1;
  DRAKE_EXPECT_THROWS_MESSAGE(Selector<double>(params), ".*input_port_index.*");
  params.outputs.at(0).selections.at(0).input_port_index = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(Selector<double>(params), ".*input_port_index.*");
}

TEST_F(SelectorTest, BadOutputInvalidInputOffset) {
  // Try to read beyond the extent of an input port.
  SelectorParams params = MakeTrivialPassthroughParams();
  params.outputs.at(0).selections.at(0).input_offset = 1;
  DRAKE_EXPECT_THROWS_MESSAGE(Selector<double>(params), ".*input_offset.*");
  params.outputs.at(0).selections.at(0).input_offset = -1;
  DRAKE_EXPECT_THROWS_MESSAGE(Selector<double>(params), ".*input_offset.*");
}

TEST_F(SelectorTest, ToAutoDiff) {
  Initialize();
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    ASSERT_EQ(converted.num_input_ports(), 1);
    ASSERT_EQ(converted.num_output_ports(), 1);
  }));
}

TEST_F(SelectorTest, ToSymbolic) {
  Initialize();
  EXPECT_TRUE(is_symbolic_convertible(*dut_, [&](const auto& converted) {
    ASSERT_EQ(converted.num_input_ports(), 1);
    ASSERT_EQ(converted.num_output_ports(), 1);
  }));
}

TEST_F(SelectorTest, DependencyTickets) {
  // Calculate the following selection recipe ...
  //  y0 = <>  (i.e., empty)
  //  y1 = <u0[0]>
  //  y2 = <u1[0]>
  //  y3 = <u0[0], u1[0]>
  // ... with u2 unused.
  const SelectorParams params{
      .inputs{
          InputPortParams{.size = 1},
          InputPortParams{.size = 1},
          InputPortParams{.size = 1},
      },
      .outputs{
          //  y0 = <>  (i.e., empty)
          OutputPortParams{.selections{}},
          //  y1 = <u0[0]>
          OutputPortParams{
              .selections{
                  OutputSelection{
                      .input_port_index = 0,
                      .input_offset = 0,
                  },
              },
          },
          //  y2 = <u1[0]>
          OutputPortParams{
              .selections{
                  OutputSelection{
                      .input_port_index = 1,
                      .input_offset = 0,
                  },
              },
          },
          //  y3 = <u0[0], u1[0]>
          OutputPortParams{
              .selections{
                  OutputSelection{
                      .input_port_index = 0,
                      .input_offset = 0,
                  },
                  OutputSelection{
                      .input_port_index = 1,
                      .input_offset = 0,
                  },
              },
          },
      },
  };
  Initialize(params);
  ASSERT_EQ(dut_->num_input_ports(), 3);
  ASSERT_EQ(dut_->num_output_ports(), 4);

  // y0 depends on nothing.
  EXPECT_FALSE(dut_->HasDirectFeedthrough(0, 0));
  EXPECT_FALSE(dut_->HasDirectFeedthrough(1, 0));
  EXPECT_FALSE(dut_->HasDirectFeedthrough(2, 0));

  // y1 depends on u0.
  EXPECT_TRUE(dut_->HasDirectFeedthrough(0, 1));
  EXPECT_FALSE(dut_->HasDirectFeedthrough(1, 1));
  EXPECT_FALSE(dut_->HasDirectFeedthrough(2, 1));

  // y2 depends on u1.
  EXPECT_FALSE(dut_->HasDirectFeedthrough(0, 2));
  EXPECT_TRUE(dut_->HasDirectFeedthrough(1, 2));
  EXPECT_FALSE(dut_->HasDirectFeedthrough(2, 2));

  // y3 depends on u0,u1.
  EXPECT_TRUE(dut_->HasDirectFeedthrough(0, 3));
  EXPECT_TRUE(dut_->HasDirectFeedthrough(1, 3));
  EXPECT_FALSE(dut_->HasDirectFeedthrough(2, 3));
}

TEST_F(SelectorTest, EfficientPacking) {
  // Define the following selection recipe:
  //  y0 = <u1[0], u1[1], u0[2], u0[3]>
  const SelectorParams params{
      .inputs{
          InputPortParams{.size = 5},
          InputPortParams{.size = 5},
      },
      .outputs{
          //  y0 = <u1[1], u1[2], u0[3], u0[4]>
          OutputPortParams{.selections{
              OutputSelection{
                  .input_port_index = 1,
                  .input_offset = 1,
              },
              OutputSelection{
                  .input_port_index = 1,
                  .input_offset = 2,
              },
              OutputSelection{
                  .input_port_index = 0,
                  .input_offset = 3,
              },
              OutputSelection{
                  .input_port_index = 0,
                  .input_offset = 4,
              },
          }},
      },
  };
  Initialize(params);

  // Check that the packed form is as expected. For this, we'll recreate the
  // program instead of fishing it out of the dut.
  using internal::SelectorProgram;
  const std::vector<const InputPortBase*> ports{&dut_->get_input_port(0),
                                                &dut_->get_input_port(1)};
  const SelectorProgram program(params, ports);
  const SelectorProgram::PerOutput& y = program.get(OutputPortIndex{0});
  // The output uses both input ports.
  ASSERT_EQ(y.size(), 2);
  EXPECT_EQ(y[0].first, ports[0]);
  EXPECT_EQ(y[1].first, ports[1]);
  // Each input port is used for a single contiguous span.
  const std::vector<SelectorProgram::Element>& u0 = y[0].second;
  const std::vector<SelectorProgram::Element>& u1 = y[1].second;
  EXPECT_EQ(u0.size(), 1);
  EXPECT_EQ(u0.at(0).input_start, 3);
  EXPECT_EQ(u0.at(0).output_start, 2);
  EXPECT_EQ(u0.at(0).count, 2);
  EXPECT_EQ(u1.size(), 1);
  EXPECT_EQ(u1.at(0).input_start, 1);
  EXPECT_EQ(u1.at(0).output_start, 0);
  EXPECT_EQ(u1.at(0).count, 2);
}

}  // namespace
}  // namespace systems
}  // namespace drake
