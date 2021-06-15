#include "drake/systems/framework/system_html.h"

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {

using ::testing::HasSubstr;

GTEST_TEST(SystemHtmlTest, LeafSystem) {
  Adder<double> adder(2, 1);
  adder.set_name("adder");
  const std::string html = GenerateHtml(adder);

  EXPECT_THAT(html, HasSubstr("key: \"adder\""));
  EXPECT_THAT(html, HasSubstr("input_ports: [ { name: \"u0\", id: \"u0\" }, { "
                              "name: \"u1\", id: \"u1\" }, ]"));
  EXPECT_THAT(html,
              HasSubstr("output_ports: [ { name: \"sum\", id: \"y0\" }, ]"));
}

// Construct a nested diagram and ensure the subcomponents are visited
// correctly.
GTEST_TEST(SystemHtmlTest, NestedDiagram) {
  DiagramBuilder<double> builder;
  auto* adder = builder.AddSystem<Adder<double>>(2, 1);
  adder->set_name("adder");
  auto* passthrough = builder.AddSystem<PassThrough<double>>(1);
  passthrough->set_name("pass");
  builder.Connect(passthrough->get_output_port(), adder->get_input_port(0));
  builder.ExportInput(adder->get_input_port(1));
  builder.ExportInput(passthrough->get_input_port());
  builder.ExportOutput(adder->get_output_port());
  DiagramBuilder<double> builder2;
  auto* subdiagram = builder2.AddSystem(builder.Build());
  subdiagram->set_name("subdiagram");
  auto diagram = builder2.Build();
  diagram->set_name("diagram");

  const std::string html = GenerateHtml(*diagram, 1);

  // Confirm that the diagram information does not regress from the manually
  // verified output.

  // Tests NodeWriter.
  EXPECT_THAT(
      html,
      HasSubstr(
          R"({ key: "diagram", name: "diagram", group: "", isGroup: true, expanded: true, },
{ key: "subdiagram", name: "subdiagram", group: "diagram", isGroup: true, expanded: false, },
{ key: "subdiagram_inputs", name: "Input Ports", group: "subdiagram", isGroup: true, },
{ key: "subdiagram_u0", name: "adder_u1", group: "subdiagram_inputs", category: "input_port", },
{ key: "subdiagram_u1", name: "pass_u", group: "subdiagram_inputs", category: "input_port", },
{ key: "subdiagram_outputs", name: "Output Ports", group: "subdiagram", isGroup: true, },
{ key: "subdiagram_y0", name: "adder_sum", group: "subdiagram_outputs", category: "output_port", },
{ key: "adder", group: "subdiagram", input_ports: [ { name: "u0", id: "u0" }, { name: "u1", id: "u1" }, ],
output_ports: [ { name: "sum", id: "y0" }, ],
},
{ key: "pass", group: "subdiagram", input_ports: [ { name: "u", id: "u0" }, ],
output_ports: [ { name: "y", id: "y0" }, ],
},)"));

  // Tests LinkWriter.
  EXPECT_THAT(
      html,
      HasSubstr(R"({ from: "pass", fromPort: "y0", to: "adder", toPort: "u0", },
{ from: "subdiagram_u0", to: "adder", toPort: "u1", },
{ from: "subdiagram_u1", to: "pass", toPort: "u0", },
{ from: "adder", fromPort: "y0", to: "subdiagram_y0", },)"));
}

// Test a diagram that uses exported input fan-out.
GTEST_TEST(SystemHtmlTest, SystemWithFanout) {
  StateInterpolatorWithDiscreteDerivative<double> system(3, 0.05);
  system.set_name("terp");
  const std::string html = GenerateHtml(system);

  // Proof of life test for fanout.
  EXPECT_THAT(html, HasSubstr(R"(from: "terp_u0", to: "drake/systems/Multiplexer)"));
  EXPECT_THAT(html, HasSubstr(R"(from: "terp_u0", to: "drake/systems/DiscreteDerivative)"));
}

}  // namespace systems
}  // namespace drake
