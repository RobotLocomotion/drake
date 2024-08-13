#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Consider accommodation for colorblind people. Avoid red/green
//  and/or use line styles. Update the legend.

std::string SpanningForest::GenerateGraphvizString(
    std::string_view label) const {
  std::string graphviz = "digraph SpanningForest {\n";
  graphviz += "rankdir=BT;\n";
  graphviz += "labelloc=t;\n";
  graphviz += fmt::format("label=\"{}\nSpanningForest\";\n", label);

  // Generate a legend.
  graphviz += "legend [shape=none]\n";
  graphviz +=
      "[label=\""
      "* = massless"
      "\nred = shadow"
      "\npurple = reversed"
      "\"]\n";

  // For each Mobod, draw the body (as a node) and the mobilizer (as an edge).
  for (const Mobod& mobod : mobods()) {
    std::string followers;
    for (LinkOrdinal ordinal : mobod.follower_link_ordinals())
      followers += fmt::format("L({}) ", ordinal);
    graphviz += fmt::format(
        "mobod{} [color={}] [label=\"mobod({}){}\n{}\"];\n", mobod.index(),
        links(mobod.link_ordinal()).is_shadow() ? "red" : "black",
        mobod.index(), mobod.has_massful_follower_link() ? "" : "*", followers);
    if (mobod.is_world()) continue;
    const Mobod& inboard = mobods(mobod.inboard());

    const std::string color = mobod.is_reversed() ? "purple"
                              : mobod.is_weld()   ? "black"
                                                  : "blue";
    const std::string arrow_type = mobod.is_weld() ? "box" : "normal";
    const std::string style = mobod.is_weld() ? "bold" : "solid";

    const LinkJointGraph::Joint& joint = joints(mobod.joint_ordinal());
    graphviz += fmt::format(
        "mobod{} -> mobod{} [arrowhead={}] [fontsize=10] [style={}]"
        "[label=\"mobilizer({})\nJ({}) {}{}\nq{} v{}\"] [color={}];\n",
        inboard.index(), mobod.index(), arrow_type, style, mobod.index(),
        joint.ordinal(), graph().joint_traits(joint.traits_index()).name,
        mobod.is_reversed() ? "-R" : "", mobod.q_start(), mobod.v_start(),
        color);
  }

  // Draw weld constraints that were added to close loops.
  for (const LoopConstraint& constraint : loop_constraints()) {
    graphviz += fmt::format(
        "mobod{} -> mobod{} [dir=both] [arrowhead=box] [arrowtail=box] "
        "[style=bold] [fontsize=10] [label=\"WELD({})\n\"] [color=red]\n",
        constraint.primary_mobod(), constraint.shadow_mobod(),
        constraint.index());
  }

  graphviz += "}\n";
  return graphviz;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
