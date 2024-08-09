#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <fmt/format.h>

#include "drake/multibody/topology/forest.h"
#include "drake/multibody/topology/graph.h"

namespace drake {
namespace multibody {
namespace internal {

void LinkJointGraph::DumpGraph(std::string title) const {
  std::cout << fmt::format("\n\n**** GRAPH {} ****\n", title);
  std::cout << "JointTypes:\n";
  for (JointTraitsIndex i(0); i < ssize(data_.joint_traits); ++i) {
    const JointTraits& traits = data_.joint_traits[i];
    std::cout << fmt::format("  {}: {} nq={} nv={} has_quat={}\n", i,
                             traits.name, traits.nq, traits.nv,
                             traits.has_quaternion);
  }
  std::cout << "User Links:\n";
  for (const auto& link : links()) {
    if (link.index() == num_user_links()) std::cout << "Ephemeral Links:\n";
    std::cout << fmt::format("  {}: {} inst={}{}{}{}{}{}{}", link.index(),
                             link.name(), link.model_instance(),
                             link.is_world() ? " World" : "",
                             link.is_static_flag_set() ? " static" : "",
                             link.is_anchored() ? " anchored" : "",
                             link.must_be_base_body() ? " must_be_base" : "",
                             link.is_massless() ? " massless" : "",
                             link.is_shadow() ? " shadow" : "");
    if (link.mobod_index().is_valid()) {
      std::cout << fmt::format(" mobod={}", link.mobod_index());
    }
    if (link.inboard_joint_index().is_valid()) {
      std::cout << fmt::format(" joint={}", link.inboard_joint_index());
    }
    if (link.composite().has_value()) {
      std::cout << fmt::format(" composite={}", *link.composite());
    }
    std::cout << "\n";

    std::cout << "      joints as_parent=";
    for (const JointIndex& i : link.joints_as_parent()) std::cout << i << " ";
    std::cout << " as_child=";
    for (const JointIndex& i : link.joints_as_child()) std::cout << i << " ";
    if (!link.loop_constraints().empty()) {
      std::cout << " loop constraints=";
      for (const LoopConstraintIndex& i : link.loop_constraints())
        std::cout << i << " ";
    }
    std::cout << "\n";
  }
  if (ssize(links()) == num_user_links()) std::cout << "(No ephemeral Links)\n";

  std::cout << "Link Composites:\n";
  for (LinkCompositeIndex i(0); i < ssize(link_composites()); ++i) {
    std::cout << fmt::format("  {}: ", i);
    for (const BodyIndex& li : link_composites()[i].links)
      std::cout << fmt::format("{} ", li);
    std::cout << "\n";
  }
  std::cout << "User Joints:\n";
  for (const Joint& joint : joints()) {
    if (joint.index() == num_user_joints()) std::cout << "Ephemeral Joints:\n";
    std::cout << fmt::format(
        "  {}: {} inst={} {} parent={} child={}\n", joint.index(), joint.name(),
        joint.model_instance(), joint_traits(joint.traits_index()).name,
        joint.parent_link_index(), joint.child_link_index());
  }
  if (ssize(joints()) == num_user_joints())
    std::cout << "(No ephemeral Joints)\n";

  std::cout << "Loop Constraints (always ephemeral):\n";
  for (const LoopConstraint& constraint : loop_constraints()) {
    std::cout << fmt::format(
        "  {}: {} inst={} primary={} shadow={}\n", constraint.index(),
        constraint.name(), constraint.model_instance(),
        constraint.primary_link(), constraint.shadow_link());
  }
}

std::string LinkJointGraph::GenerateGraphvizString(
    std::string_view label, bool include_ephemeral_elements) const {
  const bool as_modeled = include_ephemeral_elements && forest_is_valid();
  std::string graphviz = "digraph LinkJointGraph {\n";
  graphviz += "rankdir=BT;\n";
  graphviz += "labelloc=t;\n";
  graphviz += fmt::format("label=\"{}\nLinkJointGraph{}\";\n", label,
                          as_modeled ? "+" : "");

  // Generate a legend.
  graphviz += "legend [shape=none]\n";
  graphviz +=
      "[label=\""
      "* = massless"
      "\nL/J(i) link/joint(ordinal)"
      "\nname:index";
  if (as_modeled) graphviz += "\nred = ephemeral";
  graphviz += "\"]\n";

  // Link composites are discovered during forest building. If there
  // is no forest, there will be no composites. But if there are composites
  // we'd like to draw boxes around them so we'll process composited links
  // first and then pick up the leftovers below.
  for (const LinkComposite& composite : link_composites()) {
    // Oddly, in order to get the box and label for a subgraph, the _name_
    // of the subgraph must begin with "cluster"!
    graphviz +=
        fmt::format("subgraph cluster{}", &composite - &link_composites()[0]) +
        " {\n";
    graphviz += fmt::format("label=\"LinkComposite({}){}\";\n",
                            &composite - &link_composites()[0],
                            composite.is_massless ? "*" : "");
    for (const BodyIndex& link_index : composite.links) {
      const Link& link = link_by_index(link_index);
      const bool ephemeral = link_is_ephemeral(link.index());
      if (ephemeral && !include_ephemeral_elements) continue;
      graphviz += fmt::format("link{} [label=\"L({}){} {}:{}\"]{};\n",
                              link.ordinal(), link.ordinal(),
                              link.is_massless() ? "*" : "", link.name(),
                              link.index(), ephemeral ? "[color=red]" : "");
    }
    graphviz += "}\n";
  }

  // Now pick up the links that aren't in a composite.
  for (const Link& link : links()) {
    if (link.composite().has_value()) continue;
    const bool ephemeral = link_is_ephemeral(link.index());
    if (ephemeral && !include_ephemeral_elements) continue;
    graphviz +=
        fmt::format("link{} [label=\"L({}){} {}:{}\"]{};\n", link.ordinal(),
                    link.ordinal(), link.is_massless() ? "*" : "", link.name(),
                    link.index(), ephemeral ? "[color=red]" : "");
  }

  // Draw the joints as edges, with various representations. Note that a
  // loop joint gets retargeted to a shadow body; we show that and also
  // the original parent/child connection (as a dashed gray edge).
  for (const Joint& joint : joints()) {
    const JointTraits& traits = joint_traits(joint.traits_index());
    const bool ephemeral = joint_is_ephemeral(joint.index());
    if (ephemeral && !include_ephemeral_elements) continue;
    const std::string color = ephemeral               ? "red"
                              : traits.name == "weld" ? "black"
                                                      : "green";

    const std::string arrow_type = ephemeral               ? "empty"
                                   : traits.name == "weld" ? "box"
                                                           : "normal";
    const std::string style = traits.name == "weld" ? "bold" : "solid";

    const LinkOrdinal parent_ordinal =
        link_by_index(joint.parent_link_index()).ordinal();
    const LinkOrdinal child_ordinal =
        link_by_index(joint.child_link_index()).ordinal();

    // If this is a loop joint one of these links will be revised to
    // be the shadow link (if we're generating the "as modeled" graph).
    LinkOrdinal revised_parent_ordinal = parent_ordinal;
    LinkOrdinal revised_child_ordinal = child_ordinal;
    if (include_ephemeral_elements && joint.mobod_index().is_valid()) {
      const SpanningForest::Mobod& mobod = forest().mobods(joint.mobod_index());
      if (mobod.is_reversed())
        revised_parent_ordinal = mobod.link_ordinal();
      else
        revised_child_ordinal = mobod.link_ordinal();
    }

    // Draw the effective joint connection.
    graphviz += fmt::format(
        "link{} -> link{} [arrowhead={}] [style={}] [fontsize=10] "
        "[label=\"J({}) {}:{}\n{}\"] [color={}];\n",
        revised_parent_ordinal, revised_child_ordinal, arrow_type, style,
        joint.ordinal(), joint.name(), joint.index(),
        joint_traits(joint.traits_index()).name, color);

    // Draw the original joint connection if we retargeted it to a shadow.
    if (revised_parent_ordinal != parent_ordinal ||
        revised_child_ordinal != child_ordinal) {
      graphviz += fmt::format(
          "link{} -> link{} [arrowhead={}] [color=gray] [style=dashed] "
          "[label=\"loop\nJ({})\"] [fontsize=10];\n",
          parent_ordinal, child_ordinal, arrow_type, joint.ordinal());
    }
  }

  // Draw the ephemeral weld constraints added to connect shadow to primary.
  if (include_ephemeral_elements) {
    for (const LoopConstraint& constraint : loop_constraints()) {
      graphviz += fmt::format(
          "link{} -> link{} [dir=both] [arrowhead=obox] [arrowtail=obox] "
          "[style=bold] [fontsize=10] [label=\"WELD({})\n{}\"] [color=red]\n",
          link_by_index(constraint.primary_link()).ordinal(),
          link_by_index(constraint.shadow_link()).ordinal(), constraint.index(),
          constraint.name());
    }
  }

  graphviz += "}\n";
  return graphviz;
}

std::filesystem::path LinkJointGraph::MakeGraphvizFiles(
    std::filesystem::path where, const std::string_view basename) const {
  // Find the "dot" command.
  std::string dot;
  if (std::filesystem::exists("/usr/bin/dot")) {
    dot = "/usr/bin/dot";
  } else if (std::filesystem::exists("/usr/local/bin/dot")) {
    dot = "/usr/local/bin/dot";
  } else if (std::filesystem::exists("/bin/dot")) {
    dot = "/bin/dot";
  } else {
    throw std::runtime_error(fmt::format(
        "{}(): Graphviz 'dot' is required but missing. It must be in "
        "/usr/bin/dot, /usr/local/bin/dot, or /bin/dot. Install it in one "
        "of those places if you want to use this function."
        "See https://graphviz.org.",
        __func__));
  }

  // Default to the current directory if none specified.
  if (where.empty()) where = std::filesystem::current_path();

  // This lambda creates one png given a name and generator. The dot file
  // is removed after use.
  auto MakePng = [&dot, &where, &basename](
                     std::string_view suffix,
                     std::function<std::string()> generate) {
    // Create the dot file.
    std::filesystem::path dotname(
        std::filesystem::absolute(std::filesystem::path(where).append(
            fmt::format("{}_{}.dot", basename, suffix))));
    std::ofstream dotfile(dotname);
    if (!dotfile.good()) {
      throw std::runtime_error(fmt::format(
          "MakeGraphvizFiles(): can't create file {}.", dotname.string()));
    }
    dotfile << generate();
    dotfile.close();

    // Convert the dot file to a png.
    const std::string cmd = fmt::format(
        "{} -Tpng {} >{}", dot, dotname.string(),
        std::filesystem::path(dotname).replace_extension("png").string());
    int status = system(cmd.c_str());
    if (status != 0) {
      throw std::runtime_error(
          fmt::format("{}(): failed to execute command\n{}", __func__, cmd));
    }

    std::filesystem::remove(dotname);
  };

  MakePng("graph", [this, &basename]() {
    return GenerateGraphvizString(basename, false);
  });

  if (forest_is_valid()) {
    MakePng("graph+", [this, &basename]() {
      return GenerateGraphvizString(basename, true);
    });

    MakePng("forest", [this, &basename]() {
      return forest().GenerateGraphvizString(basename);
    });
  }

  return std::filesystem::absolute(where);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
