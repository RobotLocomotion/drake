#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <fmt/format.h>

#include "drake/multibody/topology/forest.h"
#include "drake/multibody/topology/graph.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Consider accommodation for colorblind people. Avoid red/green
//  and/or use line styles. Update the legend.

std::string LinkJointGraph::GenerateGraphvizString(
    std::string_view label, bool include_ephemeral_elements) const {
  const bool show_as_modeled = include_ephemeral_elements && forest_is_valid();
  std::string graphviz = "digraph LinkJointGraph {\n";
  graphviz += "rankdir=BT;\n";
  graphviz += "labelloc=t;\n";
  graphviz += fmt::format("label=\"{}\nLinkJointGraph{}\";\n", label,
                          show_as_modeled ? "+" : "");

  // Generate a legend.
  graphviz += "legend [shape=none]\n";
  graphviz +=
      "[label=\""
      "* = massless"
      "\nL/J(i) link/joint(ordinal)"
      "\nname:index";
  if (show_as_modeled) graphviz += "\nred = ephemeral";
  graphviz += "\"]\n";

  // WeldedLinkAssemblies are discovered during forest building. If there
  // is no forest, there will be no assemblies. But if there are assemblies
  // we'd like to draw boxes around them so we'll process assembled links
  // first and then pick up the leftovers below.
  for (WeldedLinksAssemblyIndex index{0};
       index < ssize(welded_links_assemblies()); ++index) {
    const WeldedLinksAssembly& assembly = welded_links_assemblies(index);
    // Oddly, in order to get the box and label for a subgraph, the _name_
    // of the subgraph must begin with "cluster"!
    graphviz += fmt::format("subgraph cluster{}", index) + " {\n";
    graphviz += fmt::format("label=\"WeldedLinksAssembly({}){}\";\n", index,
                            assembly.is_massless() ? "*" : "");
    for (const LinkIndex& link_index : assembly.links()) {
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

  // Now pick up the links that aren't in an assembly.
  for (const Link& link : links()) {
    if (link.welded_links_assembly().has_value()) continue;
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
    if (show_as_modeled && joint.mobod_index().is_valid()) {
      const SpanningForest::Mobod& mobod = forest().mobods(joint.mobod_index());
      if (mobod.is_reversed()) {
        revised_parent_ordinal = mobod.link_ordinal();
      } else {
        revised_child_ordinal = mobod.link_ordinal();
      }
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
  if (show_as_modeled) {
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
  const std::array<std::string, 4> search{"/usr/bin/dot", "/usr/local/bin/dot",
                                          "/opt/homebrew/bin/dot", "/bin/dot"};
  std::string dot_command;
  for (const auto& command : search) {
    if (std::filesystem::exists(command)) {
      dot_command = command;
      break;
    }
  }
  if (dot_command.empty()) {
    throw std::runtime_error(fmt::format(
        "{}(): Graphviz 'dot' is required but missing. It must be in "
        "{}, {}, {}, or {}. Install it in one of those places if you want to "
        "use this function. See https://graphviz.org.",
        __func__, search[0], search[1], search[2], search[3]));
  }

  // Default to the current directory if none specified.
  if (where.empty()) where = std::filesystem::current_path();

  // This lambda creates one png given a name and generator. The dot file
  // is removed after use.
  auto MakePng = [&dot_command, &where, &basename](
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
        "{} -Tpng {} >{}", dot_command, dotname.string(),
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
