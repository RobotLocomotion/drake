#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Add more tests. This doesn't check everything.

namespace {
// This is a helper for checking assemblies. It populates the link_to_level
// map for each link of an assembly with the distance of that link from the
// active (most inboard) link of the assembly.
void RecordLevelOfOutboardLinks(const SpanningForest& forest,
                                JointIndex previous_joint_index,
                                LinkIndex inboard_index,
                                std::map<LinkIndex, int>* link_to_level) {
  DRAKE_DEMAND(link_to_level->contains(inboard_index));
  const LinkJointGraph::Link& inboard_link =
      forest.link_by_index(inboard_index);
  const int inboard_level = (*link_to_level)[inboard_index];
  for (const auto& joint_index : inboard_link.joints()) {
    if (previous_joint_index.is_valid() && joint_index == previous_joint_index)
      continue;  // Skip the one that got us here.
    const LinkJointGraph::Joint& joint = forest.joint_by_index(joint_index);
    if (!joint.is_weld()) continue;
    if (inboard_link.JointHasMovedToShadowLink(joint_index))
      continue;  // We'll find it on the shadow's branch.
    const LinkIndex effective_outboard_link_index =
        joint.other_effective_link_index(inboard_index);
    if (link_to_level->contains(effective_outboard_link_index))
      continue;  // We found a loop.
    (*link_to_level)[effective_outboard_link_index] = inboard_level + 1;
    RecordLevelOfOutboardLinks(forest, joint_index,
                               effective_outboard_link_index, &*link_to_level);
  }
}
}  // namespace

void SpanningForest::SanityCheckForest() const {
  // Should always have a LinkJointGraph backpointer even if empty.
  DRAKE_THROW_UNLESS(data_.graph != nullptr);
  if (mobods().empty()) return;

  const Mobod& world_mobod = mobods(MobodIndex(0));
  DRAKE_THROW_UNLESS(world_mobod.is_world());
  DRAKE_THROW_UNLESS(!world_mobod.is_base_body());
  DRAKE_THROW_UNLESS(world_mobod.is_anchored());
  DRAKE_THROW_UNLESS(world_mobod.has_massful_follower_link());
  DRAKE_THROW_UNLESS(world_mobod.index() == MobodIndex(0));
  DRAKE_THROW_UNLESS(world_mobod.link_ordinal() == LinkOrdinal(0));
  DRAKE_THROW_UNLESS(!world_mobod.joint_ordinal().is_valid());
  DRAKE_THROW_UNLESS(!world_mobod.tree().is_valid());
  DRAKE_THROW_UNLESS(world_mobod.level() == 0);
  DRAKE_THROW_UNLESS(!world_mobod.inboard().is_valid());
  DRAKE_THROW_UNLESS(ssize(world_mobod.outboards()) == ssize(trees()));

  /* WeldedMobods groups and WeldedLinksAssemblies are different but in either
  case there is always a World group or assembly, it must come first, and the
  World Link or Mobod must be listed first in that group or assembly. */
  DRAKE_THROW_UNLESS(world_mobod.welded_mobods_group().has_value());
  DRAKE_THROW_UNLESS(world_mobod.welded_mobods_group() == WeldedMobodsIndex(0));
  DRAKE_THROW_UNLESS(welded_mobods()[0][0] == MobodIndex(0));

  const LinkJointGraph::Link& world_link = links(LinkOrdinal(0));
  DRAKE_THROW_UNLESS(world_link.mobod_index() == 0);
  DRAKE_THROW_UNLESS(world_link.welded_links_assembly().has_value());
  DRAKE_THROW_UNLESS(world_link.welded_links_assembly() ==
                     WeldedLinksAssemblyIndex(0));
  DRAKE_THROW_UNLESS(graph().welded_links_assemblies()[0].links()[0] ==
                     LinkIndex(0));

  for (MobodIndex index(1); index < ssize(mobods()); ++index) {
    const Mobod& mobod = mobods(index);
    DRAKE_THROW_UNLESS(mobod.index() == index);
    DRAKE_THROW_UNLESS(links(mobod.link_ordinal()).mobod_index() == index);
    DRAKE_THROW_UNLESS(joints(mobod.joint_ordinal()).mobod_index() == index);

    // The mobod's Tree must include the mobod.
    const Tree& tree = trees(mobod.tree());
    DRAKE_THROW_UNLESS(tree.base_mobod() <= mobod.index() &&
                       mobod.index() <= tree.last_mobod());

    // If the mobod is part of a WeldedMobods group, that group must include it!
    if (mobod.welded_mobods_group().has_value()) {
      const auto& welded_group = welded_mobods()[*mobod.welded_mobods_group()];
      DRAKE_THROW_UNLESS(std::find(welded_group.begin(), welded_group.end(),
                                   index) != welded_group.end());
    }
  }

  /* Check that Links alleged to be in a WeldedLinksAssembly agree that they
  are, and that massless WeldedLinksAssemblies are composed only of massless
  Links. */
  for (WeldedLinksAssemblyIndex assembly_index(0);
       assembly_index < ssize(graph().welded_links_assemblies());
       ++assembly_index) {
    const LinkJointGraph::WeldedLinksAssembly& assembly =
        graph().welded_links_assemblies(assembly_index);
    bool all_links_are_massless = true;
    for (const LinkIndex& link_index : assembly.links()) {
      const LinkJointGraph::Link& link = link_by_index(link_index);
      DRAKE_THROW_UNLESS(link.welded_links_assembly() == assembly_index);
      if (!link.is_massless()) all_links_are_massless = false;
    }
    DRAKE_THROW_UNLESS(assembly.is_massless() == all_links_are_massless);
  }

  /* Check that Links in a WeldedLinksAssembly are listed in inboard->outboard
  order. Starting with the active (most inboard) link which we'll assign
  level 0, we'll determine a level for each link equal to its distance
  (number of edges) from the active link. Then we'll go through each joint
  in the assembly to see that its inboard link is listed before its outboard
  link. (If there was a loop within the assembly we might find the levels
  are the same for a joint's links. In that case the order is arbitrary.) */
  for (WeldedLinksAssemblyIndex assembly_index(0);
       assembly_index < ssize(graph().welded_links_assemblies());
       ++assembly_index) {
    const LinkJointGraph::WeldedLinksAssembly& assembly =
        graph().welded_links_assemblies(assembly_index);
    // Map each link index to its position in the links vector.
    std::map<LinkIndex, int> link_to_position;
    for (int i = 0; i < ssize(assembly.links()); ++i)
      link_to_position[assembly.links()[i]] = i;
    // Map each link to its level (distance from active link).
    std::map<LinkIndex, int> link_to_level;
    const LinkIndex active_link_index = assembly.links().at(0);
    link_to_level[active_link_index] = 0;
    RecordLevelOfOutboardLinks(*this, JointIndex(), active_link_index,
                               &link_to_level);

    // Sanity check of the sanity checker: make sure we built the maps right.
    for (const LinkIndex& link_index : assembly.links()) {
      DRAKE_DEMAND(link_to_level.contains(link_index));
      DRAKE_DEMAND(link_to_position.contains(link_index));
    }

    // Each joint connects two links. Verify that the inboard link
    // appears before the outboard link.
    for (const JointIndex& joint_index : assembly.joints()) {
      const LinkIndex parent =
          graph().joint_by_index(joint_index).effective_parent_link_index();
      const LinkIndex child =
          graph().joint_by_index(joint_index).effective_child_link_index();
      const int parent_level = link_to_level[parent];
      const int parent_pos = link_to_position[parent];
      const int child_level = link_to_level[child];
      const int child_pos = link_to_position[child];
      if (parent_level < child_level) {
        DRAKE_THROW_UNLESS(parent_pos < child_pos);
      } else if (parent_level > child_level) {
        DRAKE_THROW_UNLESS(parent_pos > child_pos);
      }
      // If the levels are equal the order is arbitrary.
    }
  }

  /* Check that all the Links and Joints have Mobods that point back to them. */
  for (LinkOrdinal link_ordinal{0}; link_ordinal < ssize(links());
       ++link_ordinal) {
    const Link& link = links(link_ordinal);
    DRAKE_THROW_UNLESS(link.ordinal() == link_ordinal);
    DRAKE_THROW_UNLESS(&link_by_index(link.index()) == &link);
    DRAKE_THROW_UNLESS(mobods(link.mobod_index()).HasFollower(link_ordinal));
  }
  for (JointOrdinal joint_ordinal{0}; joint_ordinal < ssize(joints());
       ++joint_ordinal) {
    const Joint& joint = joints(joint_ordinal);
    DRAKE_THROW_UNLESS(joint.ordinal() == joint_ordinal);
    DRAKE_THROW_UNLESS(&joint_by_index(joint.index()) == &joint);
    if (!joint.mobod_index().is_valid()) continue;  // Not modeled
    DRAKE_THROW_UNLESS(mobods(joint.mobod_index()).joint_ordinal() ==
                       joint_ordinal);
  }

  /* Check tree height and make sure the tree's mobods agree they are part
  of the tree. Make sure the forest height is the max tree height. */
  int forest_height = 1;  // World alone has a height of 1.
  for (const auto& tree : trees()) {
    DRAKE_THROW_UNLESS(tree.forest_ == this);  // Check backpointer.
    DRAKE_THROW_UNLESS(mobods(tree.base_mobod()).level() == 1);
    DRAKE_THROW_UNLESS(tree.last_mobod() >= tree.base_mobod());
    int computed_height = 0;
    for (auto& mobod : tree) {
      DRAKE_THROW_UNLESS(mobod.tree() == tree.index());
      computed_height = std::max(computed_height, mobod.level());
    }
    DRAKE_THROW_UNLESS(tree.height() == computed_height);
    forest_height = std::max(forest_height, tree.height() + 1);
  }
  DRAKE_THROW_UNLESS(height() == forest_height);

  /* Check each WeldedMobods group to make sure:
     - the mobods it contains agree that they are contained in that group
     - mobods welded to World know they are "anchored"
     - except for the World group, the first entry in each group is a
       Mobod that has a non-weld inboard joint, and all the others are Welds.
     - the first entry in each WeldedMobods group should map to the Link (the
       "active link") that is listed first in its WeldedLinksAssembly. */
  for (WeldedMobodsIndex welded_mobods_index(0);
       welded_mobods_index < ssize(welded_mobods()); ++welded_mobods_index) {
    const std::vector<MobodIndex> welded_mobods_group =
        welded_mobods()[welded_mobods_index];
    for (MobodIndex mobod_index : welded_mobods_group) {
      const Mobod& mobod = mobods(mobod_index);
      const bool is_active_mobod = (mobod_index == welded_mobods_group[0]);
      const bool should_be_weld =  // World or an interior body.
          welded_mobods_index == 0 || !is_active_mobod;
      DRAKE_THROW_UNLESS(mobod.is_weld() == should_be_weld);
      DRAKE_THROW_UNLESS(mobod.welded_mobods_group() == welded_mobods_index);
      const bool should_be_anchored = (welded_mobods_index == 0);
      DRAKE_THROW_UNLESS(mobod.is_anchored() == should_be_anchored);

      if (is_active_mobod) {
        const LinkOrdinal active_link_ordinal = mobod.link_ordinal();
        const Link& active_link = links(active_link_ordinal);
        const std::optional<WeldedLinksAssemblyIndex> link_assembly =
            active_link.welded_links_assembly();
        DRAKE_THROW_UNLESS(link_assembly.has_value());
        DRAKE_THROW_UNLESS(
            graph().welded_links_assemblies(*link_assembly).links()[0] ==
            active_link.index());
      }
    }
  }
}

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
