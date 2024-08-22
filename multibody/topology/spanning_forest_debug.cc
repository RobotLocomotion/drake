#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

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

  /* WeldedMobods groups and LinkComposites are different but in either case
  there is always a World group or composite, it must come first, and the World
  Link or Mobod must be listed first in that group or composite. */
  DRAKE_THROW_UNLESS(world_mobod.welded_mobods_group().has_value());
  DRAKE_THROW_UNLESS(world_mobod.welded_mobods_group() == WeldedMobodsIndex(0));
  DRAKE_THROW_UNLESS(welded_mobods()[0][0] == MobodIndex(0));

  const LinkJointGraph::Link& world_link = links(LinkOrdinal(0));
  DRAKE_THROW_UNLESS(world_link.mobod_index() == 0);
  DRAKE_THROW_UNLESS(world_link.composite().has_value());
  DRAKE_THROW_UNLESS(world_link.composite() == LinkCompositeIndex(0));
  DRAKE_THROW_UNLESS(graph().link_composites()[0].links[0] == LinkIndex(0));

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

  /* Check that Links alleged to be in a LinkComposite agree that they are,
  and that massless LinkComposites are composed only of massless Links. */
  for (LinkCompositeIndex composite_index(0);
       composite_index < ssize(graph().link_composites()); ++composite_index) {
    const LinkJointGraph::LinkComposite& composite =
        graph().link_composites(composite_index);
    bool all_links_are_massless = true;
    for (const LinkIndex& link_index : composite.links) {
      const LinkJointGraph::Link& link = link_by_index(link_index);
      DRAKE_THROW_UNLESS(link.composite() == composite_index);
      if (!link.is_massless()) all_links_are_massless = false;
    }
    DRAKE_THROW_UNLESS(composite.is_massless == all_links_are_massless);
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
       "active link") that is listed first in its LinkComposite. */
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
        const std::optional<LinkCompositeIndex> link_composite =
            active_link.composite();
        DRAKE_THROW_UNLESS(link_composite.has_value());
        DRAKE_THROW_UNLESS(graph().link_composites(*link_composite).links[0] ==
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
