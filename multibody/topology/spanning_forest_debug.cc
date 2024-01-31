#include <iostream>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

void SpanningForest::SanityCheckForest() const {
  // Should always have a LinkJointGraph backpointer even if empty.
  DRAKE_DEMAND(data_.graph != nullptr);
  if (mobods().empty()) return;

  const Mobod& world_mobod = mobods(MobodIndex(0));
  DRAKE_DEMAND(world_mobod.index() == 0 && world_mobod.index() == 0);
  DRAKE_DEMAND(!world_mobod.joint().is_valid());
  DRAKE_DEMAND(world_mobod.level_ == 0);
  DRAKE_DEMAND(!world_mobod.inboard_mobod_.is_valid());
  DRAKE_DEMAND(world_mobod.outboard_mobods_.size() == trees().size());

  /* WeldedMobods groups and LinkComposites are different but in either case
  there is always a World group or composite, it must come first, and the World
  Link or Mobod must be listed first in that group or composite. */
  DRAKE_DEMAND(world_mobod.welded_mobods_group().is_valid());
  DRAKE_DEMAND(world_mobod.welded_mobods_group() == WeldedMobodsIndex(0));
  DRAKE_DEMAND(welded_mobods()[0][0] == MobodIndex(0));

  const LinkJointGraph::Link& world_link = links(BodyIndex(0));
  DRAKE_DEMAND(world_link.mobod_index() == 0);
  DRAKE_DEMAND(world_link.composite().is_valid());
  DRAKE_DEMAND(world_link.composite() == LinkCompositeIndex(0));
  DRAKE_DEMAND(graph().link_composites()[0][0] == BodyIndex(0));

  for (MobodIndex index(1); index < ssize(mobods()); ++index) {
    const Mobod& mobod = mobods(index);
    DRAKE_DEMAND(mobod.index() == index);
    DRAKE_DEMAND(links(mobod.link()).mobod_index() == index);
    DRAKE_DEMAND(joints(mobod.joint()).mobod_index() == index);

    // The mobod's Tree must include the mobod.
    const Tree& tree = trees(mobod.tree_index_);
    DRAKE_DEMAND(tree.base_mobod_ <= mobod.index() &&
                 mobod.index() <= tree.last_mobod_);

    // If the mobod is part of a WeldedMobods group, that group must include it!
    if (mobod.welded_mobods_group().is_valid()) {
      const auto& welded_group = welded_mobods()[mobod.welded_mobods_group()];
      DRAKE_DEMAND(std::find(welded_group.begin(), welded_group.end(), index) !=
                   welded_group.end());
    }
  }

  /* Check that added Links and Joints have assigned Mobods that point
  back to them. */
  for (BodyIndex link_index(graph().num_user_links());
       link_index < ssize(links()); ++link_index) {
    const Link& link = links(link_index);
    DRAKE_DEMAND(mobods(link.mobod_index()).link() == link_index);
  }
  for (JointIndex joint_index(graph().num_user_joints());
       joint_index < ssize(joints()); ++joint_index) {
    const Joint& joint = joints(joint_index);
    if (!joint.mobod_index().is_valid()) continue;  // Not modeled with Mobod
    DRAKE_DEMAND(mobods(joint.mobod_index()).joint() == joint_index);
  }

  /* Check that _all_ the Links and Joints match their assigned Mobods.
  (Repeating for the added ones.) */
  for (BodyIndex link_index(graph().num_user_links());
       link_index < ssize(links()); ++link_index) {
    const Link& link = links(link_index);
    DRAKE_DEMAND(mobods(link.mobod_index()).link() == link_index);
  }
  for (JointIndex joint_index(graph().num_user_joints());
       joint_index < ssize(joints()); ++joint_index) {
    const Joint& joint = joints(joint_index);
    if (!joint.mobod_index().is_valid()) continue;  // Not modeled
    DRAKE_DEMAND(mobods(joint.mobod_index()).joint() == joint_index);
  }

  /* Check tree height and make sure the tree's mobods agree they are part
  of the tree. Make sure the forest height is the max tree height. */
  int forest_height = 1;  // World alone has a height of 1.
  for (const auto& tree : data_.trees) {
    DRAKE_DEMAND(tree.forest_ == this);
    DRAKE_DEMAND(mobods(tree.base_mobod_).level() == 1);
    DRAKE_DEMAND(tree.last_mobod_ >= tree.base_mobod_);
    int computed_height = 0;
    for (auto& mobod : tree) {
      DRAKE_DEMAND(mobod.tree_index_ == tree.index());
      computed_height = std::max(computed_height, mobod.level_);
    }
    DRAKE_DEMAND(tree.height() == computed_height);
    forest_height = std::max(forest_height, tree.height() + 1);
  }
  DRAKE_DEMAND(height() == forest_height);

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
    for (MobodIndex index : welded_mobods_group) {
      const Mobod& mobod = mobods(index);
      const bool is_active_mobod = index == welded_mobods_group[0];
      const bool should_be_weld =  // World or an interior body.
          welded_mobods_index == 0 || !is_active_mobod;
      DRAKE_DEMAND(mobod.is_weld() == should_be_weld);
      DRAKE_DEMAND(mobod.welded_mobods_group().is_valid() &&
                   mobod.welded_mobods_group() == welded_mobods_index);
      DRAKE_DEMAND((welded_mobods_index != 0) ^ mobod.is_anchored());

      if (is_active_mobod) {
        const BodyIndex active_link = mobod.link();
        const LinkCompositeIndex link_composite =
            links(active_link).composite();
        DRAKE_DEMAND(link_composite.is_valid());
        DRAKE_DEMAND(graph().link_composites(link_composite)[0] == active_link);
      }
    }
  }
}

namespace {
// Handy for writing out the links that comprise a Composite.
std::ostream& operator<<(std::ostream& o, const std::vector<BodyIndex>& v) {
  for (int i = 0; i < ssize(v); ++i) {
    if (i > 0) o << "+";
    o << v[i];
  }
  return o;
}

std::ostream& operator<<(std::ostream& o, ForestBuildingOptions options) {
  if (options == ForestBuildingOptions::kDefault) {
    o << "Default";
  } else {
    if ((options & ForestBuildingOptions::kStatic) ==
        ForestBuildingOptions::kStatic)
      o << " Static";
    if ((options & ForestBuildingOptions::kUseRpyFloatingJoints) ==
        ForestBuildingOptions::kUseRpyFloatingJoints)
      o << " UseRpy";
    if ((options & ForestBuildingOptions::kCombineLinkComposites) ==
        ForestBuildingOptions::kCombineLinkComposites)
      o << " CombineComposites";
    if ((options & ForestBuildingOptions::kUseFixedBase) ==
        ForestBuildingOptions::kUseFixedBase)
      o << " UseFixedBase";
  }
  return o;
}

}  // namespace

void SpanningForest::DumpForest(std::string title) const {
  std::cout << "\n\n******** FOREST " << title << "\n";
  std::cout << "  options=" << options();
  for (auto it = graph().data_.model_instance_forest_building_options.begin();
       it != graph().data_.model_instance_forest_building_options.end(); ++it) {
    std::cout << " mi" << it->first << ": " << options(it->first);
  }
  std::cout << "\n  height=" << height() << "\n";
  std::cout << "\nWorld " << world_mobod().index()
            << (world_mobod().is_anchored() ? "A" : " ");
  std::cout << " L" << world_mobod().follower_links() << "\n";
  for (const auto& tree : trees()) {
    std::cout << "Tree " << tree.index() << " base " << tree.base_mobod()
              << " last " << tree.last_mobod()
              << " num_mobods=" << tree.num_mobods() << " q@" << tree.q_start()
              << ":" << tree.nq() << " v@" << tree.v_start() << ":" << tree.nv()
              << " height=" << tree.height() << "\n";
    std::cout << "Mobod path World->last: ";
    for (MobodIndex i : FindPathFromWorld(tree.last_mobod()))
      std::cout << i << " ";
    std::cout << "\nLink path World->last: ";
    for (BodyIndex i : graph().FindPathFromWorld(tree.back().link()))
      std::cout << i << " ";
    std::cout << "\n";
    DumpForestImpl(tree.base_mobod(), 2);
  }

  std::cout << "added joint(mobod):\n";
  for (JointIndex joint_index(graph().num_user_joints());
       joint_index < ssize(joints()); ++joint_index) {
    const Joint& joint = joints()[joint_index];
    std::cout << joint.index() << ":";
    std::cout << graph().joint_types(joint.type_index()).type_name;
    std::cout << " " << joint.name() << " -> ";
    if (joint.mobod_index().is_valid())
      std::cout << "mobod=" << joint.mobod_index() << "\n";
    else
      std::cout << "(not modeled)\n";
  }

  if (!loop_constraints().empty()) {
    std::cout << "\nadded loop constraints:\n";
    for (LoopConstraintIndex i(0); i < ssize(loop_constraints()); ++i) {
      const LoopConstraint& constraint = loop_constraints()[i];
      std::cout << fmt::format("{} mobods: {} {}  loop constraint: {}\n", i,
                               constraint.parent_mobod_index_,
                               constraint.child_mobod_index_,
                               constraint.loop_constraint_index_);
    }
  }

  std::cout << "\nlink to its mobod:\n";
  for (const auto& link : links()) std::cout << link.mobod_index() << ' ';
  std::cout << "\njoint_to_mobod:\n";
  for (const auto& joint : joints()) {
    const MobodIndex i = joint.mobod_index();
    if (!i.is_valid())
      std::cout << "IGN ";
    else
      std::cout << i << ' ';
  }

  std::cout << "\nComposite Mobods:\n";
  for (WeldedMobodsIndex ci(0); ci < ssize(welded_mobods()); ++ci) {
    std::cout << ci << ": ";
    for (MobodIndex i : welded_mobods()[ci]) std::cout << i << " ";
    std::cout << "\n";
  }
}

void SpanningForest::DumpForestImpl(MobodIndex index, int level) const {
  const Mobod& mobod = mobods(index);

  for (int i = 0; i < level; ++i) std::cout << ' ';
  std::cout << index << (mobod.is_anchored() ? "A" : " ");
  std::cout << " L" << mobod.follower_links();
  if (mobod.joint().is_valid()) {
    std::cout << " J" << mobod.joint();
    std::cout << (mobod.use_reverse_mobilizer_ ? "R" : " ");
    std::cout << " q@" << mobod.q_start() << ":" << mobod.nq();
    std::cout << " v@" << mobod.v_start() << ":" << mobod.nv();
  } else {
    std::cout << " JOINT INVALID";
  }
  std::cout << "\n";

  for (MobodIndex outboard : mobod.outboards())
    DumpForestImpl(outboard, level + 2);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
