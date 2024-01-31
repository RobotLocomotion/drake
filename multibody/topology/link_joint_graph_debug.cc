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
  for (JointTypeIndex i(0); i < ssize(data_.joint_types); ++i) {
    const JointType& type = data_.joint_types[i];
    std::cout << fmt::format("  {}: {} nq={} nv={} has_quat={}\n", i,
                             type.type_name, type.nq, type.nv,
                             type.has_quaternion);
  }
  std::cout << "User Links:\n";
  for (const auto& link : links()) {
    if (link.index() == num_user_links()) std::cout << "Ephemeral Links:\n";
    std::cout << fmt::format("  {}: {} inst={}{}{}{}{}{}{}", link.index(),
                             link.name(), link.model_instance(),
                             link.is_world() ? " World" : "",
                             link.is_static() ? " static" : "",
                             link.is_anchored() ? " anchored" : "",
                             link.must_be_base_body() ? " must_be_base" : "",
                             link.treat_as_massless() ? " massless" : "",
                             link.is_shadow() ? " shadow" : "");
    if (link.mobod_index().is_valid()) {
      std::cout << fmt::format(" mobod={}", link.mobod_index());
    }
    if (link.inboard_joint_index().is_valid()) {
      std::cout << fmt::format(" joint={}", link.inboard_joint_index());
    }
    if (link.composite().is_valid()) {
      std::cout << fmt::format(" composite={}", link.composite());
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
    for (const BodyIndex& li : link_composites()[i])
      std::cout << fmt::format("{} ", li);
    std::cout << "\n";
  }
  std::cout << "User Joints:\n";
  for (const Joint& joint : joints()) {
    if (joint.index() == num_user_joints()) std::cout << "Ephemeral Joints:\n";
    std::cout << fmt::format(
        "  {}: {} inst={} {} parent={} child={}\n", joint.index(), joint.name(),
        joint.model_instance(), joint_types(joint.type_index()).type_name,
        joint.parent_link(), joint.child_link());
  }
  if (ssize(joints()) == num_user_joints())
    std::cout << "(No ephemeral Joints)\n";

  std::cout << "Loop Constraints (always ephemeral):\n";
  for (const LoopConstraint& constraint : loop_constraints()) {
    std::cout << fmt::format("  {}: {} inst={} parent={} child={}\n",
                             constraint.index(), constraint.name(),
                             constraint.model_instance(),
                             constraint.parent_link(), constraint.child_link());
  }
  if (ssize(joints()) == num_user_joints())
    std::cout << "(No added Model Joints)\n";
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
