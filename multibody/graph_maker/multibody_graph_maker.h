#pragma once

/* Adapted for Drake from Simbody's MultibodyGraphMaker class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Modifications copyright (c) 2019 Toyota Research Institute, Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

/** @file
Declares the MultibodyGraphMaker class for use in constructing a
spanning-tree-plus-constraints representation of a multibody system from a
list of its links and joints. */

#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/multibody/graph_maker/multibody_graph.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

//==============================================================================
//                          MULTIBODY GRAPH MAKER
//==============================================================================
/** Construct a reasonably good spanning-tree-plus-constraints structure for
modeling a given set of links and joints with a generalized coordinate
multibody system like Drake or Simbody. The idea is to map a directed graph
of _links_ and _joints_, that is possibly cyclic and disconnected, to an output
graph consisting of a fully-connected tree of _bodies_ and _mobilizers_, plus a
set of _constraints_ to enforce loop connectivity. This is particularly useful
when parsing an `.sdf` or `.urdf` file.

<h3>A basic example</h3>
Here is a usage example, assuming the given input is: <pre>
       link1 <-- link2 <-- link3 --> link4 --> link5
</pre> where the arrows represent revolute joints and point from the "parent"
link to the "child" link. Note that no explicit connection to World (link 0) has
been given, so we're expecting the %MultibodyGraphMaker to add one.
@code
  MultibodyGraphMaker maker;
  // "world" is link 0
  maker.AddLink("link1");  // name (optional flags)
  maker.AddLink("link2");
  maker.AddLink("link3");
  maker.AddLink("link4");
  maker.AddLink("link5");
  maker.AddJoint("joint0", "revolute", "link2", "link1");  // parent, child
  maker.AddJoint("joint1", "revolute", "link3", "link2");
  maker.AddJoint("joint2", "revolute", "link3", "link4");
  maker.AddJoint("joint3", "revolute", "link4", "link5");
  std::unique_ptr<MultibodyGraph> graph = maker.MakeGraph();

  for (int i=0; i < graph.num_mobilizers(); ++i) {
    auto& mobilizer = graph.get_mobilizer(i);
    auto& inb = graph.body(mobilizer.inboard_body_num());
    auto& outb = graph.body(mobilizer.outboard_body_num());
    std::cout << i << ": " << inb.name() << " --> " << outb.name()
              << " (" << mobilizer.get_joint_type_name() << ")\n";
  }
@endcode

The above outputs <pre>
  0: world --> link3 (free)
  1: link3 --> link2 (revolute)
  2: link3 --> link4 (revolute)
  3: link2 --> link1 (revolute)
  4: link4 --> link5 (revolute)
</pre>
showing that the algorithm picked link3 as the base body and was thus able to
preserve parent->child ordering for every joint. In actual use, the contents
of the mobilizer for-loop above would _build_ the multibody system rather than
_print_ it.

<h3>Explanation</h3>
Each link has a unique name; each joint connects two distinct links with one
designated as the "parent" link and the other the "child" link. We will output
a spanning tree with World as the root, with a body corresponding to every link,
and containing a mobilizer for every body. You can also think of this graph as a
forest of trees each with a "base body" as its root, where a base body is a
body directly connected to World (often by a "free" or "floating" joint).

There is a mobilizer in the generated graph corresponding to each of
the joints from the input, with an "inboard" (topologically closer to World)
and "outboard" (further from World) body, chosen from the parent and child
bodies of the joint but possibly reordered in which case the mobilizer is
marked as "reversed". Additional "free" mobilizers are added as needed between
bodies and World so that there is a path from every body in the inboard
direction all the way to World. Additional bodies and constraints are added
if topological loops are found in the input.

Heuristics here attempt to balance a number of competing goals:
  - choose "sensible" base bodies for trees that don't have a connection to
    World provided in the input,
  - make the mobilizer inboard/output ordering follow the parent/child ordering
    in the input,
  - cut loops such that branch lengths are balanced (minimizes the maximum
    length of any branch for better numerics),
  - make sure no massless body ends up as a terminal body of a tree (that would
    make the mass matrix singular),
  - follow any stated user preferences about the generated graph.

See below for more details.

<h3>Results</h3>
The output is
  -# A sequence of mobilizers (tree joints) in ascending order from World to a
set of terminal bodies, for each subtree. Every input link will be mobilized by
one of these, and there may also be some additional mobilized "slave" bodies due
to loop-breaking.
  -# A set of "loop constraints", representing either welds to attach slave
bodies to their masters, or loop joint constraints that correspond to particular
input joints.
  -# A correspondence between input links and mobilized bodies, with some
mobilized bodies designated as slaves to particular master bodies. That
is, more than one mobilized body may correspond to the same input link.
  -# A correspondence between mobilizers and input joints, with some extra
mobilizers having been added to connect base bodies to World.
  -# Statistics, diagnostics, and introspection of the resulting trees.

Then to build a multibody model
  -# Run through the generated mobilizers in order, adding one mobilized body to
the tree each time, using "reversed" mobilizers if appropriate (that just
affects the interpretation of the generalized coordinates). Mass properties and
joint placement information are obtained from the original links and joints;
they are not stored here.
  -# Run through the list of constraints adding master-slave welds or loop
joint constraints as indicated.

<h3>Inputs</h3>
  - links: name, mass, must_be_base_body flag
  - joints: name, type, parent link, child link, must_be_loop_joint flag
  - joint type: name, # dofs, have_good_joint_constraint_available flag
  - names for the World body and important joint types weld and free (Drake
    defaults are available).

The first link added (link 0) is assumed to be World and its name is used for
recognizing World connections later in joints. If you accept the Drake defaults,
that body will be named "world". The default names for Weld (0 dof) and Free
(6 dof) joints are, not surprisingly, "weld" and "free" but you can change them.

<h3>Loop joints</h3>
Normally every joint produces a corresponding mobilizer. A joint that would form
a kinematic loop is marked as such, and then its child link is split to form a
new "slave" body that can be mobilized by the joint. We expect that in the
constructed multibody model, a master body and its slaves will be reconnected
via weld constraints. This provides for uniform treatment of all joints, such
as providing revolute joint coordinates that can wrap. However, for some joint
types that doesn't matter because the multibody system has equally good "loop
joint" constraints. For example, Simbody's Ball mobilizer uses quaternions for
orientation and hence has no wrapping; it can be replaced with a Ball constraint
which removes the 3-dof mobilizer and 6 weld constraint equations, leaving just
3 translational constraints and indistinguishable behavior. Similarly a loop-
closing Weld joint can just be replaced by a Weld constraint, and a loop-
closing Free joint can simply be ignored.

The algorithm normally decides which joints are the loop-breakers, however you
can specify in the input that certain joints must be loop joints.

<h3>Massless links</h3>
Massless links can be very useful in defining compound joints, by composing
revolute and prismatic joints, for example. This is fine in an internal
coordinate code as long as no massless (or inertialess) body is a terminal (most
outboard) body in the spanning tree. (Terminal massless bodies are OK if their
inboard mobilizer has no dofs, i.e. is a Weld.) Links can have a mass provided;
if a movable link has zero mass the algorithm will try to avoid ending any
branch of the spanning tree with that link's body. If it fails, the resulting
spanning tree is no good and an exception will be thrown. The attempt to find
an acceptable tree is heuristic and can fail in difficult cases even if there
might be a solution. Explicit user guidance may be required in that case (for
example, specifying `must_be_base_body` for some link.)

<h3>Base bodies</h3>
A body in the spanning tree that is directly connected to World is called a
"base" body. If its mobilizer is a Free joint then it can be given an arbitrary
pose with respect to World and everything outboard of it moves together. This
is a nice property and you can influence this choice by providing an explicit
joint to World or designating some bodies as base bodies. Note that although
you will then have a set of generalized coordinates permitting you to place
these bodies arbitrarily, there is no guarantee that such placement will satisfy
constraints. If you don't designate base bodies, the algorithm will pick
them heuristically, which often leads to a good choice. The heuristic is to
attempt to find a body that does not ever appear as a child in the input.
Failing that, pick the body that has the most children. In case of tie, pick
the first body among the tied ones.

<h3>World body</h3>
The first link defined in the input will be interpreted as the World body.
Its name will be used to recognize joints that connect other bodies to World.
Typical names are "world" or "ground". If you accept the Drake defaults, the
World body will be predefined as body zero and will be named "world".

<h3>Levels</h3>
Each body in the spanning tree will be assigned a level, an integer that
measures how many edges must be traversed to get from that body to World.
World is at level 0, bodies that have a direct connection to World (base
bodies) are at level 1, bodies whose mobilizer connects them to base bodies
are at level 2, and so on. Multibody models need to be constructed in order
of increasing level, so that the inboard body is already in the tree when the
outboard body is added. We consider the level of a mobilizer to be
the same as the level of its outboard body. Loop joints are not mobilizers and
do not have a level. */
class MultibodyGraphMaker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyGraphMaker)

  using LinkNum = MultibodyGraph::LinkNum;
  using JointNum = MultibodyGraph::JointNum;
  using JointTypeNum = MultibodyGraph::JointTypeNum;

  using LinkFlags = MultibodyGraph::LinkFlags;
  using JointFlags = MultibodyGraph::JointFlags;
  using JointTypeFlags = MultibodyGraph::JointTypeFlags;

  // Local classes.
  class Link;
  class Joint;
  class JointType;

  /** Construct an empty %MultibodyGraphMaker object and set the default
  names for Weld and Free joints to "weld" and "free". Also sets up Drake's
  World body (named "world") and supported joint types, unless
  `use_drake_defaults` is set `false`. In that case the first added link (link
  number 0) is the World body and its name is used to represent World. */
  explicit MultibodyGraphMaker(bool use_drake_defaults = true);

  /** Restores this %MultibodyGraphMaker object to its just-constructed
  condition. See the constructor documentation for the meaning of the
  parameter. */
  void Clear(bool use_drake_defaults = true);

  /** @name         Define the input graph (links and joints)
  Methods in this section are used to inform %MultibodyGraphMaker about the
  links and joints as given, typically in an .sdf or .urdf file. You
  can also inspect the set of links and joints, and remove links and joints
  from an existing specification. */
  //@{

  /** Add a new link to the set of input links. Each of these will produce at
  least one Body in the MultibodyGraph.
  @param[in]      link_name
      A unique string identifying this link. There are no other restrictions
      on the contents of `name`. If this %MultibodyGraphMaker was constructed
      without Drake defaults, the first link you add is considered to be
      World, and its name is remembered and recognized when used in joints.
  @param[in]      model_instance
      The model instance to which this link belongs. If none is specified then
      the body belongs to the default model instance. This is simply a tag that
      is carried with the link and bodies generated from it; it has no effect
      on the structure of the generated graph. However, it does permit queries
      that return only elements of a particular model instance.
  @param[in]      flags
      Optional flags affecting the generated graph. Flags can be used to
      designate a link as static, to require that it not be used as a terminal
      body in the graph (because it is effectively massless), or to require
      that the resulting body is a base body (that is, connected directly to
      World).
  @param[in]      user_ref
      A generic user reference pointer that is kept with the associated
      link and can be used by the caller to map back to their own data
      structure containing additional link information.
  @see DeleteLink() */
  void AddLink(
      const std::string& link_name, ModelInstanceIndex model_instance,
      MultibodyGraph::LinkFlags flags = MultibodyGraph::kDefaultLinkFlags,
      void* user_ref = nullptr);

  /** Convenience overload that puts the link into the default model
  instance. */
  void AddLink(
      const std::string& name,
      MultibodyGraph::LinkFlags flags = MultibodyGraph::kDefaultLinkFlags,
      void* user_ref = nullptr) {
    AddLink(name, default_model_instance(), flags, user_ref);
  }

  /** Delete a link from the set of input links. All the joints that
  reference this link will be deleted too. Link numbers and joint numbers will
  have changed after this call.
  @param[in]      name
      A unique string identifying this link. There are no other restrictions
      on the contents of `name`. Don't delete the World link.
  @returns `true` if the link is successfully deleted, `false` if it
      didn't exist. */
  bool DeleteLink(const std::string& name);

  /** Add a new joint to the set of input joints. Each of these will produce
  a mobilizer or constraint in the generated graph.
  @param[in]      name
      A string uniquely identifying this joint within its model instance. There
      are no other restrictions on the contents of `name`.
  @param[in]      model_instance
      The model instance to which this joint belongs. If none is specified then
      the joint belongs to the default model instance. This is simply a tag that
      is carried with the joint and mobilizers generated from it; it has no
      effect on the structure of the generated graph. However, it does permit
      queries that return only elements of a particular model instance.
  @param[in]      type
      A string designating the type of this joint, such as "revolute" or
      "ball". This must be chosen from the set of joint types previously
      registered.
  @param[in]      parent_link_name
      This must be the name of a link that was already specified in an earlier
      AddLink() call, or it must be the designated name for the World body.
      If possible, this will be used as the inboard body for the corresponding
      mobilizer.
  @param[in]      child_link_name
      This must be the name of a link that was already specified in an earlier
      AddLink() call, or it must be the designated name for the World body.
      It must be distinct from `parent_link_name`. If possible, this will be
      used as the outboard body for the corresponding mobilizer.
  @param[in]      flags
      If you feel strongly that this joint should be implemented using
      a constraint rather than a mobilizer (not common), provide the flag
      MultibodyGraph::kMustBeConstraint. In that case the joint
      will not appear in the list of joints that are candidates to be modeled
      as mobilizers in a multibody tree. Only after the tree has been
      successfully built will this joint be added, either using a constraint
      equivalent if one is available, or by splitting one of its connected links
      into master and slave bodies otherwise. In the latter case this joint
      _will_ be made into a mobilizer but a weld constraint will be added
      to attach the slave body to its master.
  @param[in]      user_ref
      This is a generic user reference pointer that is kept with the joint
      and can be used by the caller to map back to their own data
      structure containing additional joint information.
  @see DeleteJoint() */
  void AddJoint(
      const std::string& name, ModelInstanceIndex model_instance,
      const std::string& type, const std::string& parent_link_name,
      const std::string& child_link_name,
      MultibodyGraph::JointFlags flags = MultibodyGraph::kDefaultJointFlags,
      void* user_ref = nullptr);

  /** Convenience overload that puts the joint into the default model
  instance. */
  void AddJoint(
      const std::string& name, const std::string& type,
      const std::string& parent_link_name, const std::string& child_link_name,
      MultibodyGraph::JointFlags flags = MultibodyGraph::kDefaultJointFlags,
      void* user_ref = nullptr) {
    AddJoint(name, {}, type, parent_link_name, child_link_name, flags,
             user_ref);
  }

  /** Delete an existing joint from the set of input joints. The links
  referenced by the joint are expected to exist and their references to this
  joint will be removed as well. Joint numbers will have changed after this
  call.
  @param[in]      name
      A string uniquely identifying this joint. There are no other
      restrictions on the contents of `name`.
  @param[in]      model_instance
      The model instance to which this joint belongs. If none is specified then
      the joint belongs to the default model instance.
  @returns `true` if the joint is successfully deleted, `false` if it
      didn't exist. */
  bool DeleteJoint(const std::string& name,
                   ModelInstanceIndex model_instance = {});

  /** (Advanced) Change the flags for an existing link. May cause a different
  graph to be generated on the next call to GenerateGraph(). This method
  exists primarily for testing purposes; normally the mass provided when the
  link is added remains unchanged. */
  void ChangeLinkFlags(const std::string& name,
                       ModelInstanceIndex model_instance,
                       MultibodyGraph::LinkFlags flags);

  /** Returns the name we recognize as the World (or Ground) link. This is
  the name that was provided in the first AddLink() call. */
  const std::string& world_link_name() const;

  /** Returns the link number for the World link (always zero). */
  LinkNum world_link_num() const { return LinkNum(0); }

  /** For debugging, write a human-readable representation of the input graph
  to the given output stream. */
  void DumpInput(std::ostream& output_stream) const;
  //@}

  /** Generate a new multibody graph from the input data. Throws an
  std::runtime_error if it fails, with a message in the what() string. */
  std::unique_ptr<MultibodyGraph> MakeGraph();

  /** @name             Inspect the links and joints
  Methods in this section allow introspection of the input objects. */
  //@{

  /** Returns the number of links, including all input links, and a World
  link. */
  int num_links() const { return static_cast<int>(links_.size()); }

  /** Gets a Link object by its assigned number. These are assigned first to
  World (link 0) and then input links. */
  const Link& get_link(LinkNum link_num) const {
    DRAKE_ASSERT(link_num.is_valid() && link_num < num_links());
    return links_[link_num];
  }

  /** Returns the link number assigned to the input link with the given name.
  The name must be globally unique if no model instance is given, otherwise it
  must be present in the given model instance. Throws std::logic_error if
  the name is not recognized or not unique when required to be. */
  LinkNum FindLinkNum(
      const std::string& link_name,
      optional<ModelInstanceIndex> model_instance = nullopt) const {
    return model_instance ? GetLinkNumFromNameAndInstance(
                                link_name, *model_instance, __func__)
                          : GetUniqueLinkNumFromName(link_name, __func__);
  }

  /** Returns the number of input joints. Don't confuse
  these with mobilizers, which are a possible implementation of joints
  chosen to form a spanning tree connecting all the bodies. */
  int num_joints() const { return static_cast<int>(joints_.size()); }

  /** Gets a Joint object by its assigned number. */
  const Joint& get_joint(JointNum joint_num) const {
    DRAKE_ASSERT(joint_num.is_valid() && joint_num < num_joints());
    return joints_[joint_num];
  }

  /** Returns the joint number assigned to the input joint with the given name.
  The name must be globally unique if no model instance is given, otherwise it
  must be present in the given model instance. Throws std::logic_error if
  the name is not recognized or not unique when required to be. */
  JointNum FindJointNum(
      const std::string& joint_name,
      optional<ModelInstanceIndex> model_instance = nullopt) const {
    return model_instance ? GetJointNumFromNameAndInstance(
                                joint_name, *model_instance, __func__)
                          : GetUniqueJointNumFromName(joint_name, __func__);
  }
  //@}

  /** @name         Specify and inspect available joint types
  Use the methods in this section to inform %MultibodyGraphMaker about the
  available joint types in your multibody code. You can also inspect what has
  been defined already. Some joint types are predefined if you accept the
  Drake defaults at construction. */
  //@{
  /** Specify relevant properties of a joint type as implemented in your
  multibody code. The joint type name must be unique. Weld and Free types are
  predefined and their names are reserved (though you can change the names to be
  used).
  @param[in]      name
      A unique string identifying a joint type, such as "pin" or "prismatic".
  @param[in]      num_q
      The number of generalized coordinates (positions) for this joint type.
      Must be greater than or equal to `num_v`.
  @param[in]      num_v
      The number of generalized velocities (degrees of freedom) for this joint
      type. Must be less than or equal to `num_q`.
  @param[in]      flags
      You can set this to `MultibodyGraph::kOkToUseAsJointConstraint` which
      mean that, in addition to a mobilizer of this type, you have
      available a constraint-based joint that is just as good. If so
      %MultibodyGraphMaker will propose using that constraint rather than
      cutting a link to break a loop. Typically used only for ball (spherical)
      joints.
  @param[in]      user_ref
      This is a generic user reference pointer that is kept with the joint type
      and can be used by the caller to map back to their own data
      structure containing more joint type information.
  @retval JointTypeNum Small integer joint type number used for referencing. */
  JointTypeNum RegisterJointType(const std::string& name, int num_q, int num_v,
                                 MultibodyGraph::JointTypeFlags flags =
                                     MultibodyGraph::kDefaultJointTypeFlags,
                                 void* user_ref = nullptr);

  /** Change the name to be used to identify the weld joint type (0 dof) and
  weld loop constraint type (6 constraints). The default is "weld". Changing
  this name clears and reinitializes this %MultibodyGraphMaker object. */
  void SetWeldJointTypeName(const std::string& name) {
    weld_type_name_ = name;
  }

  /** Changes the name to be used to identify the free (6 dof) joint type and
  free (0 constraints) loop constraint type. The default is "free". Changing
  this name clears and reinitializes this %MultibodyGraphMaker object. */
  void SetFreeJointTypeName(const std::string& name) {
    free_type_name_ = name;
  }

  /** Returns the number of registered joint types. */
  int num_joint_types() const { return static_cast<int>(joint_types_.size()); }

  /** Get a JointType object by its assigned number. */
  const JointType& get_joint_type(JointTypeNum joint_type_num) const {
    DRAKE_ASSERT(joint_type_num.is_valid() &&
                 joint_type_num < num_joint_types());
    return joint_types_[joint_type_num];
  }

  /** Finds the assigned number for a joint type from the type name. */
  JointTypeNum FindJointTypeNum(const std::string& joint_type_name) const {
    std::map<std::string, JointTypeNum>::const_iterator p =
        joint_type_name_to_num_.find(joint_type_name);
    return p == joint_type_name_to_num_.end() ? JointTypeNum() : p->second;
  }

  /** Return the name currently being used to identify the weld joint type
  and weld loop constraint type. */
  const std::string& get_weld_joint_type_name() const {
    return weld_type_name_;
  }

  /** Returns the name currently being used to identify the free joint type
  and free loop constraint type. */
  const std::string& get_free_joint_type_name() const {
    return free_type_name_;
  }
  //@}

 private:
  // Get writable access to links and joints.
  Link& get_mutable_link(LinkNum link_num) { return links_[link_num]; }
  Joint& get_mutable_joint(JointNum joint_num) { return joints_[joint_num]; }

  // Returns the model_instance->link_num map for all links with this name,
  // if any, otherwise nullptr.
  const std::map<ModelInstanceIndex, LinkNum>* GetAllLinkNumsFromName(
      const std::string& link_name) const;

  // Returns the link_num for the a link name in a model instance.
  LinkNum GetLinkNumFromNameAndInstance(const std::string& link_name,
                                        ModelInstanceIndex model_instance,
                                        const char* func) const;

  // Returns the link_num for this link name, which must appear in only one
  // model instance.
  LinkNum GetUniqueLinkNumFromName(const std::string& name,
                                   const char* func) const;

  // Returns the model_instance->joint_num map for all joints with this name,
  // if any, otherwise nullptr.
  const std::map<ModelInstanceIndex, JointNum>* GetAllJointNumsFromName(
      const std::string& joint_name) const;

  // Returns the joint_num for the a joint name in a model instance.
  JointNum GetJointNumFromNameAndInstance(const std::string& joint_name,
      ModelInstanceIndex model_instance, const char* func) const;

  // Returns the joint_num for this joint name, which must appear in only one
  // model instance.
  JointNum GetUniqueJointNumFromName(const std::string& joint_name,
                                     const char* func) const;

  LinkNum ChooseNewBaseLink(const MultibodyGraph& graph) const;
  MultibodyGraph::MobilizerNum AddMobilizerForJoint(JointNum joint_num,
                                                    MultibodyGraph* graph);
  JointNum FindHeaviestUnassignedForwardJoint(
      LinkNum parent_link_num, const MultibodyGraph& graph) const;
  JointNum FindHeaviestUnassignedReverseJoint(
      LinkNum parent_link_num, const MultibodyGraph& graph) const;
  void GrowTree(int start_level, MultibodyGraph* graph);
  void BreakLoops(MultibodyGraph* graph);
  bool LinksAreConnected(LinkNum link1_num, LinkNum link2_num) const;

  // Clear everything except for default names.
  void ClearContainers() {
    joint_types_.clear();
    joint_type_name_to_num_.clear();
    links_.clear();
    link_name_to_num_.clear();
    joints_.clear();
    joint_name_to_num_.clear();
  }

  std::string weld_type_name_, free_type_name_;
  std::vector<JointType> joint_types_;
  std::map<std::string, JointTypeNum> joint_type_name_to_num_;

  std::vector<Link> links_;   // world + input bodies
  std::map<std::string, std::map<ModelInstanceIndex, LinkNum>>
      link_name_to_num_;

  std::vector<Joint> joints_;  // input joints
  std::map<std::string, std::map<ModelInstanceIndex, JointNum>>
      joint_name_to_num_;
};

//------------------------------------------------------------------------------
//                      MULTIBODY GRAPH MAKER :: LINK
//------------------------------------------------------------------------------
/** Local class that collects information about links in the input. */
class MultibodyGraphMaker::Link {
 public:
  /** Returns the number of joints that specify this link as their parent
  link. This refers to the input parent/child designation _not_ necessarily
  the inboard/outboard relationship in the generated graph. */
  int num_children() const {
    return static_cast<int>(joints_as_parent_.size());
  }

  /** Returns the number of joints that specify this link as their child
  link. This refers to the input parent/child designation _not_ necessarily
  the inboard/outboard relationship in the generated graph. */
  int num_parents() const { return static_cast<int>(joints_as_child_.size()); }

  /** Returns the total number of joints that specify this body as either
  parent or child (this is num_children() + num_parents()). */
  int num_joints() const { return num_children() + num_parents(); }

  const std::string& name() const { return name_; }
  ModelInstanceIndex model_instance() const { return model_instance_; }
  MultibodyGraph::LinkFlags flags() const { return flags_; }
  bool must_be_nonterminal() const {
    return flags_ & MultibodyGraph::kMustNotBeTerminalBody;
  }
  bool is_static() const { return flags_ & MultibodyGraph::kStaticLink; }
  bool must_be_base_body() const {
    return flags_ & MultibodyGraph::kMustBeBaseBody;
  }
  void* user_ref() const { return user_ref_; }

  /** Returns the list of joints (by joint number) for which this link is
  the child link. */
  const std::vector<JointNum>& joints_as_child() const {
    return joints_as_child_;
  }

  /** Returns the list of joints (by joint number) for which this link is
  the parent link. */
  const std::vector<JointNum>& joints_as_parent() const {
    return joints_as_parent_;
  }

  /** Changes the flags for this link. */
  void set_flags(MultibodyGraph::LinkFlags flags) {
    flags_ = flags;
  }

 private:
  // Restrict construction and modification to only MultibodyGraphMaker.
  friend class MultibodyGraphMaker;

  Link(const std::string& name, ModelInstanceIndex model_instance,
       MultibodyGraph::LinkFlags flags, void* user_ref)
      : name_(name),
        model_instance_(model_instance),
        flags_(flags),
        user_ref_(user_ref) {}

  // Notes that this link serves as the parent link for the given joint.
  void add_joint_as_parent(JointNum joint_num) {
    joints_as_parent_.push_back(joint_num);
  }

  // Notes that this link serves as the child link for the given joint.
  void add_joint_as_child(JointNum joint_num) {
    joints_as_child_.push_back(joint_num);
  }

  // Returns mutable access to the list of joints for which this link is
  // the child link.
  std::vector<JointNum>& mutable_joints_as_child() { return joints_as_child_; }

  // Returns mutable access to the list of joints for which this link is
  // the parent link.
  std::vector<JointNum>& mutable_joints_as_parent() {
    return joints_as_parent_;
  }

  // Inputs
  std::string name_;
  ModelInstanceIndex model_instance_;
  MultibodyGraph::LinkFlags flags_{MultibodyGraph::kDefaultLinkFlags};
  void* user_ref_;

  // How this link appears in joints (input and added).
  std::vector<JointNum> joints_as_child_;   // where this link is the child
  std::vector<JointNum> joints_as_parent_;  // where this link is the parent
};

//------------------------------------------------------------------------------
//                      MULTIBODY GRAPH MAKER :: JOINT
//------------------------------------------------------------------------------
/** Local class that collects information about input joints. */
class MultibodyGraphMaker::Joint {
 public:
  const std::string& name() const { return name_; }
  ModelInstanceIndex model_instance() const { return model_instance_; }
  JointFlags flags() const { return flags_; }
  bool must_be_constraint() const {
    return flags_ & MultibodyGraph::kMustBeConstraint;
  }
  void* user_ref() const { return user_ref_; }

  LinkNum parent_link_num() const { return parent_link_num_; }
  LinkNum child_link_num() const { return child_link_num_; }
  JointTypeNum joint_type_num() const { return joint_type_num_; }

  JointNum joint_num() const { return joint_num_; }

 private:
  // Restrict construction and modification to only MultibodyGraphMaker.
  friend class MultibodyGraphMaker;

  Joint(const std::string& name, ModelInstanceIndex model_instance,
        JointTypeNum joint_type_num, LinkNum parent_link_num,
        LinkNum child_link_num, MultibodyGraph::JointFlags flags,
        void* user_ref)
      : name_(name),
        model_instance_(model_instance),
        joint_type_num_(joint_type_num),
        flags_(flags),
        user_ref_(user_ref),
        parent_link_num_(parent_link_num),
        child_link_num_(child_link_num) {}

  void set_parent_link_num(LinkNum parent_link_num) {
    parent_link_num_ = parent_link_num;
  }
  void set_child_link_num(LinkNum child_link_num) {
    child_link_num_ = child_link_num;
  }

  // Inputs
  std::string name_;
  ModelInstanceIndex model_instance_;
  JointTypeNum joint_type_num_;
  MultibodyGraph::JointFlags flags_;
  void* user_ref_{nullptr};

  LinkNum parent_link_num_;
  LinkNum child_link_num_;

  // Assigned index.
  JointNum joint_num_;
};

//------------------------------------------------------------------------------
//                   MULTIBODY GRAPH MAKER :: JOINT TYPE
//------------------------------------------------------------------------------
/** Local class that defines the properties of a known joint type. */
class MultibodyGraphMaker::JointType {
 public:
  const std::string& name() const { return name_; }
  int num_q() const { return num_q_; }
  int num_v() const { return num_v_; }
  bool have_good_joint_constraint_available() const {
    return flags_ & MultibodyGraph::kOkToUseAsJointConstraint;
  }
  void* user_ref() const { return user_ref_; }

  JointTypeNum joint_type_num() const {
    return joint_type_num_;
  }

 private:
  // Restrict construction to only MultibodyGraphMaker.
  friend class MultibodyGraphMaker;

  JointType(const std::string& name, int num_q, int num_v,
            MultibodyGraph::JointTypeFlags flags,
            void* user_ref, JointTypeNum joint_type_num)
      : name_(name),
        num_q_(num_q),
        num_v_(num_v),
        flags_(flags),
        user_ref_(user_ref),
        joint_type_num_(joint_type_num) {}

  std::string name_;
  int num_q_{-1};
  int num_v_{-1};
  MultibodyGraph::JointTypeFlags flags_{MultibodyGraph::kDefaultJointTypeFlags};
  void* user_ref_{nullptr};

  // Assigned index.
  JointTypeNum joint_type_num_;
};

}  // namespace multibody
}  // namespace drake
