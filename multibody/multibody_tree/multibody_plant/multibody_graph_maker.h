#pragma once

/* Adapted for Drake from Simbody's MultibodyGraphMaker class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

/** @file
Declares the MultibodyGraphMaker class for use in constructing a
spanning-tree-plus-constraints representation of a multibody system from a
list of its links and joints. */

#include <iosfwd>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

//==============================================================================
//                          MULTIBODY GRAPH MAKER
//==============================================================================
/** Construct a reasonably good spanning-tree-plus-constraints structure for
modeling a given set of links (bodies) and joints with a generalized coordinate
multibody system like Drake or Simbody. The idea is to map a directed graph
of _links_ and _joints_, that is possibly cyclic and disconnected, to an output
graph consisting of a fully-connected tree of _bodies_ and _mobilizers_, plus a
set of constraints to enforce loop connectivity. This is particularly useful
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
  maker.AddLink("link1", 1.);  // name and mass
  maker.AddLink("link2", 1.);
  maker.AddLink("link3", 1.);
  maker.AddLink("link4", 1.);
  maker.AddLink("link5", 1.);
  maker.AddJoint("joint0", "revolute", "link2", "link1");  // parent, child
  maker.AddJoint("joint1", "revolute", "link3", "link2");
  maker.AddJoint("joint2", "revolute", "link3", "link4");
  maker.AddJoint("joint3", "revolute", "link4", "link5");
  maker.GenerateGraph();

  for (int i=0; i < maker.num_mobilizers(); ++i) {
    auto& mobilizer = maker.get_mobilizer(i);
    auto& inb = maker.get_body(mobilizer.inboard_body_num());
    auto& outb = maker.get_body(mobilizer.outboard_body_num());
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
of the mobilizer loop above would _build_ the multibody system rather than
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
  - choose sensible base bodies for trees that don't have a connection to World
    provided in the input,
  - make the mobilizer inboard/output ordering follow the parent/child ordering
    in the input,
  - cut loops such that branch lengths are balanced (minimizes the maximum
    length of any branch),
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
mobilized bodies designated as slaves to particular master input links. That
is, more than one mobilized body may correspond to the same input link.
  -# A correspondence between mobilizers and input joints, with some extra
mobilizers having been added to connect base bodies to World.
  -# Statistics, diagnostics, and inspection of the resulting trees.

Then to build a multibody model
  -# Run through the generated mobilizers in order, adding one mobilized body to
the tree each time, using "reversed" mobilizers if appropriate (that just
affects the interpretation of the generalized coordinates). Mass properties and
joint placement information are obtained from the original bodies and joints;
they are not stored here.
  -# Run through the list of constraints adding master-slave welds or loop
joint constraints as indicated.

<h3>Inputs</h3>
  - links: name, mass, must_be_base_body flag
  - joints: name, type, parent link, child link, must_be_loop_joint flag
  - joint type: name, # dofs, have_good_loop_joint flag
  - names for the World body and important joint types weld and free.

The first body added (body 0) is assumed to be World and its name is used for
recognizing World connections later in joints. If you accept the Drake defauls,
that body will be named "world". The default names for Weld (0 dof) and Free
(6 dof) joints are, not surprisingly, "weld" and "free" but you can change them.

<h3>Loop joints</h3>
Normally every joint produces a corresponding mobilizer. A joint that would
form a loop is marked as such, and then its child link is split to form a new
"slave" body that can be mobilized by the joint. We expect that in the
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

  // Local classes.
  class Body;
  class Joint;
  class JointType;
  class Mobilizer;
  class LoopConstraint;

  /** Construct an empty %MultibodyGraphMaker object and set the default
  names for Weld and Free joints to "weld" and "free". Also sets up Drake's
  World body (named "world") and supported joint types, unless
  `use_drake_defaults` is set `false`. In that case the first added link (body
  number 0) is the World body and its name is used to represent World. */
  explicit MultibodyGraphMaker(bool use_drake_defaults = true);

  /** Restores this %MultibodyGraphMaker object to its just-constructed
  condition. See the constructor documentation for the meaning of the
  parameter. */
  void Clear(bool use_drake_defaults = true);

  /** @name         Define the input graph (links and joints)
  Methods in this section are used to inform %MultibodyGraphMaker about the
  links (bodies) and joints as given, typically in an .sdf or .urdf file. You
  can also inspect the set of links and joints, and remove links and joints
  from an existing specification. */
  //@{

  /** Add a new link (bodies) to the set of input links. These are immediately
  stored as Body objects since there is always at least one body corresponding
  to an input link. If a graph has already been generated, calling this method
  clears the graph.
  @param[in]      name
      A unique string identifying this link. There are no other restrictions
      on the contents of `name`. If this %MultibodyGraphMaker was constructed
      without Drake defaults, the first link you add is considered to be
      World, and its name is remembered and recognized when used in joints.
  @param[in]      mass
      The mass here is used as a graph-building hint. If the link is massless,
      or has a very small mass compared to other links, set mass=0
      so that the algorithm can avoid making this a terminal body in the
      generated graph. The algorithm might also use `mass` to preferentially
      choose heavier bodies as terminal to improve conditioning.
  @param[in]      must_be_base_body
      If you feel strongly that this link should be able to move freely with
      respect to World, set this flag so that the algorithm will connect it
      to World by a Free joint before attempting to build the rest of the
      tree. Alternatively, provide a joint that connects this link directly
      to World, in which case you should _not_ set this flag.
  @param[in]      user_ref
      This is a generic user reference pointer that is kept with the associated
      body and can be used by the caller to map back to their own data
      structure containing additional link information.
  @see DeleteLink() */
  void AddLink(const std::string& name, double mass,
               bool must_be_base_body = false,
               void* user_ref = nullptr);

  /** Delete a link (body) from the set of input links. All the joints that
  reference this link will be deleted too. The generated graph, if any, is
  cleared.
  @param[in]      name
      A unique string identifying this link. There are no other restrictions
      on the contents of `name`. Don't delete the World link.
  @returns `true` if the link is successfully deleted, `false` if it
      didn't exist. */
  bool DeleteLink(const std::string& name);

  /** Add a new joint to the set of input joints. If a graph has already been
  generated, calling this method clears the graph.
  @param[in]      name
      A string uniquely identifying this joint. There are no other
      restrictions on the contents of `name`.
  @param[in]      type
      A string designating the type of this joint, such as "revolute" or
      "ball". This must be chosen from the set of joint types previously
      registered.
  @param[in]      parent_body_name
      This must be the name of a link that was already specified in an earlier
      AddLink() call, or it must be the designated name for the World body.
      If possible, this will be used as the inboard body for the corresponding
      mobilizer.
  @param[in]      child_body_name
      This must be the name of a link that was already specified in an earlier
      AddLink() call, or it must be the designated name for the World body.
      It must be distinct from `parent_body_name`. If possible, this will be
      used as the outboard body for the corresponding mobilizer.
  @param[in]      must_be_loop_joint
      If you feel strongly that this joint should be chosen as a loop joint,
      set this flag. In that case the joint will not appear in the list of
      joints that are candidates for mobilizers (tree joints). Only after the
      tree has been successfully built will this joint be added, either using
      a loop joint equivalent if one is available, or by splitting the child
      link into master and slave bodies otherwise. In the latter case this joint
      _will_ be made into a mobilizer but a loop weld constraint will be added
      to attach the child slave body to its master.
  @param[in]      user_ref
      This is a generic user reference pointer that is kept with the joint
      and can be used by the caller to map back to their own data
      structure containing additional joint information.
  @see DeleteJoint() */
  void AddJoint(const std::string& name, const std::string& type,
                const std::string& parent_body_name,
                const std::string& child_body_name,
                bool must_be_loop_joint = false,
                void* user_ref = nullptr);

  /** Delete an existing joint from the set of input joints. The links
  referenced by the joint are expected to exist and their references to this
  joint will be removed as well. The generated graph, if any, is cleared.
  @param[in]      name
      A string uniquely identifying this joint. There are no other
      restrictions on the contents of `name`.
  @returns `true` if the joint is successfully deleted, `false` if it
      didn't exist. */
  bool DeleteJoint(const std::string& name);

  /** (Advanced) Change the mass of an existing link. If it changes from
  massful to massless or vice versa this may cause the generated graph to
  change. In any case this method will clear the generated graph. This method
  exists primarily for testing purposes; normally the mass provided when the
  link is added remains unchanged. */
  void ChangeLinkMass(const std::string& name, double new_mass);

  /** Returns the name we recognize as the World (or Ground) body. This is
  the name that was provided in the first AddLink() call. */
  const std::string& get_world_body_name() const;
  //@}

  /** @name               Work with the generated graph
  Methods in this section manipulate the generated multibody graph consisting of
  bodies (input links and additional slaves), mobilizers (input joints plus
  additional connections to World), and loop constraints. */
  //@{

  /** Generate a new multibody graph from the input data. Throws an
  std::runtime_error if it fails, with a message in the what() string.
  Throws std::logic_error if the graph has already been generated since the last
  change or call to ClearGraph(). */
  void GenerateGraph();

  /** Throw away the multibody graph produced by GenerateGraph(). Does nothing
  if the graph hasn't been generated since the last modification. */
  void ClearGraph();

  /** Output a text representation of the multibody graph for debugging. */
  void DumpGraph(std::ostream& out) const;

  /** Returns `true` if GenerateGraph() has been called since the last change
  to the input or last call to ClearGraph(). */
  bool graph_has_been_generated() const { return graph_generated_; }

  /** Returns the number of mobilizers (tree joints) in the spanning tree.
  These are numbered in order of level, such that branches are built from
  World outboard. The 0th mobilizer has level 0 and
  is just a placeholder for World's immobile connection to the universe.
  After that come the base mobilizers at level 1, then mobilizers that
  connect children to base bodies at level 2, and so on. This is also the
  number of mobilized bodies, including World and slave bodies. */
  int num_mobilizers() const { return static_cast<int>(mobilizers_.size()); }

  /** Gets a Mobilizer object by its mobilizer number, ordered outwards by
  topological distance from World. */
  const Mobilizer& get_mobilizer(int mobilizer_num) const {
    return mobilizers_[mobilizer_num];
  }

  /** Returns the number of loop joint constraints that were used to close
  loops in the graph topology. These include loops that were broken
  by cutting a body to make a slave body, and those where the joint itself
  was implemented using a constraint rather than a mobilizer plus a slave.
  The latter occurs only if were told there is a perfectly good loop joint
  constraint available; typically that applies for ball joints and not much
  else. */
  int num_loop_constraints() const {
    return static_cast<int>(constraints_.size());
  }

  /** Gets a loop constraint by its assigned number. These are assigned in
  an arbitrary order. */
  const LoopConstraint& get_loop_constraint(int loop_constraint_num) const {
    return constraints_[loop_constraint_num];
  }

  /** Returns the number of bodies, including all input bodies, a World body,
  and any slave bodies. */
  int num_bodies() const { return static_cast<int>(bodies_.size()); }

  /** Gets a Body object by its assigned number. These are assigned first to
  World (body 0), then input links, then we add slave bodies created by body
  splitting after that. */
  const Body& get_body(int body_num) const { return bodies_[body_num]; }

  /** Returns the body number assigned to the input body with the given name.
  Returns -1 if the body name is not recognized. You can't look up by name
  slave bodies that were added by the graph-making algorithm. */
  int FindBodyNum(const std::string& body_name) const {
    std::map<std::string, int>::const_iterator p =
        body_name_to_num_.find(body_name);
    return p == body_name_to_num_.end() ? -1 : p->second;
  }

  /** Returns the number of joints, including all input joints, and all joints
  added to connect otherwise disconnected bodies to World. Don't confuse
  these with mobilizers, which are an ordered subset of the joints that are
  chosen to form a spanning tree connecting all the bodies. */
  int num_joints() const { return static_cast<int>(joints_.size()); }

  /** Gets a Joint object by its assigned number. These are assigned first to
  input joints, then we add additional joints to World as needed. */
  const Joint& get_joint(int jointNum) const { return joints_[jointNum]; }

  /** Returns the joint number assigned to the input joint with the given name.
  Returns -1 if the joint name is not recognized. You can't look up by name
  extra joints that were added by the graph-making algorithm. */
  int FindJointNum(const std::string& joint_name) const {
    std::map<std::string, int>::const_iterator p =
        joint_name_to_num_.find(joint_name);
    return p == joint_name_to_num_.end() ? -1 : p->second;
  }

  /** Returns the set of base bodies (bodies connected directly to World) in
  the generated multibody graph. These are ordered by increasing mobilizer
  number of their base mobilizers. */
  std::vector<int> FindBaseBodies() const;

  /** Returns the base body of the subtree containing the given `body_num`.
  If `body_num` is a base body it is returned. If `body_num` is World we
  return -1. */
  int FindBaseBody(int body_num) const;

  /** Returns a list of bodies, starting with `body_num` and ending with the
  base body that connects that body to World through inboard mobilizers.
  Will have just one element if `body_num` is a base body, and be empty if
  `body_num` is the World body. */
  std::vector<int> FindPathToWorld(int body_num) const;
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
  @param[in]      num_mobilities
      The number of degrees of freedom for this joint type. This is the number
      of generalized _velocities_ v, not necessarily the same as the number of
      generalized _positions_ q.
  @param[in]      have_good_loop_joint_available
      Set this `true` if, in addition to a mobilizer of this type, you have
      available a constraint-based loop joint that is just as good. If so
      %MultibodyGraphMaker will propose using that loop joint rather than
      cutting a body to break a loop. Typically used only for ball joints.
  @param[in]      user_ref
      This is a generic user reference pointer that is kept with the joint type
      and can be used by the caller to map back to their own data
      structure containing more joint type information.
  @returns Small integer joint type number used for referencing. */
  int RegisterJointType(const std::string& name, int num_mobilities,
                        bool have_good_loop_joint_available = false,
                        void* user_ref = nullptr);

  /** Change the name to be used to identify the weld joint type (0 dof) and
  weld loop constraint type (6 constraints). The default is "weld". Changing
  this name clears and reinitializes this %MultibodyGraphMaker object. */
  void SetWeldJointTypeName(const std::string& name) {
    weld_type_name_ = name;
    Clear();
  }

  /** Changes the name to be used to identify the free (6 dof) joint type and
  free (0 constraints) loop constraint type. The default is "free". Changing
  this name clears and reinitializes this %MultibodyGraphMaker object. */
  void SetFreeJointTypeName(const std::string& name) {
    free_type_name_ = name;
    Clear();
  }

  /** Returns the number of registered joint types. */
  int num_joint_types() const { return static_cast<int>(joint_types_.size()); }

  /** Get a JointType object by its assigned number. */
  const JointType& get_joint_type(int joint_type_num) const {
    return joint_types_[joint_type_num];
  }

  /** Gets the assigned number for a joint type from the type name. */
  int GetJointTypeNum(const std::string& joint_type_name) const {
    std::map<std::string, int>::const_iterator p =
        joint_type_name_to_num_.find(joint_type_name);
    return p == joint_type_name_to_num_.end() ? -1 : p->second;
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
  // Get writable access to bodies and joints.
  Body& get_mutable_body(int bodyNum) { return bodies_[bodyNum]; }
  Joint& get_mutable_joint(int jointNum) { return joints_[jointNum]; }
  Joint& GetMutableJointByName(const std::string& name) {
    return joints_[joint_name_to_num_[name]];
  }

  int SplitBody(int master_body_num);
  int ChooseNewBaseBody() const;
  void ConnectBodyToWorld(int body_num);
  int AddMobilizerForJoint(int jointNum);
  int FindHeaviestUnassignedForwardJoint(int inboard_body_num) const;
  int FindHeaviestUnassignedReverseJoint(int inboard_body_num) const;
  void GrowTree();
  void BreakLoops();
  bool BodiesAreConnected(int body1_num, int body2_num) const;

  // Clear everything except for default names.
  void ClearContainers() {
    bodies_.clear();
    joints_.clear();
    joint_types_.clear();
    body_name_to_num_.clear();
    joint_name_to_num_.clear();
    joint_type_name_to_num_.clear();
    mobilizers_.clear();
    constraints_.clear();
  }

  std::string weld_type_name_, free_type_name_;
  std::vector<Body> bodies_;   // world + input bodies + slaves
  std::vector<Joint> joints_;  // input joints + added joints
  std::vector<JointType> joint_types_;
  std::map<std::string, int> body_name_to_num_;
  std::map<std::string, int> joint_name_to_num_;
  std::map<std::string, int> joint_type_name_to_num_;

  // Calculated by GenerateGraph(), which also adds bodies and joints.
  std::vector<Mobilizer> mobilizers_;  // mobilized bodies
  std::vector<LoopConstraint> constraints_;  // master/slave welds, loop joints
  bool graph_generated_{false};
};

//------------------------------------------------------------------------------
//                      MULTIBODY GRAPH MAKER :: BODY
//------------------------------------------------------------------------------
/** Local class that collects information about links in the input and then
accumulates modeling information and represents bodies as the multibody tree is
generated. */
class MultibodyGraphMaker::Body {
 public:
  /** Returns the number of fragments into which we had to break this body.
  Normally this is just one, meaning we didn't break the body, but master bodies
  that have slaves will return the number of slaves plus one. */
  int num_fragments() const { return 1 + num_slaves(); }

  /** Returns the number of slave bodies associated with this master body.
  Normally zero, but a master body with (for example) three slave bodies would
  return three. */
  int num_slaves() const { return static_cast<int>(slaves_.size()); }

  /** Returns the number of joints that specify this body as their parent
  link. This refers to the input parent/child designation _not_ necessarily
  the inboard/outboard relationship in the generated graph. */
  int num_children() const {
    return static_cast<int>(joints_as_parent_.size());
  }

  /** Returns the number of joints that specify this body as their child
  link. This refers to the input parent/child designation _not_ necessarily
  the inboard/outboard relationship in the generated graph. */
  int num_parents() const { return static_cast<int>(joints_as_child_.size()); }

  /** Returns the total number of joints that specify this body as either
  parent or child (this is num_children() + num_parents()). */
  int num_joints() const { return num_children() + num_parents(); }

  /** Returns `true` if this body is a slave body generated by breaking an
  input link into multiple bodies. If so you can find its master using
  master_body_num(). */
  bool is_slave() const { return master_body_num_ >= 0; }

  /** Returns `true` if this is a body that represents a link that had to
  be broken into multiple bodies. If so you can find the generated slave
  bodies using slaves(). */
  bool is_master() const { return num_slaves() > 0; }

  /** Returns `true` if this is body that represents the World. */
  bool is_world_body() const {return level_ == 0; }

  const std::string& name() const { return name_; }
  double mass() const { return mass_; }
  bool must_be_base_body() const { return must_be_base_body_; }
  void* user_ref() const { return user_ref_; }

  /** Returns the level of this body in the multibody tree. If this is World
  it is level 0, if it's a base body it's level 1, if it's connected to a
  base body it's level 2, etc. The level of a body is the same as the level
  of its inboard mobilizer as obtained with mobilizer_num(). */
  int level() const { return level_; }

  /** Returns the unique mobilizer (by number) for which this body is the
  outboard body, or -1 if the multibody graph has not yet be generated. */
  int mobilizer_num() const { return mobilizer_num_; }

  /** Returns the list of joints (by joint number) for which this body is
  the child body. */
  const std::vector<int>& joints_as_child() const { return joints_as_child_; }

  /** Returns the list of joints (by joint number) for which this body is
  the parent body. */
  const std::vector<int>& joints_as_parent() const { return joints_as_parent_; }

  /** Returns the list of slave bodies (by body number), if this is a
  master body. */
  const std::vector<int>& slaves() const { return slaves_; }

  /** Returns the master body number (>=0) if this is a slave body, otherwise
  returns -1. */
  int master_body_num() const { return master_body_num_; }

  /** Changes the mass (>= 0) of this body. */
  void set_mass(double mass) { DRAKE_DEMAND(mass >= 0); mass_ = mass; }

 private:
  // Restrict construction and modification to only MultibodyGraphMaker.
  friend class MultibodyGraphMaker;

  Body(const std::string& name_in, double mass_in, bool must_be_base_body_in,
       void* user_ref_in)
      : name_(name_in),
        mass_(mass_in),
        must_be_base_body_(must_be_base_body_in),
        user_ref_(user_ref_in),
        level_(-1),
        mobilizer_num_(-1),
        master_body_num_(-1) {}

  void ForgetGraph(MultibodyGraphMaker* graph);

  bool is_in_tree() const { return level_ >= 0; }

  // Notes that this body serves as the parent body for the given joint.
  void add_joint_as_parent(int joint_num) {
    joints_as_parent_.push_back(joint_num);
  }

  // Notes that this body serves as the child body for the given joint.
  void add_joint_as_child(int joint_num) {
    joints_as_child_.push_back(joint_num);
  }

  void set_level(int level) { level_ = level; }
  void set_mobilizer_num(int mobilizer_num) { mobilizer_num_ = mobilizer_num; }

  // Records the master body number for this slave body.
  void set_master_body_num(int master_body_num) {
    master_body_num_ = master_body_num;
  }

  // Records a slave body number for this master body.
  void add_slave_body_num(int slave_body_num) {
    slaves_.push_back(slave_body_num);
  }

  // Returns mutable access to the list of joints for which this body is
  // the child body.
  std::vector<int>& mutable_joints_as_child() { return joints_as_child_; }

  // Returns mutable access to the list of joints for which this body is
  // the parent body.
  std::vector<int>& mutable_joints_as_parent() { return joints_as_parent_; }

  // Inputs
  std::string name_;
  double mass_;
  bool must_be_base_body_;
  void* user_ref_;

  // How this body appears in joints (input and added).
  std::vector<int> joints_as_child_;   // where this body is the child
  std::vector<int> joints_as_parent_;  // where this body is the parent

  // Disposition of this body in the spanning tree.

  int level_{-1};          // Distance from World in the tree.
  int mobilizer_num_{-1};  // The unique inboard mobilizer.

  int master_body_num_{-1};  // >=0 if this is a slave.
  std::vector<int> slaves_;  // Slave links, if this is a master.
};

//------------------------------------------------------------------------------
//                      MULTIBODY GRAPH MAKER :: JOINT
//------------------------------------------------------------------------------
/** Local class that collects information about joints. */
class MultibodyGraphMaker::Joint {
 public:
  // Only one of the next two will be true -- we don't consider it a
  // LoopConstraint if we split a body and weld it back.

  /** Returns `true` if this joint was implemented with a mobilizer. */
  bool has_mobilizer() const { return mobilizer_num_ >= 0; }

  /** Returns `true` if this joint was implemented with a loop joint. */
  bool has_loop_constraint() const { return loop_constraint_num_ >= 0; }

  /** Returns the associated mobilizer if this joint is implemented with
  a mobilizer, otherwise -1. */
  int mobilizer_num() const { return mobilizer_num_; }

  /** Returns the associated loop constraint if this joint is implemented with
  a loop joint, otherwise -1. */
  int loop_constraint_num() const { return loop_constraint_num_; }

  /** Returns `true` if this joint was a joint-to-world added by us rather than
  present in the input. */
  bool is_added_base_joint() const { return is_added_base_joint_; }

  const std::string& name() const { return name_; }
  bool must_be_loop_joint() const { return must_be_loop_joint_; }
  void* user_ref() const { return user_ref_; }

  int parent_body_num() const { return parent_body_num_; }
  int child_body_num() const { return child_body_num_; }
  int joint_type_num() const { return joint_type_num_; }

 private:
  // Restrict construction and modification to only MultibodyGraphMaker.
  friend class MultibodyGraphMaker;

  Joint(const std::string& name, int joint_type_num, int parent_body_num,
        int child_body_num, bool must_be_loop_joint, void* user_ref)
      : name_(name),
        joint_type_num_(joint_type_num),
        must_be_loop_joint_(must_be_loop_joint),
        user_ref_(user_ref),
        parent_body_num_(parent_body_num),
        child_body_num_(child_body_num) {}

  /** Return true if the joint is deleted as a result of restoring it
      to the state prior to generateGraph(). */
  bool ForgetGraph(MultibodyGraphMaker* graph);

  void set_mobilizer_num(int mobilizer_num) { mobilizer_num_ = mobilizer_num; }
  void set_loop_constraint_num(int loop_constraint_num) {
    loop_constraint_num_ = loop_constraint_num;
  }

  void set_is_added_base_joint(bool is_added_base_joint) {
    is_added_base_joint_ = is_added_base_joint;
  }

  void set_parent_body_num(int parent_body_num) {
    parent_body_num_ = parent_body_num;
  }
  void set_child_body_num(int child_body_num) {
    child_body_num_ = child_body_num;
  }

  // Inputs
  std::string name_;
  int joint_type_num_{-1};
  bool must_be_loop_joint_{false};
  void* user_ref_{nullptr};

  // Mapping of strings to indices for fast lookup. These may have to be
  // adjusted later due to deletions.
  int parent_body_num_{-1};
  int child_body_num_{-1};

  bool is_added_base_joint_{false};  // if this wasn't one of the input joints

  // Disposition of this joint in the multibody graph.
  int mobilizer_num_{-1};        // if joint is part of the spanning tree
  int loop_constraint_num_{-1};  // if joint used a loop constraint
};

//------------------------------------------------------------------------------
//                   MULTIBODY GRAPH MAKER :: JOINT TYPE
//------------------------------------------------------------------------------
/** Local struct that defines the properties of a known joint type. */
class MultibodyGraphMaker::JointType {
 public:
  const std::string& name() const { return name_; }
  int num_mobilities() const { return num_mobilities_; }
  bool have_good_loop_joint_available() const {
    return have_good_loop_joint_available_;
  }
  void* user_ref() const { return user_ref_; }

 private:
  // Restrict construction to only MultibodyGraphMaker.
  friend class MultibodyGraphMaker;

  JointType(const std::string& name, int num_mobilities,
            bool have_good_loop_joint_available, void* user_ref)
      : name_(name),
        num_mobilities_(num_mobilities),
        have_good_loop_joint_available_(have_good_loop_joint_available),
        user_ref_(user_ref) {}

  std::string name_;
  int num_mobilities_{-1};
  bool have_good_loop_joint_available_{false};
  void* user_ref_{nullptr};
};

//------------------------------------------------------------------------------
//                   MULTIBODY GRAPH MAKER :: MOBILIZER
//------------------------------------------------------------------------------
/** Local class that represents one of the mobilizers (tree joints) in the
generated spanning tree. There is always a corresponding joint, although that
joint might be a world-to-link free joint that was added automatically. */
class MultibodyGraphMaker::Mobilizer {
 public:
  /** Returns the joint associated with this mobilizer. There will always be
  one, but it may have been an added joint rather than an input joint. */
  int joint_num() const {return joint_num_;}

  /** Returns the inboard body number of this mobilizer. This is the parent
  body of the associated joint unless the mobilizer is reversed. */
  int inboard_body_num() const {return inboard_body_num_;}

  /** Returns the outboard body number of this mobilizer. This is the child
  body of the associated joint unless the mobilizer is reversed. */
  int outboard_body_num() const {return outboard_body_num_;}

  /** Return true if this mobilizer represents one of the input joints but
  the sense of inboard->outboard is reversed from the parent->child sense
  defined in the input joint. In that case you should use a reverse joint
  when you build the system. */
  bool is_reversed_from_joint() const { return is_reversed_; }

  /** Return the level of the outboard body. World is level 0, a base body
  is level 1, etc. */
  int level() const { return level_; }

  /** Return true if this mobilizer does not represent one of the input
  joints, but is instead a joint we added connecting a base body to World.
  If this returns true, the inboard body is always World. When you
  create this mobilizer, the joint frames should be identity, that is, the
  joint should connect the World frame to the outboard body frame. */
  bool is_added_base_mobilizer() const {
    return mgm_->get_joint(joint_num_).is_added_base_joint();
  }

  /** Get the joint type name of the joint that this mobilizer represents. */
  const std::string& get_joint_type_name() const {
    return mgm_->get_joint_type(mgm_->get_joint(joint_num_).joint_type_num())
        .name();
  }

  /** Return true if the outboard body of this mobilizer is a slave we
  created in order to cut a loop, rather than one of the input bodies. */
  bool is_slave_mobilizer() const {
    return mgm_->get_body(outboard_body_num_).is_slave();
  }

  /** Return the number of fragments into which we chopped the outboard body
  of this mobilizer. There is one fragment for the master body plus however
  many slaves of that body were created. Thus you should divide the master
  body's mass properties by this number to obtain the mass and inertia to be
  assigned to each of the body fragments. */
  int num_fragments() const {
    return mgm_->get_body(outboard_master_body_num()).num_fragments();
  }

  /** Returns the body number of the master body for this mobilizer's outboard
  body, which might be a slave. If the outboard body is not a slave body, then
  this returns the same value as outboard_body_num(). */
  int outboard_master_body_num() const {
    const Body& outboard_body = mgm_->get_body(outboard_body_num_);
    return outboard_body.is_slave() ? outboard_body.master_body_num()
                                    : outboard_body_num_;
  }

 private:
  // Restrict construction to only MultibodyGraphMaker.
  friend class MultibodyGraphMaker;

  Mobilizer(int joint_num, int level, int inboard_body_num,
            int outboard_body_num, bool is_reversed,
            MultibodyGraphMaker* graph_maker)
      : joint_num_(joint_num),
        level_(level),
        inboard_body_num_(inboard_body_num),
        outboard_body_num_(outboard_body_num),
        is_reversed_(is_reversed),
        mgm_(graph_maker) {
    DRAKE_DEMAND(joint_num_ >= 0);
    DRAKE_DEMAND(level_ >= 0);
    DRAKE_DEMAND(inboard_body_num_ >= 0);
    DRAKE_DEMAND(outboard_body_num_ >= 0);
    DRAKE_DEMAND(mgm_ != nullptr);
  }

  int joint_num_{-1};          // associated joint (not necessarily from input)
  int level_{-1};              // level of outboard body; distance from World
  int inboard_body_num_{-1};   // might be World
  int outboard_body_num_{-1};  // might be a slave body; can't be World
  bool is_reversed_{false};    // if reversed, inboard=child, outboard=parent

  MultibodyGraphMaker* mgm_{nullptr};  // just a reference to container
};

//------------------------------------------------------------------------------
//                  MULTIBODY GRAPH MAKER :: LOOP CONSTRAINT
//------------------------------------------------------------------------------
/** Local class that represents one of the constraints that were added to close
topological loops that were cut to form the spanning tree. */
class MultibodyGraphMaker::LoopConstraint {
 public:
  /** Returns `true` if this is a master/slave weld introduced to break a
  loop by cutting a body. In that case there is no associated joint. Otherwise,
  this is a loop joint that directly replaces a joint and you can find the
  joint using joint_num(). */
  bool is_master_slave_weld() const { return joint_num_ == -1; }

  /** Get the loop constraint type name of the constraint that should be
  used here. This will be either the type name of the associated joint, or the
  type name of a weld joint if this is a master/slave weld. */
  const std::string& get_constraint_type_name() const { return type_; }

  /** Returns the parent body number from the modeled input joint, or the
  master body number if this is a master/slave weld. */
  int parent_body_num() const {return parent_body_num_;}

  /** Returns the child body number from the modeled input joint, or the
  slave body number if this is a master/slave weld. */
  int child_body_num() const {return child_body_num_;}

  /** Returns the joint number if this loop constraint models one of the input
  joints, otherwise -1. */
  int joint_num() const {return joint_num_;}

 private:
  // Restrict construction to only MultibodyGraphMaker.
  friend class MultibodyGraphMaker;

  LoopConstraint(const std::string& type, int jointNum, int parent_body_num,
                 int child_body_num, MultibodyGraphMaker* graph_maker)
      : type_(type),
        joint_num_(jointNum),
        parent_body_num_(parent_body_num),
        child_body_num_(child_body_num),
        mgm_(graph_maker) {
    DRAKE_DEMAND(parent_body_num >= 0 && child_body_num >= 0);
    DRAKE_DEMAND(mgm_ != nullptr);
    DRAKE_DEMAND(
        joint_num_ >= 0 ||
        (joint_num_ == -1 && type_ == mgm_->get_weld_joint_type_name()));
  }

  std::string type_;         // e.g., ball
  int joint_num_{-1};        // set if one of the input joints
  int parent_body_num_{-1};  // parent from the joint, or master body
  int child_body_num_{-1};   // child from the joint, or slave body

  MultibodyGraphMaker* mgm_{nullptr};  // just a reference to container
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
