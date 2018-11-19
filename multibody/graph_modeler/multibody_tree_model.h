#pragma once

/* Adapted for Drake from Simbody's MultibodyGraphModeler class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Modifications copyright (c) 2019-20 Toyota Research Institute, Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

/** @file
Declares the MultibodyTreeModel class that contains a computational
spanning tree-plus-constraints model representing a MultibodyGraph. */

#include <iosfwd>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/multibody/graph_modeler/multibody_graph_modeler.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

//==============================================================================
//                           MULTIBODY TREE MODEL
//==============================================================================
/** Contains a computational spanning tree-plus-constraints model representing
the bodies and joints of a MultibodyGraph. A %MultibodyTreeModel uses
"mobilized bodies" (mobods) to model bodies, and "mobilizers" or "constraints"
to model joints.

In addition to the tree model, a %MultibodyTreeModel contains an abbreviated
summary of the bodies and joints forming the input graph that was used to
construct this model. The model consists of
 - one or more _mobilized bodies_ (mobods) for each input _body_,
 - one _mobilizer_ for each of the mobilized bodies, forming a tree, and
 - one _constraint_ for each kinematic loop in the input graph
   connectivity.

Bodies are cut as necessary to break loops, producing a master mobilized body
and one or more slave mobilized bodies, and "weld" constraints fixing each slave
to its master.

Each mobilized body has an "inboard" mobilizer, which connects it to World or a
mobilized body that is
closer to World in the multibody tree. In most cases, each input joint
is represented in the tree by a mobilizer, ideally with the joint's parent body
being the mobilizer's inboard mobod, and the joint's child body being the
mobilizer's outboard mobod. Depending on parent/child ordering in the input,
that is not always possible, so the joint's mobilizer will be marked as
"reversed" meaning inboard=child and outboard=parent instead. There will be more
mobilizers than joints in general, since free bodies in the input will have a
"free" (6 dof) mobilizer, and static bodies in the input will have a "weld" (0
dof) mobilizer ("immobilizer" might be a better word :).

When possible we use a mobilizer to model a joint by the degrees of freedom it
_permits_. However, we can instead use a joint constraint to model a joint by
the restrictions it _imposes_. Most commonly we use only weld constraints that
we have added to attach master and slave mobods, although some efficiency gains
are possible by using other joint constraints when available.

<h3>A basic example</h3>
Here is a usage example, assuming the MultibodyGraph input is: <pre>
       link1 <-- link2 <-- link3 --> link4 --> link5
</pre> where the arrows represent revolute joints and point from the "parent"
body to the "child" body. Note that no explicit connection to World (body 0) has
been given, so we will add one in the generated tree model.
@code
  MultibodyGraphModeler graph;
  // "world" is body 0
  graph.AddBody("link1");  // name (optional flags)
  graph.AddBody("link2");
  graph.AddBody("link3");
  graph.AddBody("link4");
  graph.AddBody("link5");
  graph.AddJoint("joint0", "revolute", "link2", "link1");  // parent, child
  graph.AddJoint("joint1", "revolute", "link3", "link2");
  graph.AddJoint("joint2", "revolute", "link3", "link4");
  graph.AddJoint("joint3", "revolute", "link4", "link5");
  MultibodyTreeModel model(graph);

  for (int i=0; i < model.num_mobilizers(); ++i) {
    auto& mobilizer = model.get_mobilizer(i);
    auto& inb = model.mobilized_body(mobilizer.inboard_mobod_num());
    auto& outb = model.mobilized_body(mobilizer.outboard_mobod_num());
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
showing that the algorithm picked link3 as the base mobilized body and was thus
able to preserve parent->child ordering for every joint. In actual use, the
contents of the mobilizer for-loop above would _build_ the multibody system
rather than _print_ it.

<h3>Explanation</h3>
Each body has a unique name; each joint connects two distinct bodies with one
designated as the "parent" body and the other the "child" body. The generated
model (of type MultibodyTreeModel) will include a spanning tree with World as
the root, with a mobilized body corresponding to every body, and containing a
mobilizer for every mobilized body. You can also think of this graph as a
forest of trees each with a "base mobilized body" as its root, where a base
mobilized body is a mobilized body directly connected to World (often by a
"free" or "floating" joint).

There is a mobilizer in the generated tree model corresponding to each of
the joints from the input, with an "inboard" (topologically closer to World)
and "outboard" (further from World) mobilized body, chosen from the parent and
child bodies of the joint but possibly reordered in which case the mobilizer is
marked as "reversed". Additional "free" mobilizers are added as needed between
bodies and World so that there is a path from every mobilized body in the
inboard direction all the way to World. Additional mobilized bodies and
constraints are added if topological loops are found in the input.

Heuristics here attempt to balance a number of competing goals:
  - choose "sensible" base mobilized bodies for trees that don't have a
    connection to World provided in the input,
  - make the mobilizer inboard/output ordering follow the parent/child ordering
    in the input,
  - cut loops such that branch lengths are balanced (minimizes the maximum
    length of any branch for better numerics),
  - make sure no massless mobilized body ends up as a terminal node of a tree
    (that would make the mass matrix singular),
  - follow any stated user preferences about the generated graph.

See below for more details.

<h3>Results</h3>
The output is
  -# A sequence of mobilizers (tree joints) in ascending order from World to a
set of terminal bodies, for each subtree. Every input body will be mobilized by
one of these, and there may also be some additional mobilized "slave" mobilized
bodies due to loop-breaking.
  -# A set of "loop constraints", representing either welds to attach slave
mobilized bodies to their masters, or loop joint constraints that correspond to
particular input joints.
  -# A correspondence between input bodies and mobilized bodies, with some
mobilized bodies designated as slaves to particular master mobilized bodies.
That is, more than one mobilized body may correspond to the same input body.
  -# A correspondence between mobilizers and input joints, with some extra
mobilizers having been added to connect base mobilized bodies to World.
  -# Statistics, diagnostics, and introspection of the resulting trees.

Then to build a multibody system
  -# Run through the generated mobilizers in order, adding one mobilized body to
the tree each time, using "reversed" mobilizers if appropriate (that just
affects the interpretation of the generalized coordinates). Mass properties and
joint placement information are obtained from the original bodies and joints;
they are not stored here.
  -# Run through the list of constraints adding master-slave welds or loop
joint constraints as indicated.

<h3>Inputs</h3>
  - bodies: name, mass, must_be_base_mobilized_body flag
  - joints: name, type, parent body, child body, must_be_loop_joint flag
  - joint type: name, # dofs, have_good_joint_constraint_available flag
  - names for the World body and important joint types weld and free (Drake
    defaults are available).

<h3>Loop joints</h3>
Normally every joint produces a corresponding mobilizer. A joint that would form
a kinematic loop is marked as such, and then its child body is split to form a
new "slave" mobilized body that can be mobilized by the joint. We expect that in
the constructed multibody model, a master mobilized body and its slaves will be
reconnected via weld constraints. This provides for uniform treatment of all
joints, such as providing revolute joint coordinates that can wrap. However, for
some joint types that doesn't matter because the multibody system has equally
good "loop joint" constraints. For example, Simbody's Ball mobilizer uses
quaternions for orientation and hence has no wrapping; it can be replaced with
a Ball constraint which removes the 3-dof mobilizer and 6 weld constraint
equations, leaving just 3 translational constraints and indistinguishable
behavior. Similarly a loop-closing Weld joint can just be replaced by a Weld
constraint, and a loop-closing Free joint can simply be ignored.

The algorithm normally decides which joints are the loop-breakers, however you
can specify in the input that certain joints must be loop joints.

<h3>Massless bodies</h3>
Massless bodies can be very useful in defining compound joints, by composing
revolute and prismatic joints, for example. This is fine in an internal
coordinate code as long as no massless (or inertialess) mobilized body is a
terminal (most outboard) mobilized body in the spanning tree. (Terminal massless
mobilized bodies are OK if their inboard mobilizer has no dofs, i.e. is a Weld.)
Bodies can have a mass provided; if a movable body has zero mass the algorithm
will try to avoid ending any branch of the spanning tree with that body's
mobilized body. If it fails, the resulting spanning tree is no good and an
exception will be thrown. The attempt to find an acceptable tree is heuristic
and can fail in difficult cases even if there might be a solution. Explicit user
guidance may be required in that case (for example, specifying
`must_be_base_mobilized_body` for some body.)

<h3>Base mobilized bodies</h3>
A mobilized body in the spanning tree that is directly connected to World is
called a "base" mobilized body. If its mobilizer is a Free joint then it can be
given an arbitrary pose with respect to World and everything outboard of it
moves together. This is a nice property and you can influence this choice by
providing an explicit joint to World or designating some bodies as required to
be modeled as base mobilized bodies. Note that although you will then have a set
of generalized coordinates permitting you to place these mobilized bodies
arbitrarily, there is no guarantee that such placement will satisfy constraints.
If you don't designate base mobilized bodies, the algorithm will pick
them heuristically, which often leads to a good choice. The heuristic is to
attempt to find a body that does not ever appear as a child in the input.
Failing that, pick the body that has the most children. In case of tie, pick
the first body among the tied ones.

<h3>World body</h3>
The first body defined in the input will be interpreted as the World body.
Its name will be used to recognize joints that connect other bodies to World.
Typical names are "world" or "ground". If you accept the Drake defaults, the
World body will be predefined as body zero and will be named "world". The
World body in the input graph is identical to the World mobilized body in the
generated spanning tree.

<h3>Levels</h3>
Each mobilized body in the spanning tree will be assigned a level, an integer
that measures how many edges must be traversed to get from that mobilized body
to World. World is at level 0, mobilized bodies that have a direct connection to
World (base mobilized bodies) are at level 1, mobilized bodies whose mobilizer
connects them to base mobilized bodies are at level 2, and so on. Multibody
systems need to be constructed in order of increasing level, so that the inboard
mobilized body is already in the tree when the outboard mobilized body is added.
We consider the level of a mobilizer to be the same as the level of its outboard
mobilized body. Loop joints are not mobilizers and do not have a level. */
class MultibodyTreeModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeModel)

  using MobilizedBodyNum = TypeSafeIndex<class MobilizedBodyNumTag>;
  using MobilizerNum = TypeSafeIndex<class MobilizerNumTag>;
  using ConstraintNum = TypeSafeIndex<class ConstraintNumTag>;

  using Body = MultibodyGraphModeler::Body;
  using Joint = MultibodyGraphModeler::Joint;
  using JointType = MultibodyGraphModeler::JointType;

  // Local classes.
  class MobilizedBody;
  class Mobilizer;
  class Constraint;
  class BodyInfo;
  class JointInfo;
  class JointTypeInfo;

  explicit MultibodyTreeModel(const MultibodyGraphModeler& graph) {
    MakeTreeModel(graph);
  }

  /** @name               Work with the generated tree model
  Methods in this section manipulate the generated multibody tree model
  consisting of mobilized bodies (from input bodies and additional slaves),
  mobilizers (from input joints plus additional connections to World), and loop
  joint constraints. */
  //@{

  /** Output a text representation of the multibody tree model for debugging. */
  void DumpTreeModel(std::ostream& out) const;

  /** Returns the number of mobilizers (tree joints) in the spanning forest.
  This is also the number of mobilized bodies, including slave mobilized
  bodies. */
  int num_mobilizers() const { return static_cast<int>(mobilizers_.size()); }

  /** Gets a Mobilizer object by its mobilizer number, ordered outwards by
  topological distance from World. */
  const Mobilizer& get_mobilizer(MobilizerNum mobilizer_num) const {
    return mobilizers_[mobilizer_num];
  }

  /** Returns the number of joint constraints that were used to close
  loops in the graph topology. These include loops that were broken
  by cutting a body to make master and slave mobilized bodies, and those
  where the joint itself was implemented using a constraint rather than a
  mobilizer plus a slave mobilized body. The latter occurs only if we're told
  there is a perfectly good loop joint constraint available; typically that
  applies for ball (spherical) joints and not much else. */
  int num_constraints() const { return static_cast<int>(constraints_.size()); }

  /** Gets a loop constraint by its assigned number. These are assigned in
  an arbitrary order. */
  const Constraint& get_constraint(ConstraintNum loop_constraint_num) const {
    return constraints_[loop_constraint_num];
  }

  /** Returns the number of mobilized bodies, including a World body, mobilized
  bodies directly representing an input body, and any slave mobilized bodies. */
  int num_mobods() const { return static_cast<int>(mobods_.size()); }

  /** Gets a MobilizedBody object by its assigned number. These are assigned
  first to World (body 0), then bodies representing input bodies, then we add
  slave bodies created by body splitting after that. */
  const MobilizedBody& mobod(MobilizedBodyNum mobod_num) const {
    return mobods_[mobod_num];
  }

  /** Returns the mobilized body number assigned to the input body with the
  given name. Returns an invalid MobilizedBodyNum if the body name is not
  recognized. You can't look up by name slave mobilized bodies that were added
  by the model-making algorithm. */
  MobilizedBodyNum FindBodyMobodNum(const std::string& body_name) const {
    std::map<std::string, MobilizedBodyNum>::const_iterator p =
        body_name_to_mobod_num_.find(body_name);
    return p == body_name_to_mobod_num_.end() ? MobilizedBodyNum() : p->second;
  }

  /** Returns the set of base bodies (bodies connected directly to World) in
  the generated multibody model. These are ordered by increasing mobilizer
  number of their base mobilizers. */
  std::vector<MobilizedBodyNum> FindBaseBodies() const;

  /** Returns the base mobilized body of the subtree containing the given
  `mobod_num`. If `mobod_num` is a base mobilized body it is returned. If
  `mobod_num` is World we return an invalid MobilizedBodyNum. */
  MobilizedBodyNum FindBaseBody(MobilizedBodyNum mobod_num) const;

  /** Returns a list of mobilized bodies, starting with `mobod_num` and ending
  with the base mobilized body that connects that mobilized body to World
  through inboard mobilizers. Will have just one element if `mobod_num` is a
  base mobilized body, and be empty if `mobod_num` is the World body. */
  std::vector<MobilizedBodyNum> FindPathToWorld(
      MobilizedBodyNum mobod_num) const;

  /** Returns the MobilizedBodyNum of the master mobilized body being used to
  model the given body. */
  inline MobilizedBodyNum body_to_mobod_num(BodyIndex body_num) const;

  /** Returns the BodyIndex associated with this mobilized body, if any. */
  inline BodyIndex mobod_to_body_num(MobilizedBodyNum mobod_num) const;

  /** Returns the master MobilizedBody being used to model the given body. */
  const MobilizedBody& body_to_mobod(BodyIndex body_num) const {
    return mobod(body_to_mobod_num(body_num));
  }
  //@}

  int num_bodies() const { return static_cast<int>(body_info_.size()); }

  const BodyInfo& body_info(BodyIndex body_num) const {
    return body_info_[body_num];
  }

  int num_joints() const { return static_cast<int>(joint_info_.size()); }

  const JointInfo& joint_info(JointIndex joint_num) const {
    return joint_info_[joint_num];
  }

  int num_joint_types() const {
    return static_cast<int>(joint_type_info_.size());
  }
  const JointTypeInfo& joint_type_info(JointTypeIndex joint_type_num) const {
    return joint_type_info_[joint_type_num];
  }

  inline const std::string& world_body_name() const;
  MobilizedBodyNum world_mobod_num() const { return MobilizedBodyNum(0); }

  // TODO(sherm1) Make these less brittle.
  JointTypeIndex weld_joint_type_num() const { return JointTypeIndex(0); }
  JointTypeIndex free_joint_type_num() const { return JointTypeIndex(1); }

 private:
  // Restores this %MultibodyTreeModel object to its just-constructed
  // condition.
  void Clear();

  void MakeTreeModel(const MultibodyGraphModeler& graph);
  BodyIndex ChooseNewBaseBody(const MultibodyGraphModeler& graph) const;
  JointIndex FindHeaviestUnassignedForwardJoint(
      BodyIndex parent_body_num, const MultibodyGraphModeler& graph) const;
  JointIndex FindHeaviestUnassignedReverseJoint(
      BodyIndex parent_body_num, const MultibodyGraphModeler& graph) const;
  void GrowTree(const MultibodyGraphModeler& graph, int start_level);
  void BreakLoops(const MultibodyGraphModeler& graph);

  JointTypeIndex AddJointType(std::string joint_type_name,
                              void* joint_type_user_ref);

  BodyIndex AddBodyInfo(std::string body_name, void* body_user_ref);

  JointIndex AddJointInfo(std::string joint_name, JointTypeIndex joint_type_num,
                          void* joint_user_ref);

  MobilizedBodyNum AddMobodFromBody(BodyIndex body_num);

  MobilizerNum AddMobilizerFromJoint(JointIndex joint_num,
                                     MobilizedBodyNum inboard_mobod_num,
                                     MobilizedBodyNum outboard_mobod_num,
                                     bool is_reversed);

  ConstraintNum AddConstraintFromJoint(JointIndex joint_num,
                                       MobilizedBodyNum parent_mobod_num,
                                       MobilizedBodyNum child_mobod_num);

  ConstraintNum AddSlaveWeldConstraint(std::string constraint_name,
                                       JointIndex joint_num,
                                       MobilizedBodyNum master_mobod_num,
                                       MobilizedBodyNum slave_mobod_num);

  // Adds a free or weld mobilizer.
  MobilizerNum ConnectBodyToWorld(MobilizedBodyNum mobod_num, bool is_static);

  // Get writable access to bodies.
  MobilizedBody& get_mutable_body(MobilizedBodyNum mobod_num) {
    return mobods_[mobod_num];
  }

  MobilizedBodyNum SplitBody(MobilizedBodyNum master_mobod_num);
  bool MobodsAreConnected(MobilizedBodyNum mobod1_num,
                          MobilizedBodyNum mobod2_num) const {
    return MobodsAreConnectedByMobilizer(mobod1_num, mobod2_num) ||
           MobodsAreConnectedByJointConstraint(mobod1_num, mobod2_num);
  }
  bool MobodsAreConnectedByMobilizer(MobilizedBodyNum mobod1_num,
                                     MobilizedBodyNum mobod2_num) const;
  bool MobodsAreConnectedByJointConstraint(MobilizedBodyNum mobod1_num,
                                           MobilizedBodyNum mobod2_num) const;

  // Calculated by MultibodyGraphModeler::MakeTreeModel().
  // Index by MobilizedBodyNum, MobilizerNum, ConstraintNum, resp.
  std::vector<MobilizedBody> mobods_;    // world + input bodies + slaves
  std::vector<Mobilizer> mobilizers_;    // mobilized bodies
  std::vector<Constraint> constraints_;  // master/slave welds, loop joints

  // Information about bodies, modified as model is built. Order and numbering
  // here are identical to bodies in MultibodyGraphModeler at the time this
  // model was generated.
  std::map<std::string, MobilizedBodyNum> body_name_to_mobod_num_;
  std::vector<BodyInfo> body_info_;  // Index by BodyIndex.

  // Information about joints, modified as model is built. Order and numbering
  // here are identical to joints in MultibodyGraphModeler at the time this
  // model was generated. Index by JointIndex.
  std::vector<JointInfo> joint_info_;

  // Information about joint types. Order and numbering here is identical to
  // joint types in MultibodyGraphModeler at the time this model was generated.
  // Index by JointTypeIndex.
  std::vector<JointTypeInfo> joint_type_info_;
};

//------------------------------------------------------------------------------
//                    MULTIBODY TREE MODEL :: BODY INFO
//------------------------------------------------------------------------------
/** Local class that collects information about how we modeled a given
input body. */
class MultibodyTreeModel::BodyInfo {
 public:
  // Move only.
  BodyInfo(const BodyInfo&) = delete;
  BodyInfo& operator=(const BodyInfo&) = delete;
  BodyInfo(BodyInfo&&) = default;
  BodyInfo& operator=(BodyInfo&&) = default;

  const std::string& body_name() const { return body_name_; }
  void* user_ref() const { return user_ref_; }
  MobilizedBodyNum master_mobod_num() const { return master_mobod_num_; }

 private:
  friend class MultibodyTreeModel;

  BodyInfo(std::string body_name, void* user_ref)
      : body_name_(std::move(body_name)), user_ref_(user_ref) {}

  void set_master_mobod_num(MobilizedBodyNum master_mobod_num) {
    DRAKE_DEMAND(!master_mobod_num_.is_valid() && master_mobod_num.is_valid());
    master_mobod_num_ = master_mobod_num;
  }

  // Copied from the input body.
  std::string body_name_;
  void* user_ref_{nullptr};

  // Determined during model generation.
  MobilizedBodyNum master_mobod_num_;
};

//------------------------------------------------------------------------------
//                   MULTIBODY TREE MODEL :: JOINT INFO
//------------------------------------------------------------------------------
/** Local class that collects information about how we modeled a given
input joint. */
class MultibodyTreeModel::JointInfo {
 public:
  // Move only.
  JointInfo(const JointInfo&) = delete;
  JointInfo& operator=(const JointInfo&) = delete;
  JointInfo(JointInfo&&) = default;
  JointInfo& operator=(JointInfo&&) = default;

  const std::string& joint_name() const { return joint_name_; }
  void* user_ref() const { return user_ref_; }
  JointTypeIndex joint_type_num() const { return joint_type_num_; }

  bool has_mobilizer() const { return mobilizer_num_.is_valid(); }

  /** Returns the mobilizer that represents this joint, if any. Otherwise the
  returned value is not valid. */
  MobilizerNum mobilizer_num() const { return mobilizer_num_; }

  /** Returns the constraint that represents this joint, if any. Otherwise the
  returned value is not valid. */
  ConstraintNum constraint_num() const { return constraint_num_; }

 private:
  friend class MultibodyTreeModel;
  JointInfo(std::string joint_name, JointTypeIndex joint_type_num,
            void* user_ref)
      : joint_name_(std::move(joint_name)),
        joint_type_num_(joint_type_num),
        user_ref_(user_ref) {}

  void set_mobilizer_num(MobilizerNum num) { mobilizer_num_ = num; }

  void set_constraint_num(ConstraintNum num) { constraint_num_ = num; }

  // Copied from the input joint.
  std::string joint_name_;
  JointTypeIndex joint_type_num_;
  void* user_ref_{nullptr};

  // Determined during model generation (only one of these will be valid).
  MobilizerNum mobilizer_num_;    // If modeled with a mobilizer.
  ConstraintNum constraint_num_;  // If modeled with a constraint.
};

//------------------------------------------------------------------------------
//                   MULTIBODY TREE MODEL :: JOINT TYPE INFO
//------------------------------------------------------------------------------
/** Local class that holds joint type information we can reference from
mobilizers and joint constraints. */
class MultibodyTreeModel::JointTypeInfo {
 public:
  // Move only.
  JointTypeInfo(const JointTypeInfo&) = delete;
  JointTypeInfo& operator=(const JointTypeInfo&) = delete;
  JointTypeInfo(JointTypeInfo&&) = default;
  JointTypeInfo& operator=(JointTypeInfo&&) = default;

  /** Returns the name of the original joint type. */
  const std::string& joint_type_name() const { return joint_type_name_; }

  /** Returns the mobilizer type name that should be used when a joint of this
  type is modeled with a mobilizer. */
  // TODO(sherm1) Provide for a different name.
  const std::string& mobilizer_type_name() const { return joint_type_name_; }

  /** Returns the constraint type name that should be used when a joint of this
  type is modeled with a constraint. */
  // TODO(sherm1) Provide for a different name.
  const std::string& constraint_type_name() const { return joint_type_name_; }

  /** Return the user reference pointer for this joint type if one was provided
  in MultibodyGraphModeler. */
  void* user_ref() const { return user_ref_; }

 private:
  friend class MultibodyTreeModel;

  JointTypeInfo(const std::string& name, void* user_ref)
      : joint_type_name_(name), user_ref_(user_ref) {}

  std::string joint_type_name_;
  void* user_ref_{nullptr};  // Copied from the input joint type.
};

//------------------------------------------------------------------------------
//                         MULTIBODY TREE MODEL :: BODY
//------------------------------------------------------------------------------
/** Local class that accumulates modeling information and represents bodies as
the multibody tree is generated. */
class MultibodyTreeModel::MobilizedBody {
 public:
  // Move only.
  MobilizedBody(const MobilizedBody&) = delete;
  MobilizedBody& operator=(const MobilizedBody&) = delete;
  MobilizedBody(MobilizedBody&&) = default;
  MobilizedBody& operator=(MobilizedBody&&) = default;

  /** Returns the number of fragments into which we had to break the
  corresponding body. Normally this is just one, meaning we didn't break the
  body, but master mobilized bodies that have slaves will return the number of
  slaves plus one. */
  int num_fragments() const { return 1 + num_slaves(); }

  /** Returns the number of slave bodies associated with this master body.
  Normally zero, but a master body with (for example) three slave bodies would
  return three. */
  int num_slaves() const { return static_cast<int>(slaves_.size()); }

  /** Returns `true` if this mobilized body is a slave mobilized body generated
  by breaking an input body into multiple mobilized bodies. If so you can find
  its master using master_mobod_num(). */
  bool is_slave() const { return master_mobod_num_.is_valid(); }

  /** Returns `true` if this is a mobilized body that represents a body that had
  to be broken into multiple mobilized bodies. If so you can find the generated
  slave mobilized bodies using slaves(). */
  bool is_master() const { return num_slaves() > 0; }

  /** Returns `true` if this is the body that represents the World. */
  bool is_world_body() const { return level_ == 0; }

  const std::string& name() const { return name_; }

  /** Returns `true` if this mobilized body has already been assigned a place in
  the multibody tree. This will be true for all mobilized bodies once model
  building is complete. */
  bool is_in_tree() const { return level_ >= 0; }

  /** Returns the level of this mobilized body in the multibody tree. If this is
  World it is level 0, if it's a base mobilized body it's level 1, if it's
  connected to a base mobilized body it's level 2, etc. The level of a mobilized
  body is the same as the level of its inboard mobilizer as obtained with
  mobilizer_num(). */
  int level() const { return level_; }

  /** Returns the unique mobilizer (by number) for which this mobilized body is
  the outboard body, or an invalid MobilizerNum if the multibody tree has not
  yet been generated. */
  MobilizerNum mobilizer_num() const { return mobilizer_num_; }

  /** Returns the number of mobilizers for which this mobilized body is the
  inboard mobilized body. */
  int num_outboard_mobilizers() const {
    return static_cast<int>(outboard_mobilizers().size());
  }

  /** Returns a reference to a vector of MobilizerNum values containing all
  the mobilizers for which this mobilized body is the inboard body. */
  const std::vector<MobilizerNum>& outboard_mobilizers() const {
    return outboard_mobilizers_;
  }

  int num_joint_constraints() const {
    return static_cast<int>(joint_constraints().size());
  }

  const std::vector<ConstraintNum>& joint_constraints() const {
    return joint_constraints_;
  }

  /** Returns the list of slave bodies (by mobilized body number), if this is a
  master body. */
  const std::vector<MobilizedBodyNum>& slaves() const { return slaves_; }

  /** Returns the master mobilized body number if this is a slave mobilized
  body, otherwise returns an invalid MobilizedBodyNum. */
  MobilizedBodyNum master_mobod_num() const { return master_mobod_num_; }

  /** Returns the body number of the body modeled by this mobilized body. Master
  and slave mobilized bodies both return the same body number. */
  BodyIndex body_num() const { return body_num_; }

 private:
  // Restrict construction and modification to only MultibodyTreeModel.
  friend class MultibodyTreeModel;

  // Create a mobilized body that directly models a body.
  MobilizedBody(BodyIndex body_num, MultibodyTreeModel* model)
      : name_(model->body_info(body_num).body_name()),
        body_num_(body_num),
        model_(model) {}

  // Create a slave mobilized body for the given master.
  MobilizedBody(std::string name, MobilizedBodyNum master_mobod_num,
                MultibodyTreeModel* model)
      : name_(std::move(name)),
        body_num_(model->mobod(master_mobod_num).body_num()),
        master_mobod_num_(master_mobod_num),
        model_(model) {}

  void set_level(int level) { level_ = level; }
  void set_mobilizer_num(MobilizerNum mobilizer_num) {
    mobilizer_num_ = mobilizer_num;
  }

  // When this mobilized body serves as the inboard body for a mobilizer, note
  // that.
  void add_outboard_mobilizer_num(MobilizerNum outboard_mobilizer_num) {
    outboard_mobilizers_.push_back(outboard_mobilizer_num);
  }

  // When this mobilized body participates in a joint constraint, note that.
  void add_joint_constraint_num(ConstraintNum joint_constraint_num) {
    joint_constraints_.push_back(joint_constraint_num);
  }

  // Records the master body number for this slave mobilized body.
  void set_master_mobod_num(MobilizedBodyNum master_mobod_num) {
    master_mobod_num_ = master_mobod_num;
  }

  // Records a slave mobilized body number for this master body.
  void add_slave_mobod_num(MobilizedBodyNum slave_mobod_num) {
    slaves_.push_back(slave_mobod_num);
  }

  std::string name_;    // Body name if this was an input body.
  BodyIndex body_num_;  // Original body number if any.

  // Disposition of this mobilized body in the spanning tree.

  int level_{-1};               // Distance from World in the tree.
  MobilizerNum mobilizer_num_;  // The unique inboard mobilizer.
  std::vector<MobilizerNum> outboard_mobilizers_;

  // Joint constraints in which this mobilized body participates.
  std::vector<ConstraintNum> joint_constraints_;

  MobilizedBodyNum master_mobod_num_;     // valid if this is a slave.
  std::vector<MobilizedBodyNum> slaves_;  // Slave bodies, if this is a master.

  MultibodyTreeModel* const model_{nullptr};  // just a reference to container
};

//------------------------------------------------------------------------------
//                   MULTIBODY TREE MODEL :: MOBILIZER
//------------------------------------------------------------------------------
/** Local class that represents one of the mobilizers (tree joints) in the
generated spanning tree. */
class MultibodyTreeModel::Mobilizer {
 public:
  // Move only.
  Mobilizer(const Mobilizer&) = delete;
  Mobilizer& operator=(const Mobilizer&) = delete;
  Mobilizer(Mobilizer&&) = default;
  Mobilizer& operator=(Mobilizer&&) = default;

  /** Returns the mobilizer's name. This will be the input joint's name if this
  mobilizer models one of the input joints. Otherwise, it will have a name
  constructed from the inboard and output mobilized body names. */
  const std::string& name() const { return name_; }

  /** Returns the joint associated with this mobilizer. There will always be
  one, but it may have been an added joint rather than an input joint. */
  JointIndex joint_num() const { return joint_num_; }

  /** Returns the inboard mobilized body number of this mobilizer. This is the
  parent mobilized body of the associated joint unless the mobilizer is
  reversed. */
  MobilizedBodyNum inboard_mobod_num() const { return inboard_mobod_num_; }

  /** Returns the outboard mobilized body number of this mobilizer. This is the
  child
  mobilized body of the associated joint unless the mobilizer is reversed. */
  MobilizedBodyNum outboard_mobod_num() const { return outboard_mobod_num_; }

  /** Return true if this mobilizer represents one of the input joints but
  the sense of inboard->outboard is reversed from the parent->child sense
  defined in the input joint. In that case you should use a reverse mobilizer
  when you build the system. */
  bool is_reversed_from_joint() const { return is_reversed_; }

  /** Return the level of the outboard body. World is level 0, a base mobilized
  body is level 1, etc. */
  int level() const { return level_; }

  /** Return true if this mobilizer does not represent one of the input
  joints, but is instead a mobilizer we added connecting a base mobilized body
  to World. If this returns true, the inboard mobilized body is always World.
  When you create this mobilizer, the joint frames should be identity, that is,
  the joint should connect the World frame to the outboard body frame. */
  bool is_added_base_mobilizer() const { return !joint_num_.is_valid(); }

  /** Get the mobilizer type name. These are mapped from joint type names. */
  const std::string& get_joint_type_name() const {
    return model_->joint_type_info(mobilizer_type_num_).mobilizer_type_name();
  }

  /** Return true if the outboard mobilized body of this mobilizer is a slave we
  created in order to cut a loop, rather than one of the input bodies. */
  bool is_slave_mobilizer() const {
    return model_->mobod(outboard_mobod_num_).is_slave();
  }

  /** Return the number of fragments into which we chopped the outboard
  mobilized body of this mobilizer. There is one fragment for the master
  mobilized body plus however many slaves of that mobilized body were created.
  Thus you should divide the body's mass properties by this number to obtain the
  mass and inertia to be assigned to each of the mobilized body fragments. */
  int num_fragments() const {
    return model_->mobod(outboard_master_mobod_num()).num_fragments();
  }

  /** Returns the mobilized body number of the master body for this mobilizer's
  outboard mobilized body, which might be a slave. If the outboard body is not a
  slave mobilized body, then this returns the same value as
  outboard_mobod_num(). */
  MobilizedBodyNum outboard_master_mobod_num() const {
    const MobilizedBody& outboard_mobod = model_->mobod(outboard_mobod_num_);
    return outboard_mobod.is_slave() ? outboard_mobod.master_mobod_num()
                                     : outboard_mobod_num_;
  }

 private:
  // Restrict construction to only MultibodyTreeModel.
  friend class MultibodyTreeModel;

  // Construct a mobilizer that directly represents an input joint.
  Mobilizer(JointIndex joint_num, int level, MobilizedBodyNum inboard_mobod_num,
            MobilizedBodyNum outboard_mobod_num, bool is_reversed,
            MultibodyTreeModel* model)
      : joint_num_(joint_num),
        is_reversed_(is_reversed),
        level_(level),
        inboard_mobod_num_(inboard_mobod_num),
        outboard_mobod_num_(outboard_mobod_num),
        model_(model) {
    DRAKE_DEMAND(joint_num_.is_valid());
    DRAKE_DEMAND(level_ >= 0);
    DRAKE_DEMAND(inboard_mobod_num_.is_valid());
    DRAKE_DEMAND(outboard_mobod_num_.is_valid());
    DRAKE_DEMAND(model_ != nullptr);

    name_ = model->joint_info(joint_num).joint_name();
    mobilizer_type_num_ = model->joint_info(joint_num).joint_type_num();
  }

  // Construct a mobilizer for which there is no input joint counterpart.
  Mobilizer(std::string name, JointTypeIndex joint_type_num, int level,
            MobilizedBodyNum inboard_mobod_num,
            MobilizedBodyNum outboard_mobod_num, MultibodyTreeModel* model)
      : name_(std::move(name)),
        mobilizer_type_num_(joint_type_num),
        level_(level),
        inboard_mobod_num_(inboard_mobod_num),
        outboard_mobod_num_(outboard_mobod_num),
        model_(model) {
    DRAKE_DEMAND(mobilizer_type_num_.is_valid());
    DRAKE_DEMAND(level_ >= 0);
    DRAKE_DEMAND(inboard_mobod_num_.is_valid());
    DRAKE_DEMAND(outboard_mobod_num_.is_valid());
    DRAKE_DEMAND(model_ != nullptr);
  }

  std::string name_;  // Will be input joint name if possible.
  JointTypeIndex mobilizer_type_num_;

  // If this mobilizer directly models an input joint, note the joint number
  // and whether the mobilizer is reversed s.t. inboard=child, outboard=parent.
  JointIndex joint_num_;
  bool is_reversed_{false};

  int level_{-1};  // Level of outboard body; distance from World.
  MobilizedBodyNum inboard_mobod_num_;   // Might be World.
  MobilizedBodyNum outboard_mobod_num_;  // Might be a slave mobilized body;
                                         // can't be World.

  MultibodyTreeModel* const model_{nullptr};  // Just a reference to container.
};

//------------------------------------------------------------------------------
//                    MULTIBODY TREE MODEL :: CONSTRAINT
//------------------------------------------------------------------------------
/** Local class that represents one of the constraints that were added to close
topological loops that were cut to form the spanning tree. */
class MultibodyTreeModel::Constraint {
 public:
  const std::string& name() const { return name_; }

  /** Get the loop constraint type name of the constraint that should be
  used here. This will be either the type name of the associated joint, or the
  type name of a weld joint if this is a master/slave weld. */
  const std::string& constraint_type_name() const {
    return model_->joint_type_info(constraint_type_num_).constraint_type_name();
  }

  /** Returns the parent mobilized body number from the modeled input joint, or
  the master mobilized body number if this is a master/slave weld. */
  MobilizedBodyNum parent_mobod_num() const { return parent_mobod_num_; }

  /** Returns the child mobilized body number from the modeled input joint, or
  the slave mobilized body number if this is a master/slave weld. */
  MobilizedBodyNum child_mobod_num() const { return child_mobod_num_; }

  /** Returns the joint number if this loop constraint models one of the input
  joints, otherwise returns an invalid JointIndex. */
  JointIndex joint_num() const { return joint_num_; }

 private:
  // Restrict construction to only MultibodyTreeModel.
  friend class MultibodyTreeModel;

  // Construct a constraint that directly represents an input joint.
  Constraint(JointIndex joint_num, MobilizedBodyNum parent_mobod_num,
             MobilizedBodyNum child_mobod_num, MultibodyTreeModel* model)
      : joint_num_(joint_num),
        parent_mobod_num_(parent_mobod_num),
        child_mobod_num_(child_mobod_num),
        model_(model) {
    DRAKE_DEMAND(joint_num.is_valid());
    DRAKE_DEMAND(parent_mobod_num.is_valid() && child_mobod_num.is_valid());
    DRAKE_DEMAND(model != nullptr);

    name_ = model->joint_info(joint_num).joint_name();
    constraint_type_num_ = model->joint_info(joint_num).joint_type_num();
  }

  // Construct a constraint that does not directly represent an input joint,
  // but may be associated with one.
  Constraint(std::string name, JointTypeIndex joint_type_num,
             JointIndex joint_num, MobilizedBodyNum parent_mobod_num,
             MobilizedBodyNum child_mobod_num, MultibodyTreeModel* model)
      : name_(std::move(name)),
        constraint_type_num_(joint_type_num),
        joint_num_(joint_num),
        parent_mobod_num_(parent_mobod_num),
        child_mobod_num_(child_mobod_num),
        model_(model) {
    DRAKE_DEMAND(joint_type_num.is_valid());
    DRAKE_DEMAND(parent_mobod_num.is_valid() && child_mobod_num.is_valid());
    DRAKE_DEMAND(model_ != nullptr);
  }

  std::string name_;  // Will be input joint name if possible.
  JointTypeIndex constraint_type_num_;

  JointIndex joint_num_;  // Set if one of the input joints.

  MobilizedBodyNum parent_mobod_num_;  // parent from the joint, or master mobod
  MobilizedBodyNum child_mobod_num_;   // child from the joint, or slave mobod

  MultibodyTreeModel* model_{nullptr};  // Just a reference to container.
};

inline auto MultibodyTreeModel::body_to_mobod_num(BodyIndex body_num) const
    -> MobilizedBodyNum {
  const BodyInfo& body_info = this->body_info(body_num);
  return body_info.master_mobod_num();
}

inline auto MultibodyTreeModel::mobod_to_body_num(
    MobilizedBodyNum mobod_num) const -> BodyIndex {
  const MobilizedBody& mobod = this->mobod(mobod_num);
  return mobod.body_num();
}

inline const std::string& MultibodyTreeModel::world_body_name() const {
  DRAKE_DEMAND(!mobods_.empty());  // World should always be here.
  return mobod(MobilizedBodyNum(0)).name();
}

}  // namespace multibody
}  // namespace drake
