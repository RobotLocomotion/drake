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
Declares the MultibodyGraph class that contains the result from an invocation
of MultibodyGraphMaker::GenerateGraph(). */

#include <iosfwd>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

//==============================================================================
//                             MULTIBODY GRAPH
//==============================================================================
/** Holds a result from MultibodyGraphMaker. Only MultibodyGraphMaker may
create one of these.

Contains an abbreviated summary of the links and joints forming the input
graph that was in place when this %MultibodyGraph was generated, and
the resulting model as a spanning tree plus constraints. The model consists of
 - one or more _bodies_ for each input _link_,
 - one _mobilizer_ for each of the bodies, forming a tree, and
 - one _joint constraint_ for each kinematic loop in the input graph
   connectivity.

Links are cut as necessary to break loops, producing a master body and one
or more slave bodies, and "weld" constraints fixing each slave to its master.

Each body has an "inboard" mobilizer, which connects it to a body that is
closer to World in the multibody tree. In most cases, each input joint
is represented in the tree by a mobilizer, ideally with the joint's parent link
being the mobilizer's inboard body, and the joint's child link being the
mobilizer's outboard body. Depending on parent/child ordering in the input, that
is not always possible, so the joint's mobilizer will be marked as "reversed"
meaning inboard=child and outboard=parent instead. There will be more mobilizers
than joints in general, since free links in the input will have a "free" (6 dof)
mobilizer, and static links in the input will have a "weld" (0 dof) mobilizer
("immobilizer" might be a better word :).

When possible we use a mobilizer to model a joint by the degrees of freedom it
permits. However, we can instead use a joint constraint to model a joint by the
restrictions it imposes. Most commonly we use only weld constraints that we have
added to attach master and slave bodies, although some efficiency gains are
possible by using other joint constraints when available.
*/
class MultibodyGraph {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyGraph)

  /** Boolean properties of a link that can affect the resulting graph. These
  may be or-ed together and the result type is still LinkFlags. */
  enum LinkFlags : unsigned {
    kDefaultLinkFlags = 0b0000,
    kStaticLink = 0b0001,
    kMustBeBaseBody = 0b0010,
    kMustNotBeTerminalBody = 0b0100,
  };

  /** Boolean properties of a joint that can affect the resulting graph. */
  enum JointFlags : unsigned {
    kDefaultJointFlags = 0b0000,
    kMustBeConstraint = 0b0001,
  };

  /** Boolean properties of a joint _type_ that can affect the resulting
  graph. */
  enum JointTypeFlags : unsigned {
    kDefaultJointTypeFlags = 0b0000,
    kOkToUseAsJointConstraint = 0b0001,
  };

  using LinkNum = TypeSafeIndex<class LinkNumTag>;
  using JointNum = TypeSafeIndex<class JointNumTag>;
  using JointTypeNum = TypeSafeIndex<class JointTypeNumTag>;
  using BodyNum = TypeSafeIndex<class BodyNumTag>;
  using MobilizerNum = TypeSafeIndex<class MobilizerNumTag>;
  using ConstraintNum = TypeSafeIndex<class ConstraintNumTag>;

  // Local classes.
  class Body;
  class Mobilizer;
  class Constraint;
  class LinkInfo;
  class JointInfo;
  class JointTypeInfo;

  /** @name               Work with the generated graph
  Methods in this section manipulate the generated multibody graph consisting of
  bodies (input links and additional slaves), mobilizers (input joints plus
  additional connections to World), and loop constraints. */
  //@{

  /** Output a text representation of the multibody graph for debugging. */
  void DumpGraph(std::ostream& out) const;

  /** Returns the number of mobilizers (tree joints) in the spanning forest.
  This is also the number of mobilized bodies, including slave bodies. */
  int num_mobilizers() const { return static_cast<int>(mobilizers_.size()); }

  /** Gets a Mobilizer object by its mobilizer number, ordered outwards by
  topological distance from World. */
  const Mobilizer& get_mobilizer(MobilizerNum mobilizer_num) const {
    return mobilizers_[mobilizer_num];
  }

  /** Returns the number of joint constraints that were used to close
  loops in the graph topology. These include loops that were broken
  by cutting a body to make a slave body, and those where the joint itself
  was implemented using a constraint rather than a mobilizer plus a slave.
  The latter occurs only if we're told there is a perfectly good loop joint
  constraint available; typically that applies for ball (spherical) joints and
  not much else. */
  int num_constraints() const {
    return static_cast<int>(constraints_.size());
  }

  /** Gets a loop constraint by its assigned number. These are assigned in
  an arbitrary order. */
  const Constraint& get_constraint(ConstraintNum loop_constraint_num) const {
    return constraints_[loop_constraint_num];
  }

  /** Returns the number of bodies, including a World body, bodies directly
  representing an input link, and any slave bodies. */
  int num_bodies() const { return static_cast<int>(bodies_.size()); }

  /** Gets a Body object by its assigned number. These are assigned first to
  World (body 0), then input links, then we add slave bodies created by link
  splitting after that. */
  const Body& body(BodyNum body_num) const { return bodies_[body_num]; }

  /** Returns the body number assigned to the input link with the given name.
  Returns an invalid BodyNum if the link name is not recognized. You can't look
  up by name slave bodies that were added by the graph-making algorithm. */
  BodyNum FindLinkBodyNum(const std::string& link_name) const {
    std::map<std::string, BodyNum>::const_iterator p =
        link_name_to_body_num_.find(link_name);
    return p == link_name_to_body_num_.end() ? BodyNum() : p->second;
  }

  /** Returns the set of base bodies (bodies connected directly to World) in
  the generated multibody graph. These are ordered by increasing mobilizer
  number of their base mobilizers. */
  std::vector<BodyNum> FindBaseBodies() const;

  /** Returns the base body of the subtree containing the given `body_num`.
  If `body_num` is a base body it is returned. If `body_num` is World we
  return an invalid BodyNum. */
  BodyNum FindBaseBody(BodyNum body_num) const;

  /** Returns a list of bodies, starting with `body_num` and ending with the
  base body that connects that body to World through inboard mobilizers.
  Will have just one element if `body_num` is a base body, and be empty if
  `body_num` is the World body. */
  std::vector<BodyNum> FindPathToWorld(BodyNum body_num) const;

  /** Returns the BodyNum of the master body being used to model the
  given link. */
  inline BodyNum link_to_body_num(LinkNum link_num) const;

  /** Returns the LinkNum associated with this body, if any. */
  inline LinkNum body_to_link_num(BodyNum body_num) const;

  /** Returns the master Body being used to model the given link. */
  const Body& link_to_body(LinkNum link_num) const {
    return body(link_to_body_num(link_num));
  }
  //@}

  int num_links() const { return static_cast<int>(link_info_.size()); }

  const LinkInfo& link_info(LinkNum link_num) const {
    return link_info_[link_num];
  }

  int num_joints() const { return static_cast<int>(joint_info_.size()); }

  const JointInfo& joint_info(JointNum joint_num) const {
    return joint_info_[joint_num];
  }

  int num_joint_types() const {
    return static_cast<int>(joint_type_info_.size());
  }
  const JointTypeInfo& joint_type_info(JointTypeNum joint_type_num) const {
    return joint_type_info_[joint_type_num];
  }

  inline const std::string& world_body_name() const;
  BodyNum world_body_num() const { return BodyNum(0); }

  // TODO(sherm1) Make these less brittle.
  JointTypeNum weld_joint_type_num() const { return JointTypeNum(0); }
  JointTypeNum free_joint_type_num() const { return JointTypeNum(1); }

 private:
  // Only MultibodyGraphMaker can make a MultibodyGraph.
  friend class MultibodyGraphMaker;

  // Construct an empty %MultibodyGraph.
  MultibodyGraph() = default;

  // Restores this %MultibodyGraph object to its just-constructed
  // condition.
  void Clear();

  JointTypeNum AddJointType(std::string joint_type_name,
                            void* joint_type_user_ref);

  LinkNum AddLinkInfo(std::string link_name, void* link_user_ref);

  JointNum AddJointInfo(std::string joint_name, JointTypeNum joint_type_num,
                        void* joint_user_ref);

  BodyNum AddBodyFromLink(LinkNum link_num);

  MobilizerNum AddMobilizerFromJoint(JointNum joint_num,
                                     BodyNum inboard_body_num,
                                     BodyNum outboard_body_num,
                                     bool is_reversed);

  ConstraintNum AddConstraintFromJoint(JointNum joint_num,
                                       BodyNum parent_body_num,
                                       BodyNum child_body_num);

  ConstraintNum AddSlaveWeldConstraint(std::string constraint_name,
                                       JointNum joint_num,
                                       BodyNum master_body_num,
                                       BodyNum slave_body_num);

  // Adds a free or weld mobilizer.
  MobilizerNum ConnectBodyToWorld(BodyNum body_num, bool is_static);


  // Get writable access to bodies.
  Body& get_mutable_body(BodyNum body_num) { return bodies_[body_num]; }

  BodyNum SplitBody(BodyNum master_body_num);
  bool BodiesAreConnected(BodyNum body1_num, BodyNum body2_num) const {
    return BodiesAreConnectedByMobilizer(body1_num, body2_num) ||
           BodiesAreConnectedByJointConstraint(body1_num, body2_num);
  }
  bool BodiesAreConnectedByMobilizer(BodyNum body1_num,
                                     BodyNum body2_num) const;
  bool BodiesAreConnectedByJointConstraint(BodyNum body1_num,
                                           BodyNum body2_num) const;

  // Calculated by MultibodyGraphMaker::MakeGraph().
  // Index by BodyNum, MobilizerNum, ConstraintNum, resp.
  std::vector<Body> bodies_;             // world + input bodies + slaves
  std::vector<Mobilizer> mobilizers_;    // mobilized bodies
  std::vector<Constraint> constraints_;  // master/slave welds, loop joints

  // Information about links, modified as graph is built. Order and numbering
  // here are identical to links in MultibodyGraphMaker at the time this
  // graph was generated.
  std::map<std::string, BodyNum> link_name_to_body_num_;
  std::vector<LinkInfo> link_info_;  // Index by LinkNum.

  // Information about joints, modified as graph is built. Order and numbering
  // here are identical to joints in MultibodyGraphMaker at the time this
  // graph was generated. Index by JointNum.
  std::vector<JointInfo> joint_info_;

  // Information about joint types. Order and numbering here is identical to
  // joint types in MultibodyGraphMaker at the time this graph was generated.
  // Index by JointTypeNum.
  std::vector<JointTypeInfo> joint_type_info_;
};

//------------------------------------------------------------------------------
//                    MULTIBODY GRAPH :: LINK INFO
//------------------------------------------------------------------------------
/** Local class that collects information about how we modeled a given
input link. */
class MultibodyGraph::LinkInfo {
 public:
  // Move only.
  LinkInfo(const LinkInfo&) = delete;
  LinkInfo& operator=(const LinkInfo&) = delete;
  LinkInfo(LinkInfo&&) = default;
  LinkInfo& operator=(LinkInfo&&) = default;

  const std::string& link_name() const { return link_name_; }
  void* user_ref() const { return user_ref_; }
  BodyNum master_body_num() const { return master_body_num_; }

 private:
  friend class MultibodyGraph;

  LinkInfo(std::string link_name, void* user_ref)
      : link_name_(std::move(link_name)),
        user_ref_(user_ref) {}

  void set_master_body_num(BodyNum master_body_num) {
    DRAKE_DEMAND(!master_body_num_.is_valid() && master_body_num.is_valid());
    master_body_num_ = master_body_num;
  }

  // Copied from the input link.
  std::string link_name_;
  void* user_ref_{nullptr};

  // Determined during graph generation.
  BodyNum master_body_num_;
};

//------------------------------------------------------------------------------
//                   MULTIBODY GRAPH :: JOINT INFO
//------------------------------------------------------------------------------
/** Local class that collects information about how we modeled a given
input joint. */
class MultibodyGraph::JointInfo {
 public:
  // Move only.
  JointInfo(const JointInfo&) = delete;
  JointInfo& operator=(const JointInfo&) = delete;
  JointInfo(JointInfo&&) = default;
  JointInfo& operator=(JointInfo&&) = default;

  const std::string& joint_name() const { return joint_name_; }
  void* user_ref() const { return user_ref_; }
  JointTypeNum joint_type_num() const { return joint_type_num_; }

  bool has_mobilizer() const { return mobilizer_num_.is_valid(); }

  /** Returns the mobilizer that represents this joint, if any. Otherwise the
  returned value is not valid. */
  MobilizerNum mobilizer_num() const { return mobilizer_num_; }

  /** Returns the constraint that represents this joint, if any. Otherwise the
  returned value is not valid. */
  ConstraintNum constraint_num() const { return constraint_num_; }

 private:
  friend class MultibodyGraph;
  JointInfo(std::string joint_name, JointTypeNum joint_type_num, void* user_ref)
      : joint_name_(std::move(joint_name)),
        joint_type_num_(joint_type_num),
        user_ref_(user_ref) {}

  void set_mobilizer_num(MobilizerNum num) { mobilizer_num_ = num; }

  void set_constraint_num(ConstraintNum num) { constraint_num_ = num; }

  // Copied from the input joint.
  std::string joint_name_;
  JointTypeNum joint_type_num_;
  void* user_ref_{nullptr};

  // Determined during graph generation (only one of these will be valid).
  MobilizerNum mobilizer_num_;    // If modeled with a mobilizer.
  ConstraintNum constraint_num_;  // If modeled with a constraint.
};

//------------------------------------------------------------------------------
//                   MULTIBODY GRAPH :: JOINT TYPE INFO
//------------------------------------------------------------------------------
/** Local class that holds joint type information we can reference from
mobilizers and joint constraints. */
class MultibodyGraph::JointTypeInfo {
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
  in MultibodyGraphMaker. */
  void* user_ref() const { return user_ref_; }

 private:
  friend class MultibodyGraph;

  JointTypeInfo(const std::string& name, void* user_ref)
      : joint_type_name_(name), user_ref_(user_ref) {}

  std::string joint_type_name_;
  void* user_ref_{nullptr};  // Copied from the input joint type.
};

//------------------------------------------------------------------------------
//                         MULTIBODY GRAPH :: BODY
//------------------------------------------------------------------------------
/** Local class that accumulates modeling information and represents bodies as
the multibody tree is generated. */
class MultibodyGraph::Body {
 public:
  // Move only.
  Body(const Body&) = delete;
  Body& operator=(const Body&) = delete;
  Body(Body&&) = default;
  Body& operator=(Body&&) = default;

  /** Returns the number of fragments into which we had to break the
  corresponding link. Normally this is just one, meaning we didn't break the
  link, but master bodies that have slaves will return the number of slaves
  plus one. */
  int num_fragments() const { return 1 + num_slaves(); }

  /** Returns the number of slave bodies associated with this master body.
  Normally zero, but a master body with (for example) three slave bodies would
  return three. */
  int num_slaves() const { return static_cast<int>(slaves_.size()); }

  /** Returns `true` if this body is a slave body generated by breaking an
  input link into multiple bodies. If so you can find its master using
  master_body_num(). */
  bool is_slave() const { return master_body_num_.is_valid(); }

  /** Returns `true` if this is a body that represents a link that had to
  be broken into multiple bodies. If so you can find the generated slave
  bodies using slaves(). */
  bool is_master() const { return num_slaves() > 0; }

  /** Returns `true` if this is body that represents the World. */
  bool is_world_body() const { return level_ == 0; }

  const std::string& name() const { return name_; }

  /** Returns `true` if this body has already been assigned a place in
  the multibody tree. This will be true for all bodies once graph
  building is complete. */
  bool is_in_tree() const { return level_ >= 0; }

  /** Returns the level of this body in the multibody tree. If this is World
  it is level 0, if it's a base body it's level 1, if it's connected to a
  base body it's level 2, etc. The level of a body is the same as the level
  of its inboard mobilizer as obtained with mobilizer_num(). */
  int level() const { return level_; }

  /** Returns the unique mobilizer (by number) for which this body is the
  outboard body, or an invalid MobilizerNum if the multibody graph has not yet
  been generated. */
  MobilizerNum mobilizer_num() const { return mobilizer_num_; }

  /** Returns the number of mobilizers for which this body is the inboard
  body. */
  int num_outboard_mobilizers() const {
    return static_cast<int>(outboard_mobilizers().size());
  }

  /** Returns a reference to a vector of MobilizerNum values containing all
  the mobilizers for which this body is the inboard body. */
  const std::vector<MobilizerNum>& outboard_mobilizers() const {
    return outboard_mobilizers_;
  }

  int num_joint_constraints() const {
    return static_cast<int>(joint_constraints().size());
  }

  const std::vector<ConstraintNum>& joint_constraints() const {
    return joint_constraints_;
  }

  /** Returns the list of slave bodies (by body number), if this is a
  master body. */
  const std::vector<BodyNum>& slaves() const { return slaves_; }

  /** Returns the master body number if this is a slave body, otherwise
  returns an invalid BodyNum. */
  BodyNum master_body_num() const { return master_body_num_; }

  /** Returns the link number of the link modeled by this body. Master
  and slave bodies both return the same link number. */
  LinkNum link_num() const { return link_num_; }

 private:
  // Restrict construction and modification to only MultibodyGraph.
  friend class MultibodyGraph;

  // Create a body that directly models a link.
  Body(LinkNum link_num, MultibodyGraph* graph)
      : name_(graph->link_info(link_num).link_name()),
        link_num_(link_num),
        graph_(graph) {}

  // Create a slave body for the given master.
  Body(std::string name, BodyNum master_body_num, MultibodyGraph* graph)
      : name_(std::move(name)),
        link_num_(graph->body(master_body_num).link_num()),
        master_body_num_(master_body_num),
        graph_(graph) {}

  void set_level(int level) { level_ = level; }
  void set_mobilizer_num(MobilizerNum mobilizer_num) {
    mobilizer_num_ = mobilizer_num;
  }

  // When this body serves as the inboard body for a mobilizer, note that.
  void add_outboard_mobilizer_num(MobilizerNum outboard_mobilizer_num) {
    outboard_mobilizers_.push_back(outboard_mobilizer_num);
  }

  // When this body participates in a joint constraint, note that.
  void add_joint_constraint_num(ConstraintNum joint_constraint_num) {
    joint_constraints_.push_back(joint_constraint_num);
  }

  // Records the master body number for this slave body.
  void set_master_body_num(BodyNum master_body_num) {
    master_body_num_ = master_body_num;
  }

  // Records a slave body number for this master body.
  void add_slave_body_num(BodyNum slave_body_num) {
    slaves_.push_back(slave_body_num);
  }

  std::string name_;  // Link name if this was an input link.
  LinkNum link_num_;  // Original link number if any.

  // Disposition of this body in the spanning tree.

  int level_{-1};               // Distance from World in the tree.
  MobilizerNum mobilizer_num_;  // The unique inboard mobilizer.
  std::vector<MobilizerNum> outboard_mobilizers_;

  // Joint constraints in which this body participates.
  std::vector<ConstraintNum> joint_constraints_;

  BodyNum master_body_num_;      // valid if this is a slave.
  std::vector<BodyNum> slaves_;  // Slave links, if this is a master.

  MultibodyGraph* const graph_{nullptr};  // just a reference to container
};

//------------------------------------------------------------------------------
//                   MULTIBODY GRAPH :: MOBILIZER
//------------------------------------------------------------------------------
/** Local class that represents one of the mobilizers (tree joints) in the
generated spanning tree. There is always a corresponding joint, although that
joint might be a world-to-link free joint that was added automatically. */
class MultibodyGraph::Mobilizer {
 public:
  // Move only.
  Mobilizer(const Mobilizer&) = delete;
  Mobilizer& operator=(const Mobilizer&) = delete;
  Mobilizer(Mobilizer&&) = default;
  Mobilizer& operator=(Mobilizer&&) = default;

  /** Returns the mobilizer's name. This will be the input joint's name if this
  mobilizer models one of the input joints. Otherwise, it will have a name
  constructed from the inboard and output body names. */
  const std::string& name() const { return name_; }

  /** Returns the joint associated with this mobilizer. There will always be
  one, but it may have been an added joint rather than an input joint. */
  JointNum joint_num() const { return joint_num_; }

  /** Returns the inboard body number of this mobilizer. This is the parent
  body of the associated joint unless the mobilizer is reversed. */
  BodyNum inboard_body_num() const { return inboard_body_num_; }

  /** Returns the outboard body number of this mobilizer. This is the child
  body of the associated joint unless the mobilizer is reversed. */
  BodyNum outboard_body_num() const { return outboard_body_num_; }

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
  bool is_added_base_mobilizer() const { return !joint_num_.is_valid(); }

  /** Get the mobilizer type name. These are mapped from joint type names. */
  const std::string& get_joint_type_name() const {
    return graph_->joint_type_info(mobilizer_type_num_).mobilizer_type_name();
  }

  /** Return true if the outboard body of this mobilizer is a slave we
  created in order to cut a loop, rather than one of the input bodies. */
  bool is_slave_mobilizer() const {
    return graph_->body(outboard_body_num_).is_slave();
  }

  /** Return the number of fragments into which we chopped the outboard body
  of this mobilizer. There is one fragment for the master body plus however
  many slaves of that body were created. Thus you should divide the master
  body's mass properties by this number to obtain the mass and inertia to be
  assigned to each of the body fragments. */
  int num_fragments() const {
    return graph_->body(outboard_master_body_num()).num_fragments();
  }

  /** Returns the body number of the master body for this mobilizer's outboard
  body, which might be a slave. If the outboard body is not a slave body, then
  this returns the same value as outboard_body_num(). */
  BodyNum outboard_master_body_num() const {
    const Body& outboard_body = graph_->body(outboard_body_num_);
    return outboard_body.is_slave() ? outboard_body.master_body_num()
                                    : outboard_body_num_;
  }

 private:
  // Restrict construction to only MultibodyGraph.
  friend class MultibodyGraph;

  // Construct a mobilizer that directly represents an input joint.
  Mobilizer(JointNum joint_num, int level, BodyNum inboard_body_num,
            BodyNum outboard_body_num, bool is_reversed, MultibodyGraph* graph)
      : joint_num_(joint_num),
        is_reversed_(is_reversed),
        level_(level),
        inboard_body_num_(inboard_body_num),
        outboard_body_num_(outboard_body_num),
        graph_(graph) {
    DRAKE_DEMAND(joint_num_.is_valid());
    DRAKE_DEMAND(level_ >= 0);
    DRAKE_DEMAND(inboard_body_num_.is_valid());
    DRAKE_DEMAND(outboard_body_num_.is_valid());
    DRAKE_DEMAND(graph_ != nullptr);

    name_ = graph->joint_info(joint_num).joint_name();
    mobilizer_type_num_ = graph->joint_info(joint_num).joint_type_num();
  }

  // Construct a mobilizer for which there is no input joint counterpart.
  Mobilizer(std::string name, JointTypeNum joint_type_num, int level,
            BodyNum inboard_body_num, BodyNum outboard_body_num,
            MultibodyGraph* graph)
      : name_(std::move(name)),
        mobilizer_type_num_(joint_type_num),
        level_(level),
        inboard_body_num_(inboard_body_num),
        outboard_body_num_(outboard_body_num),
        graph_(graph) {
    DRAKE_DEMAND(mobilizer_type_num_.is_valid());
    DRAKE_DEMAND(level_ >= 0);
    DRAKE_DEMAND(inboard_body_num_.is_valid());
    DRAKE_DEMAND(outboard_body_num_.is_valid());
    DRAKE_DEMAND(graph_ != nullptr);
  }

  std::string name_;  // Will be input joint name if possible.
  JointTypeNum mobilizer_type_num_;

  // If this mobilizer directly models an input joint, note the joint number
  // and whether the mobilizer is reversed s.t. inboard=child, outboard=parent.
  JointNum joint_num_;
  bool is_reversed_{false};

  int level_{-1};              // level of outboard body; distance from World
  BodyNum inboard_body_num_;   // might be World
  BodyNum outboard_body_num_;  // might be a slave body; can't be World

  MultibodyGraph* const graph_{nullptr};  // just a reference to container
};

//------------------------------------------------------------------------------
//                      MULTIBODY GRAPH :: CONSTRAINT
//------------------------------------------------------------------------------
/** Local class that represents one of the constraints that were added to close
topological loops that were cut to form the spanning tree. */
class MultibodyGraph::Constraint {
 public:
  const std::string& name() const { return name_; }

  /** Get the loop constraint type name of the constraint that should be
  used here. This will be either the type name of the associated joint, or the
  type name of a weld joint if this is a master/slave weld. */
  const std::string& constraint_type_name() const {
    return graph_->joint_type_info(constraint_type_num_).constraint_type_name();
  }

  /** Returns the parent body number from the modeled input joint, or the
  master body number if this is a master/slave weld. */
  BodyNum parent_body_num() const { return parent_body_num_; }

  /** Returns the child body number from the modeled input joint, or the
  slave body number if this is a master/slave weld. */
  BodyNum child_body_num() const { return child_body_num_; }

  /** Returns the joint number if this loop constraint models one of the input
  joints, otherwise returns an invalid JointNum. */
  JointNum joint_num() const { return joint_num_; }

 private:
  // Restrict construction to only MultibodyGraph.
  friend class MultibodyGraph;

  // Construct a constraint that directly represents an input joint.
  Constraint(JointNum joint_num, BodyNum parent_body_num,
                 BodyNum child_body_num, MultibodyGraph* graph)
      : joint_num_(joint_num),
        parent_body_num_(parent_body_num),
        child_body_num_(child_body_num),
        graph_(graph) {
    DRAKE_DEMAND(joint_num.is_valid());
    DRAKE_DEMAND(parent_body_num.is_valid() && child_body_num.is_valid());
    DRAKE_DEMAND(graph != nullptr);

    name_ = graph->joint_info(joint_num).joint_name();
    constraint_type_num_ = graph->joint_info(joint_num).joint_type_num();
  }

  // Construct a constraint that does not directly represent an input joint,
  // but may be associated with one.
  Constraint(std::string name, JointTypeNum joint_type_num,
                 JointNum joint_num, BodyNum parent_body_num,
                 BodyNum child_body_num, MultibodyGraph* graph)
      : name_(std::move(name)),
        constraint_type_num_(joint_type_num),
        joint_num_(joint_num),
        parent_body_num_(parent_body_num),
        child_body_num_(child_body_num),
        graph_(graph) {
    DRAKE_DEMAND(joint_type_num.is_valid());
    DRAKE_DEMAND(parent_body_num.is_valid() && child_body_num.is_valid());
    DRAKE_DEMAND(graph_ != nullptr);
  }

  std::string name_;  // Will be input joint name if possible.
  JointTypeNum constraint_type_num_;

  JointNum joint_num_;  // set if one of the input joints

  BodyNum parent_body_num_;  // parent from the joint, or master body
  BodyNum child_body_num_;   // child from the joint, or slave body

  MultibodyGraph* graph_{nullptr};  // just a reference to container
};

inline auto MultibodyGraph::link_to_body_num(LinkNum link_num) const
    -> BodyNum {
  const LinkInfo& link_info = this->link_info(link_num);
  return link_info.master_body_num();
}

inline auto MultibodyGraph::body_to_link_num(BodyNum body_num) const
    -> LinkNum {
  const Body& body = this->body(body_num);
  return body.link_num();
}

inline const std::string& MultibodyGraph::world_body_name() const {
  DRAKE_DEMAND(!bodies_.empty());  // World should always be here.
  return body(BodyNum(0)).name();
}

inline MultibodyGraph::LinkFlags operator|(MultibodyGraph::LinkFlags left,
                                           MultibodyGraph::LinkFlags right) {
  return static_cast<MultibodyGraph::LinkFlags>(static_cast<unsigned>(left) |
                                                static_cast<unsigned>(right));
}

inline MultibodyGraph::LinkFlags operator&(MultibodyGraph::LinkFlags left,
                                           MultibodyGraph::LinkFlags right) {
  return static_cast<MultibodyGraph::LinkFlags>(static_cast<unsigned>(left) &
      static_cast<unsigned>(right));
}

std::string to_string(MultibodyGraph::LinkFlags);
std::string to_string(MultibodyGraph::JointFlags);
std::string to_string(MultibodyGraph::JointTypeFlags);

}  // namespace multibody
}  // namespace drake
