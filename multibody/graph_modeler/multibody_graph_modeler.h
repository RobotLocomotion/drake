#pragma once

/* Adapted for Drake from Simbody's MultibodyGraphMaker class.
Portions copyright (c) 2013-14 Stanford University and the Authors.
Authors: Michael Sherman
Contributors: Kevin He

Modifications copyright (c) 2019-20 Toyota Research Institute, Inc.

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

/** @file
Declares the MultibodyGraphModeler class for use in representing a multibody
system as input by a user in terms of bodies and joints. */

#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

// TODO(sherm1) Make these multibody index types.
using JointTypeIndex = TypeSafeIndex<class JointTypeNumTag>;

// TODO(sherm1) Move these flags to MultibodyTreeModel and don't expose
// them here?

/** Boolean properties of a body that can affect the resulting tree model.
These may be or-ed together and the result type is still BodyFlags. */
enum BodyFlags : unsigned {
  kDefaultBodyFlags = 0b0000,
  kStaticBody = 0b0001,
  kMustBeBaseBody = 0b0010,
  kMustNotBeTerminalBody = 0b0100,
};

inline BodyFlags operator|(BodyFlags left, BodyFlags right) {
  return static_cast<BodyFlags>(static_cast<unsigned>(left) |
                                static_cast<unsigned>(right));
}

inline BodyFlags operator&(BodyFlags left, BodyFlags right) {
  return static_cast<BodyFlags>(static_cast<unsigned>(left) &
                                static_cast<unsigned>(right));
}

/** Boolean properties of a joint that can affect the resulting tree model. */
enum JointFlags : unsigned {
  kDefaultJointFlags = 0b0000,
  kMustBeConstraint = 0b0001,
};

/** Boolean properties of a joint _type_ that can affect the resulting
tree model. */
enum JointTypeFlags : unsigned {
  kDefaultJointTypeFlags = 0b0000,
  kOkToUseAsJointConstraint = 0b0001,
};

std::string to_string(BodyFlags);
std::string to_string(JointFlags);
std::string to_string(JointTypeFlags);

//==============================================================================
//                          MULTIBODY GRAPH MODELER
//==============================================================================
/** Defines a multibody graph consisting of bodies interconnected by joints,
with semantics identical to `.sdf` or `.urdf` files. This is not the
computational model used by Drake; see MultibodyTreeModel for information about
generating a computational model from a MultibodyGraph.

The first body added (body 0) is assumed to be World and its name is used for
recognizing World connections later in joints. If you accept the Drake defaults,
that body will be named "world". The default names for Weld (0 dof) and Free
(6 dof) joints are, not surprisingly, "weld" and "free" but you can change them.
*/
class MultibodyGraphModeler {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyGraphModeler)

  // Local classes.
  class Body;
  class Joint;
  class JointType;

  /** Construct an empty %MultibodyGraphModeler object and set the default
  names for Weld and Free joints to "weld" and "free". Also sets up Drake's
  World body (named "world") and supported joint types, unless
  `use_drake_defaults` is set `false`. In that case the first added body (body
  number 0) is the World body and its name is used to represent World. */
  explicit MultibodyGraphModeler(bool use_drake_defaults = true);

  /** Restores this %MultibodyGraphModeler object to its just-constructed
  condition. See the constructor documentation for the meaning of the
  parameter. */
  void Clear(bool use_drake_defaults = true);

  /** @name         Define the input graph (bodies and joints)
  Methods in this section are used to inform %MultibodyGraphModeler about the
  bodies and joints as given, typically in an .sdf or .urdf file. You
  can also inspect the set of bodies and joints, and remove bodies and joints
  from an existing specification. */
  //@{

  /** Add a new body to the set of input bodies.
  @param[in]      body_name
      A unique string identifying this body. There are no other restrictions
      on the contents of `name`. If this %MultibodyGraphModeler was constructed
      without Drake defaults, the first body you add is considered to be
      World, and its name is remembered and recognized when used in joints.
  @param[in]      model_instance
      The model instance to which this body belongs. If none is specified then
      the body belongs to the default model instance. This is simply a tag that
      is carried with the body and bodies generated from it; it has no effect
      on the structure of the generated tree model. However, it does permit
      queries that return only elements of a particular model instance.
  @param[in]      flags
      Optional flags affecting the generated tree model. Flags can be used to
      designate a body as static, to require that it not be used as a terminal
      mobilized body in the graph (because it is effectively massless), or to
      require that the resulting mobilized body is a base body (that is,
      connected directly to World).
  @param[in]      user_ref
      A generic user reference pointer that is kept with the associated
      body and can be used by the caller to map back to their own data
      structure containing additional body information.
  @see DeleteBody() */
  void AddBody(std::string name, ModelInstanceIndex model_instance,
               BodyFlags flags = kDefaultBodyFlags, void* user_ref = nullptr);

  /** Convenience overload that puts the body into the default model
  instance. */
  void AddBody(const std::string& name, BodyFlags flags = kDefaultBodyFlags,
               void* user_ref = nullptr) {
    AddBody(name, default_model_instance(), flags, user_ref);
  }

  /** Delete a body from the set of input bodies. All the joints that
  reference this body will be deleted too. Body numbers and joint numbers will
  have changed after this call.
  @param[in]      body_num
      The body index of the body to delete. Note that other bodies may get
      renumbered after a deletion.
  @returns `true` if the body is successfully deleted, `false` if it
      didn't exist. */
  bool DeleteBody(BodyIndex body_num);

  /** Delete a body given by just its name, which must be unique among all
  model instances. */
  bool DeleteBody(const std::string& body_name) {
    return DeleteBody(FindBodyIndex(body_name));
  }

  /** Add a new joint to the set of input joints.
  @param[in]      name
      A string uniquely identifying this joint within its model instance. There
      are no other restrictions on the contents of `name`.
  @param[in]      model_instance
      The model instance to which this joint belongs. If none is specified then
      the joint belongs to the default model instance. This is simply a tag that
      is carried with the joint and mobilizers generated from it; it has no
      effect on the structure of the generated tree model. However, it does
      permit queries that return only elements of a particular model instance.
  @param[in]      type
      A string designating the type of this joint, such as "revolute" or
      "ball". This must be chosen from the set of joint types previously
      registered.
  @param[in]      parent_body_num
      This must be the index of a body that was already specified in an earlier
      AddBody() call, including the World body which is always body 0. If
      possible, this will be used as the inboard mobilized body for the
      corresponding mobilizer in the tree model.
  @param[in]      child_body_num
      This must be the index of a body that was already specified in an earlier
      AddBody() call, including the World body which is always body 0. It must
      be distinct from `parent_body_num`. If possible, this will be used as the
      outboard mobilized body for the corresponding mobilizer.
  @param[in]      flags
      If you feel strongly that this joint should be implemented using
      a constraint rather than a mobilizer (not common), provide the flag
      kMustBeConstraint. In that case the joint will not appear in the list of
      joints that are candidates to be modeled as mobilizers in a multibody
      tree. See MultibodyTreeModel for details.
  @param[in]      user_ref
      This is a generic user reference pointer that is kept with the joint
      and can be used by the caller to map back to their own data
      structure containing additional joint information.
  @see DeleteJoint() */
  void AddJoint(const std::string& name, ModelInstanceIndex model_instance,
                const std::string& type, BodyIndex parent_body_num,
                BodyIndex child_body_num, JointFlags flags = kDefaultJointFlags,
                void* user_ref = nullptr);

  /** Convenience overload that puts the joint into the default model
  instance. */
  void AddJoint(const std::string& name, const std::string& type,
                BodyIndex parent_body_num, BodyIndex child_body_num,
                JointFlags flags = kDefaultJointFlags,
                void* user_ref = nullptr) {
    AddJoint(name, default_model_instance(), type, parent_body_num,
             child_body_num, flags, user_ref);
  }

  /** Convenience overload that accepts names for the parent and child bodies.
  This can only be used if those names are unique across all model instances. */
  void AddJoint(const std::string& name, const std::string& type,
                const std::string& parent_body_name,
                const std::string& child_body_name,
                JointFlags flags = kDefaultJointFlags,
                void* user_ref = nullptr) {
    AddJoint(name, default_model_instance(), type,
             FindBodyIndex(parent_body_name), FindBodyIndex(child_body_name),
             flags, user_ref);
  }

  /** Delete an existing joint from the set of input joints. The bodies
  referenced by the joint are expected to exist and their references to this
  joint will be removed as well. Joint numbers will have changed after this
  call.
  @param[in]      joint_num
      The index of the joint to delete. Note that other joints may get
      renumbered after a deletion.
  @returns `true` if the joint is successfully deleted, `false` if it
      didn't exist. */
  bool DeleteJoint(JointIndex joint_num);

  /** Delete a body given by just its name, which must be unique among all
  model instances. */
  bool DeleteJoint(const std::string& joint_name) {
    return DeleteJoint(FindJointIndex(joint_name));
  }

  /** (Advanced) Change the flags for an existing body. May cause a different
  graph to be generated on the next call to MakeTreeModel(). This method
  exists primarily for testing purposes; normally the flags provided when the
  body is added remain unchanged. */
  void ChangeBodyFlags(BodyIndex body_num, BodyFlags flags);

  /** (Advanced) Convenience overload for changing flags for a body given by
  just its name, which must be unique among all model instances. */
  void ChangeBodyFlags(const std::string& body_name, BodyFlags flags) {
    ChangeBodyFlags(FindBodyIndex(body_name), flags);
  }

  /** Returns the name we recognize as the World (or Ground) body. This is
  the name that was provided in the first AddBody() call. */
  const std::string& world_body_name() const;

  /** Returns the body number for the World body (always zero). */
  BodyIndex world_body_num() const { return BodyIndex(0); }

  /** For debugging, write a human-readable representation of the input graph
  to the given output stream. */
  void DumpInput(std::ostream& output_stream) const;
  //@}

  /** @name             Inspect the bodies and joints
  Methods in this section allow introspection of the input objects. */
  //@{

  /** Returns the number of bodies, including all input bodies, and a World
  body. */
  int num_bodies() const { return static_cast<int>(bodies_.size()); }

  /** Gets a Body object by its assigned number. These are assigned first to
  World (body 0) and then input bodies. */
  const Body& get_body(BodyIndex body_num) const {
    DRAKE_ASSERT(has_body(body_num));
    return bodies_[body_num];
  }

  /** Returns true if the given BodyIndex corresponds to a Body.
  @pre `body_num` must not be invalid (that is, it can't be uninitialized). */
  bool has_body(BodyIndex body_num) const {
    DRAKE_DEMAND(body_num.is_valid());
    return body_num < num_bodies();
  }

  /** Returns true iff there is a body by this name in any model instance. Note
  that there may be more than one. */
  bool HasBody(const std::string& body_name) const {
    auto instance_map = GetAllBodyIndexFromName(body_name);
    return instance_map != nullptr;  // At least one body found.
  }

  /** Returns true iff this body name appears in exactly one model instance. */
  bool IsUniqueBodyName(const std::string& body_name) const {
    auto instance_map = GetAllBodyIndexFromName(body_name);
    return instance_map && instance_map->size() == 1;
  }

  /** Returns the body number assigned to the input body with the given name
  in the given model instance. Throws std::logic_error if the name is not found
  in that model instance. */
  BodyIndex FindBodyIndex(const std::string& name,
                          ModelInstanceIndex model_instance) const {
    DRAKE_DEMAND(!name.empty() && model_instance.is_valid());
    return GetBodyIndexFromNameAndInstance(name, model_instance, __func__);
  }

  /** Returns the body number assigned to the input body identified by name
  only (no model instance). The name must be globally unique. Throws
  std::logic_error if the name is not recognized or not unique. */
  BodyIndex FindBodyIndex(const std::string& name) const {
    return GetUniqueBodyIndexFromName(name, __func__);
  }

  /** Returns the number of input joints. */
  int num_joints() const { return static_cast<int>(joints_.size()); }

  /** Gets a Joint object by its assigned number. */
  const Joint& get_joint(JointIndex joint_num) const {
    DRAKE_ASSERT(has_joint(joint_num));
    return joints_[joint_num];
  }

  /** Returns true if the given JointIndex corresponds to a Joint.
  @pre `joint_num` must not be invalid (that is, it can't be uninitialized). */
  bool has_joint(JointIndex joint_num) const {
    DRAKE_DEMAND(joint_num.is_valid());
    return joint_num < num_joints();
  }

  /** Returns true iff there is a joint by this name in any model instance. Note
  that there may be more than one. */
  bool HasJoint(const std::string& joint_name) const {
    auto instance_map = GetAllJointIndexFromName(joint_name);
    return instance_map != nullptr;  // At least one joint found.
  }

  /** Returns true iff this joint name appears in exactly one model instance. */
  bool IsUniqueJointName(const std::string& joint_name) const {
    auto instance_map = GetAllJointIndexFromName(joint_name);
    return instance_map && instance_map->size() == 1;
  }

  /** Returns the joint number assigned to the input joint with the given name
  in the given model instance. Throws std::logic_error if the name is not found
  in that model instance. */
  JointIndex FindJointIndex(const std::string& name,
                            ModelInstanceIndex model_instance) const {
    return GetJointIndexFromNameAndInstance(name, model_instance, __func__);
  }

  /** Returns the joint number assigned to the input joint identified by name
  only (no model instance). The name must be globally unique. Throws
  std::logic_error if the name is not recognized or not unique.  */
  JointIndex FindJointIndex(const std::string& name) const {
    return GetUniqueJointIndexFromName(name, __func__);
  }

  /** Returns true if there is any joint _directly_ connecting these two
  bodies. */
  bool BodiesAreConnected(BodyIndex body1_num, BodyIndex body2_num) const;
  //@}

  /** @name         Specify and inspect available joint types
  Use the methods in this section to inform %MultibodyGraphModeler about the
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
      There is no model instance associated with a joint type.
  @param[in]      num_q
      The number of generalized coordinates (positions) to be included in the
      model for this joint type. Must be greater than or equal to `num_v`.
  @param[in]      num_v
      The number of generalized velocities (degrees of freedom) to be included
      in the model for this joint type. Must be less than or equal to `num_q`.
  @param[in]      flags
      You can set this to `kOkToUseAsJointConstraint` which
      means that, in addition to a mobilizer of this type, your multibody code
      has available a constraint-based joint that is just as good. If so
      MultibodyTreeModel will attempt to use that constraint rather than
      cutting a body to break a loop. Typically used only for ball (spherical)
      joints.
  @param[in]      user_ref
      This is a generic user reference pointer that is kept with the joint type
      and can be used by the caller to map back to their own data
      structure containing more joint type information.
  @retval JointTypeNum Small integer joint type number used for referencing. */
  JointTypeIndex RegisterJointType(
      const std::string& name, int num_q, int num_v,
      JointTypeFlags flags = kDefaultJointTypeFlags, void* user_ref = nullptr);

  /** Change the name to be used to identify the weld joint type (0 dof) and
  weld loop constraint type (6 constraints). */
  void SetWeldJointTypeName(const std::string& name) { weld_type_name_ = name; }

  /** Changes the name to be used to identify the free (6 dof) joint type and
  free (0 constraints) loop constraint type. */
  void SetFreeJointTypeName(const std::string& name) { free_type_name_ = name; }

  /** Returns the number of registered joint types. */
  int num_joint_types() const { return static_cast<int>(joint_types_.size()); }

  /** Get a JointType object by its assigned number. */
  const JointType& get_joint_type(JointTypeIndex joint_type_num) const {
    DRAKE_ASSERT(joint_type_num.is_valid() &&
                 joint_type_num < num_joint_types());
    return joint_types_[joint_type_num];
  }

  /** Finds the assigned number for a joint type from the type name. */
  JointTypeIndex FindJointTypeNum(const std::string& joint_type_name) const {
    std::map<std::string, JointTypeIndex>::const_iterator p =
        joint_type_name_to_num_.find(joint_type_name);
    return p == joint_type_name_to_num_.end() ? JointTypeIndex() : p->second;
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
  Body& get_mutable_body(BodyIndex body_num) {
    DRAKE_DEMAND(has_body(body_num));
    return bodies_[body_num];
  }

  Joint& get_mutable_joint(JointIndex joint_num) {
    DRAKE_DEMAND(has_joint(joint_num));
    return joints_[joint_num];
  }

  // These maps are used to collect together all the bodies or joints with
  // the same name, but in different model instances.
  using InstanceBodyIndexMap = std::map<ModelInstanceIndex, BodyIndex>;
  using InstanceJointIndexMap = std::map<ModelInstanceIndex, JointIndex>;

  // Returns the model_instance->body_num map for all bodies with this name,
  // if any, otherwise nullptr.
  const InstanceBodyIndexMap* GetAllBodyIndexFromName(
      const std::string& body_name) const;

  // Returns the body_num for the a body name in a model instance.
  BodyIndex GetBodyIndexFromNameAndInstance(const std::string& body_name,
                                            ModelInstanceIndex model_instance,
                                            const char* func) const;

  // Returns the body_num for this body name, which must appear in only one
  // model instance.
  BodyIndex GetUniqueBodyIndexFromName(const std::string& name,
                                       const char* func) const;

  // Returns the model_instance->joint_num map for all joints with this name,
  // if any, otherwise nullptr.
  const InstanceJointIndexMap* GetAllJointIndexFromName(
      const std::string& joint_name) const;

  // Returns the JointIndex for the a joint name in a model instance.
  JointIndex GetJointIndexFromNameAndInstance(const std::string& joint_name,
                                              ModelInstanceIndex model_instance,
                                              const char* func) const;

  // Returns the JointIndex for this joint name, which must appear in only one
  // model instance.
  JointIndex GetUniqueJointIndexFromName(const std::string& joint_name,
                                         const char* func) const;

  // Clear everything except for default names.
  void ClearContainers() {
    joint_types_.clear();
    joint_type_name_to_num_.clear();
    bodies_.clear();
    body_name_to_num_.clear();
    joints_.clear();
    joint_name_to_num_.clear();
  }

  std::string weld_type_name_, free_type_name_;
  std::vector<JointType> joint_types_;
  std::map<std::string, JointTypeIndex> joint_type_name_to_num_;

  std::vector<Body> bodies_;  // world + input bodies
  std::map<std::string, InstanceBodyIndexMap> body_name_to_num_;

  std::vector<Joint> joints_;  // input joints
  std::map<std::string, InstanceJointIndexMap> joint_name_to_num_;
};

//------------------------------------------------------------------------------
//                      MULTIBODY GRAPH MODELER :: BODY
//------------------------------------------------------------------------------
/** Local class that collects information about bodies in the input. */
class MultibodyGraphModeler::Body {
 public:
  /** Returns the number of joints that specify this body as their parent
  body. This refers to the input parent/child designation _not_ necessarily
  the inboard/outboard relationship in the generated tree. */
  int num_children() const {
    return static_cast<int>(joints_as_parent_.size());
  }

  /** Returns the number of joints that specify this body as their child
  body. This refers to the input parent/child designation _not_ necessarily
  the inboard/outboard relationship in the generated tree. */
  int num_parents() const { return static_cast<int>(joints_as_child_.size()); }

  /** Returns the total number of joints that specify this body as either
  parent or child (this is num_children() + num_parents()). */
  int num_joints() const { return num_children() + num_parents(); }

  const std::string& name() const { return name_; }
  ModelInstanceIndex model_instance() const { return model_instance_; }
  BodyFlags flags() const { return flags_; }
  bool must_be_nonterminal() const { return flags_ & kMustNotBeTerminalBody; }
  bool is_static() const { return flags_ & kStaticBody; }
  bool must_be_base_body() const { return flags_ & kMustBeBaseBody; }
  void* user_ref() const { return user_ref_; }

  /** Returns the list of joints (by joint number) for which this body is
  the child body. */
  const std::vector<JointIndex>& joints_as_child() const {
    return joints_as_child_;
  }

  /** Returns the list of joints (by joint number) for which this body is
  the parent body. */
  const std::vector<JointIndex>& joints_as_parent() const {
    return joints_as_parent_;
  }

  /** Changes the flags for this body. */
  void set_flags(BodyFlags flags) { flags_ = flags; }

 private:
  // Restrict construction and modification to only MultibodyGraphModeler.
  friend class MultibodyGraphModeler;

  Body(const std::string& name, ModelInstanceIndex model_instance,
       BodyFlags flags, void* user_ref)
      : name_(name),
        model_instance_(model_instance),
        flags_(flags),
        user_ref_(user_ref) {
    DRAKE_DEMAND(!name.empty() && model_instance.is_valid());
  }

  // Notes that this body serves as the parent body for the given joint.
  void add_joint_as_parent(JointIndex joint_num) {
    joints_as_parent_.push_back(joint_num);
  }

  // Notes that this body serves as the child body for the given joint.
  void add_joint_as_child(JointIndex joint_num) {
    joints_as_child_.push_back(joint_num);
  }

  // Returns mutable access to the list of joints for which this body is
  // the child body.
  std::vector<JointIndex>& mutable_joints_as_child() {
    return joints_as_child_;
  }

  // Returns mutable access to the list of joints for which this body is
  // the parent body.
  std::vector<JointIndex>& mutable_joints_as_parent() {
    return joints_as_parent_;
  }

  // Inputs
  std::string name_;
  ModelInstanceIndex model_instance_;
  BodyFlags flags_{kDefaultBodyFlags};
  void* user_ref_;

  // How this body appears in joints (input and added).
  std::vector<JointIndex> joints_as_child_;   // where this body is the child
  std::vector<JointIndex> joints_as_parent_;  // where this body is the parent
};

//------------------------------------------------------------------------------
//                      MULTIBODY GRAPH MODELER :: JOINT
//------------------------------------------------------------------------------
/** Local class that collects information about input joints. */
class MultibodyGraphModeler::Joint {
 public:
  const std::string& name() const { return name_; }
  ModelInstanceIndex model_instance() const { return model_instance_; }
  JointFlags flags() const { return flags_; }
  bool must_be_constraint() const { return flags_ & kMustBeConstraint; }
  void* user_ref() const { return user_ref_; }

  BodyIndex parent_body_num() const { return parent_body_num_; }
  BodyIndex child_body_num() const { return child_body_num_; }
  JointTypeIndex joint_type_num() const { return joint_type_num_; }

 private:
  // Restrict construction and modification to only MultibodyGraphModeler.
  friend class MultibodyGraphModeler;

  Joint(const std::string& name, ModelInstanceIndex model_instance,
        JointTypeIndex joint_type_num, BodyIndex parent_body_num,
        BodyIndex child_body_num, JointFlags flags, void* user_ref)
      : name_(name),
        model_instance_(model_instance),
        joint_type_num_(joint_type_num),
        flags_(flags),
        user_ref_(user_ref),
        parent_body_num_(parent_body_num),
        child_body_num_(child_body_num) {
    DRAKE_DEMAND(!name.empty() && model_instance.is_valid());
  }

  void set_parent_body_num(BodyIndex parent_body_num) {
    parent_body_num_ = parent_body_num;
  }
  void set_child_body_num(BodyIndex child_body_num) {
    child_body_num_ = child_body_num;
  }

  // Inputs
  std::string name_;
  ModelInstanceIndex model_instance_;
  JointTypeIndex joint_type_num_;
  JointFlags flags_;
  void* user_ref_{nullptr};

  BodyIndex parent_body_num_;
  BodyIndex child_body_num_;
};

//------------------------------------------------------------------------------
//                   MULTIBODY GRAPH MODELER :: JOINT TYPE
//------------------------------------------------------------------------------
/** Local class that defines the properties of a known joint type. */
class MultibodyGraphModeler::JointType {
 public:
  const std::string& name() const { return name_; }
  int num_q() const { return num_q_; }
  int num_v() const { return num_v_; }
  bool have_good_joint_constraint_available() const {
    return flags_ & kOkToUseAsJointConstraint;
  }
  void* user_ref() const { return user_ref_; }

 private:
  // Restrict construction to only MultibodyGraphModeler.
  friend class MultibodyGraphModeler;

  JointType(const std::string& name, int num_q, int num_v, JointTypeFlags flags,
            void* user_ref)
      : name_(name),
        num_q_(num_q),
        num_v_(num_v),
        flags_(flags),
        user_ref_(user_ref) {
    DRAKE_DEMAND(!name.empty());
  }

  std::string name_;
  int num_q_{-1};
  int num_v_{-1};
  JointTypeFlags flags_{kDefaultJointTypeFlags};
  void* user_ref_{nullptr};
};

}  // namespace multibody
}  // namespace drake
