#pragma once

#include <bitset>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

static const int MAX_NUM_COLLISION_FILTER_GROUPS = 128;


// forward declaration
template <typename U> class RigidBody;

namespace DrakeCollision {

typedef std::bitset<MAX_NUM_COLLISION_FILTER_GROUPS> bitmask;

// Constants
// The empty bit mask.  Used in the membership group mask represents no
// membership.  As the ignore mask, the body ignores nothing.
extern const bitmask NONE_MASK;
// The membership bit mask indicating the CFG to which *all* bodies belong. A
// body can be made invisible (from a collision perspective) by having setting
// its ignore mask to DEFAULT_GROUP.
extern const bitmask DEFAULT_GROUP;

/**
 The specification of a collision filter group: its name, bodies that belong
 to it, and the names of collision filter groups that it ignores.  This
 class is used for initialization and not run-time calculations.

 By definition, a collision filter group can only apply to bodies within a
 single model instance.  This class tracks the identifier of the model it
 works with.

 @tparam  T  A valid Eigen scalar type.
 */
template <typename T>
class CollisionFilterGroup {
 public:
  /**
   Default constructor for use with unordered_map.
   */
  CollisionFilterGroup();

  /**
   @param name          The name for the collision filter group.
   @param id            The bit id for this collision filter group.
   */
  CollisionFilterGroup(const std::string& name, int id);

  int get_mask_id() const { return mask_id_; }

  void add_body(const RigidBody<T>& body) { bodies_.push_back(&body); }

  std::vector<const RigidBody<T>*> get_bodies() { return bodies_; }

  const std::vector<std::string>& get_ignore_groups() const {
    return ignore_groups_;
  }

  void add_ignore_group(const std::string& group_name) {
    ignore_groups_.push_back(group_name);
  }

 private:
  std::string name_{};
  int mask_id_{};
  std::vector<const RigidBody<T>*> bodies_{};
  std::vector<std::string> ignore_groups_{};
};

/**
 This class provides management utilities for the collision filter groups for
 RigidBodyTree instances.

 The intent of the manager is to serve as an accumulator during the *parsing*
 process.  It serves as an intermediate representation of the collision filter
 group semantics. A unique instance should be owned by each RigidBodyTree
 instance.

 The manager is used in parsing *sessions*.  By design, one session maps to
 parsing a single file.  The session ends with a call to Clear (called in
 RigidBodyTree::compile).  Multiple sessions can be run on a single manager
 (this naturally arises from parsing multiple files).  Collision filter groups
 defined during a single session must all have unique group *names*. Names
 used in different sessions are treated as being *different* collision filter
 groups.  The names of the groups is only maintained during parsing.  At
 runtime, the names are replaced with integer identifiers.

 The group manager can handle a finite number of groups
 (MAX_NUM_COLLISION_FILTER_GROUPS).  Each session contributes towards reaching
 that total.  If the maximum number of groups has been defined, subsequent
 efforts to define a new collision filter group will cause exceptions to be
 thrown.

 There are several implications of this design:

    - Collision filter groups have a scope limited to a single URDF file. That
      means a collision filter group *cannot* be configured between bodies
      defined in different URDF files.
    - Even if the same file is parsed multiple times, leading to collision
      filter groups with identical names in each session, the groups will be
      considered different.  Bodies from one parsing of the file will not be
      in groups visible to another parsing.
    - A user cannot change the collision filter groups a body belongs to or
      which groups it ignores outside of the URDF specification.

 @tparam  T  A valid Eigen scalar type.
 */
template <typename T>
class CollisionFilterGroupManager {
 public:
  /** Default constructor. */
  CollisionFilterGroupManager() {}

  /**
   Based on the current specification, builds the appropriate collision filter
   bitmasks and assigns them to the specified rigid bodies.
   */
  void CompileGroups();

  /**
   Attempts to define a new collision filter group.  The given name *must*
   be unique.  Duplicate names or attempting to add more
   collision filter groups than the system can handle will lead to failure. In
   the event of failure, an exception is thrown.
   @param name        The unique name of the new group.
   */
  void DefineCollisionFilterGroup(const std::string& name);

  // Note: unlike the other public methods of this class, this method does *not*
  // throw an exception upon failure.  The reason is two-fold:
  //  1) This method a a single fail condition (which can be succinctly
  //     communicated with a boolean.)
  //  2) The caller has more context to why this was called and is better able
  //     to provide a meaningful error message.
  /**
   Adds a RigidBody to a collision filter group.  The process will fail if the
   group cannot be found.
   @param group_name      The collision filter group name to add the body to.
   @param body            The name of the body to add (as a member of the
                          group's model instance id.
   @returns False if the group could not be found.
   */
  bool AddCollisionFilterGroupMember(const std::string& group_name,
                                     const RigidBody<T>& body);

  /**
   Adds a collision group to the set of groups ignored by the specified
   collision filter group.  Will fail if the specified specified group name
   does not refer to an existing collision filter group.  (Although, the
   target group name need not exist at this time.)  An exception is thrown
   upon failure.
   @param group_name            The name of the group to modify.
   @param target_group_name     The name of the group to ignore.
   */
  void AddCollisionFilterIgnoreTarget(const std::string& group_name,
                                      const std::string& target_group_name);

  /**
   Reports the collision filter group assigned to the given group name.
   @param group_name    The group name to query.
   @returns the assigned group name (-1 if in valid group name).
   */
  int GetGroupId(const std::string& group_name);

  /**
   Returns the group membership bitmask for the given @p body.  If there is
   no information for this body, the zero bitmask will be returned.
   */
  bitmask get_group_mask(const RigidBody<T>& body);

  /**
   Returns the ignored group bitmask for the given @p body.  If there is
   no information for this body, the zero bitmask will be returned.
   */
  bitmask get_ignore_mask(const RigidBody<T>& body);

  // TODO(SeanCurtis-TRI): Kill this method when matlab dependencies are
  // removed.  There is a corresponding method on the RigidBodyTree.
  /**
   Directly set the masks for a body.  The values will remain in the current
   session (i.e., until Clear is called).
   This is a convenience function for Matlab integration.  The Matlab parser
   handles the mapping of collision filter group names to ids and passes the
   mapped ids directly the manager for when the tree gets compiled.  It relies
   on correct encoding of groups into bitmasks.
   */
  void SetBodyCollisionFilters(const RigidBody<T>& body, const bitmask& group,
                               const bitmask& ignores);

  /**
   Clears the cached collision filter group specification data.  It does *not*
   reset the counter for available collision filter groups.  This allows the
   accumulation of unique collision filter groups for a single instance of
   the manager.
   */
  void Clear();

 private:
  // Attempts to provision a group id for the next group.  Throws an exception
  // if this manager is out of ids.
  int acquire_next_group_id();

  // The next available collision filter group identifier.  Assumes that 0
  // is the default group (which is implicitly consumed.)
  int next_id_{1};

  // Map between group names and specification of its collision filter group.
  std::unordered_map<std::string, DrakeCollision::CollisionFilterGroup<T>>
      collision_filter_groups_{};

  // Mappings between a RigidBody and the bitmasks that define its group
  // membership and the groups it ignores.  This is populated during
  // CompileGroups.  The pair is: (group mask, ignore mask).
  std::unordered_map<const RigidBody<T>*, std::pair<bitmask, bitmask>>
      body_groups_;
};
}  // namespace DrakeCollision
