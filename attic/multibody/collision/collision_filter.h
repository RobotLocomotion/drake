#pragma once

#include <bitset>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"

// forward declaration
template <typename U>
class RigidBody;

namespace drake {
namespace multibody {
namespace collision {
/** The maximum width of the collision filter group bitmasks. */
constexpr int kMaxNumCollisionFilterGroups = 128;

typedef std::bitset<kMaxNumCollisionFilterGroups> bitmask;

// Constants
// The empty bit mask.  Used in the membership group mask represents no
// membership.  As the ignore mask, the body ignores nothing.
constexpr bitmask kNoneMask(0);
// The membership bit mask indicating the CFG to which *all* bodies belong. A
// body can be made invisible (from a collision perspective) by having setting
// its ignore mask to kDefaultGroup.
constexpr bitmask kDefaultGroup(1);

/**
 The specification of a collision filter group: its name, bodies that belong
 to it, and the names of collision filter groups that it ignores.  This
 class is used for initialization and not run-time calculations.

 A collision filter group is a mechanism for cheaply culling pairs of collision
 elements from consideration during collision detection. One collision filter
 group associates a set of bodies with a set of *ignored* collision filter
 groups.  At runtime, when a pair of bodies @f$(A, B)@f$ are determined to be a
 collision candidate, their collision filter group membership is examined.

 Given the following definitions:

   - @f$G(A) ≜ \{g^A_0, g^A_1, ..., g^A_n\}@f$ is the set of all groups
     to which @f$A@f$ belongs,
   - @f$I(f) ≜ \{g^f_0, g^f_1, ..., g^f_m\}@f$ is the set set of all
     groups that group @f$f@f$ ignores,
   - @f$I(A) ≜ \{I(g^A_0) \cap I(g^A_1) \cap ... \cap I(g^A_n)\}@f$
     such that @f$g^A_i \in G(A)@f$ is the set of all groups that @f$A@f$
     ignores.

 Then, the pair @f$(A, B)@f$ will be filtered if:

    @f$I(A) \cap G(B) \ne \emptyset \lor I(B) \cap G(A) \ne \emptyset@f$.

 In other words, if either body belongs to a group which is ignored by *any*
 group the other body belongs to.

 @tparam  T  A valid Eigen scalar type.
 */
template <typename T>
class CollisionFilterGroup {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterGroup)

  /**
   Default constructor required by use in std::unordered_map.
   */
  CollisionFilterGroup();

  /**
   @param name          The name for the collision filter group.
   @param id            The bit id for this collision filter group.
   */
  CollisionFilterGroup(const std::string& name, int id);

  int get_mask_id() const { return mask_id_; }

  void add_body(const RigidBody<T>& body) { bodies_.push_back(&body); }

  std::vector<const RigidBody<T>*>& get_bodies() { return bodies_; }

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

// TODO(SeanCurtis-TRI): Per discussion in issue #4729, this will eventually
// be expanded to include programmatic manipulation of groups during
// construction, and, eventually, dynamic manipulation of groups during
// simulation.
/**
 This class provides management utilities for the definition of collision filter
 groups for RigidBodyTree instances.

 The intent of the manager is to serve as an accumulator during the
 *construction* process. It serves as an intermediate representation of the
 collision filter group semantics. A unique instance should be owned by each
 RigidBodyTree instance.

 The manager is used in construction *sessions*. The first session begins upon
 construction and ends with a call to Clear(). Each call to Clear() implicitly
 marks the beginning of the next session. Collision filter groups
 defined during a single session must all have unique group *names*. Names
 repeated in different sessions are treated as being *different* collision
 filter groups.  The names of the groups are only maintained during the session.
 During RigidBodyTree compilation, the names are replaced with integers
 in the filter mechanism identifiers.

 The group manager can only handle a finite number of groups
 (kMaxNumCollisionFilterGroups). Each session contributes towards reaching
 that total. If the maximum number of groups has been reached, subsequent
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CollisionFilterGroupManager)

  /** Default constructor. */
  CollisionFilterGroupManager() {}

  /**
   Based on the current specification, builds the appropriate collision filter
   bitmasks and assigns them to the previously provided rigid bodies.
   */
  void CompileGroups();

  /**
   Attempts to define a new collision filter group. The given name *must*
   be unique in this session. Duplicate names or attempting to add more
   collision filter groups than the system can handle will lead to failure.
   @param name        The unique name of the new group.
   @throws std::logic_error in response to failure conditions.
   */
  void DefineCollisionFilterGroup(const std::string& name);

  // Note: unlike the other public methods of this class, this method does *not*
  // throw an exception upon failure.  The reason is two-fold:
  //  1) This method has a single fail condition (which can be succinctly
  //     communicated with a boolean.)
  //  2) The caller has more context to why this was called and is better able
  //     to provide a meaningful error message.  More particularly, this code
  //     cannot create an error message including the body name without creating
  //     a circular dependency between drakeRBM and drakeCollision.
  /**
   Adds a RigidBody to a collision filter group.  The process will fail if the
   group cannot be found.
   @param group_name      The name of the collision filter group to add the body
                          to.
   @param body            The body to add.
   @returns False if the group could not be found.
   */
  bool AddCollisionFilterGroupMember(const std::string& group_name,
                                     const RigidBody<T>& body);

  /**
   Adds a collision group to the set of groups ignored by the specified
   collision filter group. Will fail if the specified group name
   does not refer to an existing collision filter group. (Although, the
   target group name need not exist at this time.)
   @param group_name            The name of the group to modify.
   @param target_group_name     The name of the group to ignore.
   @throws std::logic_error in response to failure conditions.
   */
  void AddCollisionFilterIgnoreTarget(const std::string& group_name,
                                      const std::string& target_group_name);

  /**
   Reports the collision filter group assigned to the given group name.
   @param group_name    The group name to query.
   @returns the assigned group id (kInvalidGroupId if an valid group name).
   */
  int GetGroupId(const std::string& group_name);

  /**
   Returns the group membership bitmask for the given @p body.  If there is
   no information for this body, the zero bitmask will be returned.  This
   should only be called *after* CompileGroups.
   */
  const bitmask& get_group_mask(const RigidBody<T>& body);

  /**
   Returns the ignored group bitmask for the given @p body.  If there is
   no information for this body, the zero bitmask will be returned.  This
   should only be called *after* CompileGroups.
   */
  const bitmask& get_ignore_mask(const RigidBody<T>& body);

  /**
   Clears the cached collision filter group specification data from the current
   session.  It does *not* reset the counter for available collision filter
   groups. This is what makes it possible for a file to be read multiple times
   in sequence, but to have each parsing produce a unique set of collision
   filter groups.  Or if two files are parsed, and both use a common name for a
   collision filter group. They are treated as unique collision filter groups
   and all of those groups count against the *total* number of groups supported
   by a single instance of the manager.
   */
  void Clear();

  const std::unordered_map<const RigidBody<T>*, std::pair<bitmask, bitmask>>&
  body_groups() const { return body_groups_; }

  /**
   Reported value for group names that do not map to known collision filter
   groups.
   */
  static const int kInvalidGroupId;

 private:
  // Attempts to provision a group id for the next group.  Throws an exception
  // if this manager is out of ids.
  int acquire_next_group_id();

  // The next available collision filter group identifier.  Assumes that 0
  // is the default group (which is implicitly consumed.)
  int next_id_{1};

  // Map between group names and its collision filter group specification.
  std::unordered_map<std::string,
                     drake::multibody::collision::CollisionFilterGroup<T>>
      collision_filter_groups_{};

  // Mappings between a RigidBody and the bitmasks that define its group
  // membership and the groups it ignores.  This is populated during
  // CompileGroups. The pair is: (group mask, ignore mask).
  std::unordered_map<const RigidBody<T>*, std::pair<bitmask, bitmask>>
      body_groups_;
};
}  // namespace collision
}  // namespace multibody
}  // namespace drake
