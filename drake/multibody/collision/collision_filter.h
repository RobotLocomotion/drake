#pragma once

#include <bitset>
#include <unordered_map>
#include <vector>

static const int MAX_NUM_COLLISION_FILTER_GROUPS = 128;


// forward declaration
template <typename U> class RigidBody;

namespace DrakeCollision {

typedef std::bitset<MAX_NUM_COLLISION_FILTER_GROUPS> bitmask;

// Constants
extern const bitmask ALL_MASK;
extern const bitmask NONE_MASK;
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
   @param model_id      The identifier of the model with which this group is
                        associated.
   @param id            The bit id for this collision filter group.
   */
  CollisionFilterGroup(const std::string& name, int model_id, int id);

  int get_model_id() const { return model_id_; }

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
  int model_id_{};
  int mask_id_{};
  std::vector<const RigidBody<T>*> bodies_{};
  std::vector<std::string> ignore_groups_{};
};

/**
 This class provides management utilities for the creation of RigidBodyTree
 instances.

 @tparam  T  A valid Eigen scalar type.
 */
template <typename T>
class CollisionFilterGroupManager {
 public:
  /** Default constructor. */
  CollisionFilterGroupManager() {}

  /** Based on the current specification, builds the appropriate collision
   filter bitmasks and assigns them to the specified rigid bodies.
   */
  void CompileGroups();

  /**
   Attempts to define a new collision filter group.  The given name *must*
   be unique.  Duplicate names or attempting to add more
   collision filter groups than the system can handle will lead to failure. In
   the event of failure, an exception is thrown.
   @param name        The unique name of the new group.
   @param model_id    The model instance id to associate with this group.
   */
  void DefineCollisionFilterGroup(const std::string& name, int model_id);

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
   @param group_name
   @param target_group_name
   */
  void AddCollisionFilterIgnoreTarget(const std::string& group_name,
                                      const std::string& target_group_name);

  /**
   Reports the model instance id associated with the given group name.  If the
   group is undefined, a negative number is returned.  Note: unlike the other
   methods in this class, this returns an error code instead of throwing an
   exception; that is so the caller can provide an intelligent message for
   failure based on information not available to this invocation.
   @param group_name    The name of the collision filter group.
   @returns             The named group's model instance id.
   */
  int GetGroupModelInstanceId(const std::string& group_name);

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

  /**
   Clears the cached collision filter group specification data.  It does *not*
   reset the counter for available collision filter groups.  This allows the
   accumulation of unique collision filter groups for a single instance of
   the manager.
   */
   void Clear();

 private:
  /**
   Acquires the next available collision filter group id.  If no id is
   available, an exception is thrown.
   @returns    The next available id.
   */
  int acquire_next_group_id();

  // The next available collision filter group identifier.
  int next_id_{0};

  // Map between group names and specification of its collision filter group.
  std::unordered_map<std::string, DrakeCollision::CollisionFilterGroup<T>>
      collision_filter_groups_{};

  // Mappings between a RigidBody and the bitmasks that define its group
  // membership and the groups it ignores.  This is populated during
  // CompileGroups.  The pair is: (group mask, ignore mask).
  std::unordered_map<const RigidBody<T>*, std::pair<bitmask, bitmask>> body_groups_;
};
}  // namespace DrakeCollision
