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
   */
  CollisionFilterGroup(const std::string& name, int model_id);

  int get_model_id() const { return model_id_; }

  void add_body(RigidBody<T>* body) { bodies_.push_back(body); }

  void add_ignore_group(const std::string& group_name) {
    ignore_groups_.push_back(group_name);
  }

 private:
  std::string name_{};
  int model_id_{};
  std::vector<RigidBody<T>*> bodies_{};
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

  /**
   Attempts to define a new collision filter group.  The given name *must*
   be unique.  Duplicate names will lead to failure.
   @param name        The unique name of the new group.
   @param model_id    The model instance id to associate with this group.
   @returns           True to indicate successful addition.
   */
  bool DefineCollisionFilterGroup(const std::string& name, int model_id);

  /**
   Adds a RigidBody to a collision filter group.  The process will fail if the
   group cannot be found.
   @param group_name      The collision filter group name to add the body to.
   @param body            The name of the body to add (as a member of the
                          group's model instance id.
   @returns False if the group could not be found.
   */
  bool AddCollisionFilterGroupMember(const std::string& group_name,
                                     RigidBody<T>* body);

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
   group is undefined, a negative number is returned.
   @param group_name    The name of the collision filter group.
   @returns             The named group's model instance id.
   */
  int GetGroupModelInstanceId(const std::string& group_name);

 private:
  // Map between group names and specification of its collision filter group.
  std::unordered_map<std::string, DrakeCollision::CollisionFilterGroup<T>>
      collision_filter_groups_{};
};
}  // namespace DrakeCollision
