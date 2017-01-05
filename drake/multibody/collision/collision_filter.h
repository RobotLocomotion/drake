#pragma once

#include <bitset>
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
}  // namespace DrakeCollision
