#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rbt_with_alternates/object_with_alternates.h"

template <typename T>
class RigidBodyTreeWithAlternates :
    public ObjectWithAlternates<RigidBodyTreeWithAlternates, T> {  // CRTP!
  using Super = ObjectWithAlternates<::RigidBodyTreeWithAlternates, T>;

 public:
  // Construct and take over ownership of the given RigidBodyTree.
  RigidBodyTreeWithAlternates(std::unique_ptr<RigidBodyTree<T>> tree) :
      tree_(std::move(tree)) {}

  // Construct an alternate MySystem<T> with same structure as the given one.
  // This is also the copy constructor for the T=double fundamental.
  explicit RigidBodyTreeWithAlternates(
      const RigidBodyTreeWithAlternates<double>& fundamental)
      : Super(fundamental), tree_(new RigidBodyTree<T>(*fundamental.tree_)) {}

  // For demo purposes, report the name of the scalar type here.
  const char* type() const { return typeid(T).name(); }

  // Return interesting concrete System-specific stuff.
  const RigidBodyTree<T>& get_rigid_body_tree() const { return *tree_; }

 private:
  // Let all other instantiations of this class be friends with this one.
  template <typename TT>
  friend class RigidBodyTreeWithAlternates;

  std::unique_ptr<RigidBodyTree<T>> tree_;
};
