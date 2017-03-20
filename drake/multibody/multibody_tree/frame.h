#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"

#include <iostream>

namespace drake {
namespace multibody {

// Forward declarations.
template <class T> class Body;
template <class T> class MultibodyTree;
template <class T> class RigidBody;

template <typename T>
class Frame : public MultibodyTreeElement<Frame<T>, FrameIndex> {};

template <typename T>
class MaterialFrame : public Frame<T> {
 public:
  BodyIndex get_body_index() const { return body_index_;}

 protected:
  // Only derived classes can use this constructor.
  explicit MaterialFrame(const Body<T>& body);

 private:
  const BodyIndex body_index_;

  /// At MultibodyTree::Compile() time, each frame will retrieve its topology
  /// from the parent MultibodyTree.
  virtual void Compile() {}
};

template <typename T>
class BodyFrame : public MaterialFrame<T> {
 public:
  /// Creates a new BodyFrame and adds it to the MultibodyTree @p tree.
  /// The MultibodyTree @param tree takes ownership of the frame.
  static const BodyFrame<T>& Create(MultibodyTree<T>* tree, const Body<T>& body);

 private:
  // Do not allow users to create a body frames using its public constructors
  // but force them to use the factory method Create().

 public: // TODO: REMOVE. RIGHT NOW FOR TESTING.
  BodyFrame(const Body<T>& body);
};

/// This class represents a frame `F` with pose `X_BF` measured and expressed in
/// the body frame `B` of a rigid body.
template <typename T>
class RigidBodyFrame : public MaterialFrame<T> {
 public:
  static RigidBodyFrame<T>& Create(
      MultibodyTree<T>* tree,
      const RigidBody<T>& body, const Isometry3<T>& X_BM);

 private:
  RigidBodyFrame(const RigidBody<T>& B, const Isometry3<T>& X_BM);

  Isometry3<T> X_BM_;
};

}  // namespace multibody
}  // namespace drake
