#pragma once

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <class ElementType, typename ElementIndexType>
class MultibodyTreeElement;

template <template <typename> class ElementType, typename T, typename ElementIndexType>
class MultibodyTreeElement<ElementType<T>, ElementIndexType> {
 public:
  const MultibodyTree<T>& get_parent_tree() const {
    DRAKE_ASSERT_VOID(HasParentTreeOrThrows());
    return *parent_tree_;
  }

  ElementIndexType get_id() const { return id_;}

  /// Chekcs if this MultibodyTreeElement has been registered into a
  /// MultibodyTree. If not, it throws an exception.
  void HasParentTreeOrThrows() const {
    if (parent_tree_ == nullptr || get_id().is_invalid()) {
      throw std::runtime_error(
          "This multibody component was not added to a MultibodyTree.");
    }
  }

  template <template <typename> class OtherElementType, typename OtherElementIndexType>
  void HasSameParentTreeOrThrows(
      const MultibodyTreeElement<OtherElementType<T>, OtherElementIndexType>&
      other) {
    this->HasParentTreeOrThrows();
    other.HasParentTreeOrThrows();
    if (parent_tree_ != other.parent_tree_) {
      throw std::runtime_error(
          "These two MultibodyTreeElement's do not belong to the same tree.");
    }
  }

  /// Gives MultibodyTree elements the opportunity to perform internal setup
  /// when MultibodyTree::Compile() is invoked.
  virtual void Compile() = 0;

 protected:
  const MultibodyTree<T>* parent_tree_{nullptr};
  ElementIndexType id_{ElementIndexType::Invalid()};

  // Only MultibodyTree<T> can set these.
  void set_parent_tree(const MultibodyTree<T>* tree) { parent_tree_ = tree; }
  virtual void set_id(ElementIndexType id) { id_ = id; }
};

}  // namespace multibody
}  // namespace drake
