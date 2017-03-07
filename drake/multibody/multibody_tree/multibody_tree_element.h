#pragma once

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <class ElementType, typename ElementIndexType>
class MultibodyTreeElement;

/// A class representing an element or component of a MultibodyTree. Examples of
/// multibody tree elements are bodies, joints, force elements and constraints.
/// Multibody tree elements are owned and managed by a parent MultibodyTree.
/// As part of their construction process they get assigned an id that uniquely
/// identifies them within their parent MultibodyTree.
/// Multibody tree elements have type safe index that is generated with the
/// macro DrakeMultibody_DEFINE_INDEX_TYPE(IndexClass).
/// A generic multibody tree element `MultibodyComponent` is derived from
/// this class as: <pre>
/// template <typename T>
/// class MultibodyComponent :
///     public MultibodyTreeElement<MultibodyComponent<T>,
///                                 MultibodyComponentIndex> {
///  ...
/// };
/// </pre>
///
/// @tparam ElementType The type of the multibody tree component.
/// @tparam ElemeentIndexType The type-safe index used for this element type.
template <template <typename> class ElementType,
    typename T, typename ElementIndexType>
class MultibodyTreeElement<ElementType<T>, ElementIndexType> {
 public:
  virtual ~MultibodyTreeElement() {}

  /// Returns a constant reference to the parent MultibodyTree that owns
  /// this element.
  const MultibodyTree<T>& get_parent_tree() const {
    DRAKE_ASSERT_VOID(HasParentTreeOrThrows());
    return *parent_tree_;
  }

  /// Returns the unique identifier in its parent MultibodyTree to this element.
  ElementIndexType get_id() const { return id_;}

  /// Checks if this MultibodyTreeElement has been registered into a
  /// MultibodyTree. If not, it throws an exception of type std::runtime_error.
  void HasParentTreeOrThrows() const {
    if (parent_tree_ == nullptr) {
      throw std::runtime_error(
          "This multibody component was not added to a MultibodyTree.");
    }
  }

  /// Checks if `this` element has the same parent three as @p other.
  /// If not, it throws an exception of type std::runtime_error.
  template <template <typename> class OtherElementType,
      typename OtherElementIndexType>
  void HasSameParentTreeOrThrows(
      const MultibodyTreeElement<OtherElementType<T>, OtherElementIndexType>&
      other) const {
    this->HasParentTreeOrThrows();
    other.HasParentTreeOrThrows();
    if (parent_tree_ != other.parent_tree_) {
      throw std::runtime_error(
          "These two MultibodyTreeElement's do not belong to "
          "the same MultibodyTree.");
    }
  }

  /// Gives MultibodyTree elements the opportunity to perform internal setup
  /// when MultibodyTree::Compile() is invoked.
  virtual void Compile() = 0;

  // TODO(amcastro-tri): Add DeepClone API for transmogrification to other
  // scalar types.
  // This will make use the template argument "MultibodyTreeElement".

 protected:
  const MultibodyTree<T>* parent_tree_{nullptr};
  ElementIndexType id_{0};  // ElementIndexType requires a valid initialization.

  // Only derived sub-classes can set these within their Create() factories.
  void set_parent_tree(const MultibodyTree<T>* tree) { parent_tree_ = tree; }
  virtual void set_id(ElementIndexType id) { id_ = id; }
};

}  // namespace multibody
}  // namespace drake
