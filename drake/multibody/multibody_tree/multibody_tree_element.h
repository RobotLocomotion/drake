#pragma once

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

// Primary template defining the signature of this class.
// Two template arguments are required in general, the type of the class
// inheriting from MultibodyTreeElement (i.e. this is a CRTP class), and the
// type of the type-safe index used to identify these element types within a
// MultibodyTree.
// The signature below is the primary template definition for
// MultibodyTreeElement (even when at a first glance it looks like a forward
// declaration, it is not). This will allow us, a few lines below, to provide an
// explicit (full) template specialization for the case in which the class is a
// template in a scalar type T. The template specialization allows the compiler
// to automatically deduce the scalar type from the subclass template signature
// itself.
// Hide from Doxygen.
/// @cond
template <class ElementType, typename ElementIndexType>
class MultibodyTreeElement;
/// @endcond

/// A class representing an element or component of a MultibodyTree. Examples of
/// multibody tree elements are bodies, joints, force elements, and constraints.
/// Multibody tree elements are owned and managed by a parent MultibodyTree.
/// As part of their construction process they get assigned an index that
/// uniquely identifies them within their parent MultibodyTree.
/// A generic multibody tree element `MultibodyComponent` is derived from
/// this class as:
/// @code{.cpp}
/// template <typename T>
/// class MultibodyComponent :
///     public MultibodyTreeElement<MultibodyComponent<T>,
///                                 MultibodyComponentIndex> {
///  ...
/// };
/// @endcode
///
/// @tparam ElementType The type of the specific multibody element, for
///                     instance, a body or a mobilizer. It must be a template
///                     class on the scalar type `T`.
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar. With the
///           signature below the scalar type is automatically deduced from the
///           `ElementType` template argument.
/// @tparam ElementIndexType The type-safe index used for this element type.
///
/// As an example of usage, consider the definition of a `ForceElement` class
/// as a multibody tree element. This would be accomplished with:
/// @code{.cpp}
///   template <typename T>
///   class ForceElement :
///       public MultibodyTreeElement<ForceElement<T>, ForceElementIndex>;
/// @endcode
/// Notice that with the signature below the scalar type is automatically
/// deduced from the template arguments.
template <template <typename> class ElementType,
    typename T, typename ElementIndexType>
class MultibodyTreeElement<ElementType<T>, ElementIndexType> {
  // The owning MultibodyTree has access to protected methods in this class to
  // set the owning parent tree and its unique index in that tree.
  friend class MultibodyTree<T>;
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyTreeElement)

  virtual ~MultibodyTreeElement() {}

  /// Returns a constant reference to the parent MultibodyTree that owns
  /// this element.
  /// Sub-classes of %MultibodyTreeElement will have a set of `Create()` methods
  /// that, when successful, will create and add a %MultibodyTreeElement to a
  /// valid MultibodyTree. Therefore, on success, the result of a `Create()`
  /// method is a properly initialized %MultibodyTreeElement with a
  /// valid MultibodyTree parent. @see RigidBody::Create() for an example of a
  /// `Create()` method.
  const MultibodyTree<T>& get_parent_tree() const {
    DRAKE_ASSERT_VOID(HasParentTreeOrThrow());
    return *parent_tree_;
  }

  /// Returns the unique index in its parent MultibodyTree to this element.
  ElementIndexType get_index() const { return index_;}

  /// Checks whether this MultibodyTreeElement has been registered into a
  /// MultibodyTree. If not, it throws an exception of type std::logic_error.
  void HasParentTreeOrThrow() const {
    if (parent_tree_ == nullptr) {
      throw std::logic_error(
          "This multibody component was not added to a MultibodyTree.");
    }
  }

  /// Checks whether `this` element has the same parent tree as `other`.
  /// If not, it throws an exception of type `std::logic_error`.
  /// A `std::logic_error` exception is thrown if either or both elements do not
  /// have a parent tree.
  template <template <typename> class OtherElementType,
      typename OtherElementIndexType>
  void HasSameParentTreeOrThrow(
      const MultibodyTreeElement<OtherElementType<T>, OtherElementIndexType>&
      other) const {
    this->HasParentTreeOrThrow();
    other.HasParentTreeOrThrow();
    if (parent_tree_ != other.parent_tree_) {
      throw std::logic_error(
          "These two MultibodyTreeElement objects do not belong to "
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
  // Default constructor made protected so that sub-classes can still declare
  // their default constructors if they need to.
  MultibodyTreeElement() {}

  // Only derived sub-classes can call these set methods from within their
  // Create() factories.
  void set_parent_tree(const MultibodyTree<T>* tree) { parent_tree_ = tree; }
  void set_index(ElementIndexType index) { index_ = index; }

 private:
  const MultibodyTree<T>* parent_tree_{nullptr};
  // ElementIndexType requires a valid initialization.
  ElementIndexType index_{0};
};

}  // namespace multibody
}  // namespace drake
