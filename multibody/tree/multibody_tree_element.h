#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

namespace internal {

// This is a class used by MultibodyTree internals to create the implementation
// for a particular joint object.
template <typename T>
class JointImplementationBuilder;
}  // namespace internal

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
/// At MultibodyTree::Finalize() stage, they get assigned an index that
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
///                     class on the scalar type T.
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
  const internal::MultibodyTree<T>& get_parent_tree() const {
    DRAKE_ASSERT_VOID(HasParentTreeOrThrow());
    return *parent_tree_;
  }

  /// Returns this element's unique index in its parent MultibodyTree.
  ElementIndexType index() const { return index_;}

  /// Returns this element's model instance index in its parent MultibodyTree.
  ModelInstanceIndex model_instance() const { return model_instance_;}

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

  /// Checks whether this MultibodyTreeElement belongs to the provided
  /// MultibodyTree `tree`. If not, it throws a std::logic_error.
  void HasThisParentTreeOrThrow(const internal::MultibodyTree<T>* tree) const {
    DRAKE_ASSERT(tree != nullptr);
    if (parent_tree_ != tree) {
      throw std::logic_error("This multibody component does not belong to the "
                             "supplied MultibodyTree.");
    }
  }

  // TODO(amcastro-tri): Add DeepClone API for transmogrification to other
  // scalar types.
  // This will make use the template argument "MultibodyTreeElement".

 protected:
  /// Default constructor made protected so that sub-classes can still declare
  /// their default constructors if they need to.
  MultibodyTreeElement() {}

  /// Constructor which allows specifying a model instance.
  explicit MultibodyTreeElement(ModelInstanceIndex model_instance)
      : model_instance_(model_instance) {}

  /// Gives MultibodyTree elements the opportunity to retrieve their topology
  /// when MultibodyTree::Finalize() is invoked.
  /// NVI to pure virtual method DoSetTopology().
  void SetTopology(const internal::MultibodyTreeTopology& tree) {
    DoSetTopology(tree);
  }

  /// Implementation of the NVI SetTopology(). For advanced use only for
  /// developers implementing new MultibodyTree components.
  virtual void DoSetTopology(const internal::MultibodyTreeTopology& tree) = 0;

 private:
  void set_parent_tree(
      const internal::MultibodyTree<T>* tree, ElementIndexType index) {
    index_ = index;
    parent_tree_ = tree;
  }

  void set_model_instance(ModelInstanceIndex model_instance) {
    model_instance_ = model_instance;
  }

  // MultibodyTree<T> is a natural friend of MultibodyTreeElement objects and
  // therefore it can set the owning parent tree and unique index in that tree.
  friend class internal::MultibodyTree<T>;

  const internal::MultibodyTree<T>* parent_tree_{nullptr};

  // The default index value is *invalid*. This must be set to a valid index
  // value before the element is released to the wild.
  ElementIndexType index_;

  // The default model instance id is *invalid*. This must be set to a
  // valid index value before the element is released to the wild.
  ModelInstanceIndex model_instance_;
};

}  // namespace multibody
}  // namespace drake
