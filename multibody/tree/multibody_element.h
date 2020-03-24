#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyPlant;

namespace internal {
// This is a class used by MultibodyTree internals to create the implementation
// for a particular joint object.
template <typename T>
class JointImplementationBuilder;
}  // namespace internal

/// A class representing an element (subcomponent) of a MultibodyPlant or
/// (internally) a MultibodyTree. Examples of multibody elements are bodies,
/// joints, force elements, and actuators. After a Finalize() call, multibody
/// elements get assigned an type-specific index that uniquely identifies them.
/// A generic multibody tree element `MultibodyThing` is derived from
/// this class as:
/// @code{.cpp}
/// template <typename T>
/// class MultibodyThing :
///     public MultibodyElement<MultibodyThing, T, MultibodyThingIndex> {
///  ...
/// };
/// @endcode
///
/// @tparam ElementType The type of the specific multibody element, for
///     instance, a body or a mobilizer. It must be a template class that can
///     be templatized by scalar type T.
/// @tparam_default_scalar
/// @tparam ElementIndexType The type-safe index used for this element type.
///
/// As an example of usage, consider the definition of a `ForceElement` class
/// as a multibody element. This is accomplished with:
/// @code{.cpp}
///   template <typename T>
///   class ForceElement :
///       public MultibodyElement<ForceElement, T, ForceElementIndex>;
/// @endcode
template <template <typename> class ElementType,
    typename T, typename ElementIndexType>
class MultibodyElement {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyElement)

  virtual ~MultibodyElement() {}

  /// Returns this element's unique index.
  ElementIndexType index() const { return index_;}

  /// Returns the ModelInstanceIndex of the model instance to which this
  /// element belongs.
  ModelInstanceIndex model_instance() const { return model_instance_;}

  /// Returns the MultibodyPlant that owns this %MultibodyElement.
  ///
  /// @note You can only invoke this method if you have a definition of
  /// %MultibodyPlant available. That is, you must include
  /// `drake/multibody/plant/multibody_plant.h` in the translation unit that
  /// invokes this method; multibody_element.h cannot do that for you.
  ///
  /// @throws std::logic_error if there is no %MultibodyPlant owner.
  template <typename MultibodyPlantDeferred = MultibodyPlant<T>>
  const MultibodyPlantDeferred& GetParentPlant() const {
    HasParentTreeOrThrow();

    const auto plant = dynamic_cast<const MultibodyPlantDeferred*>(
        &get_parent_tree().tree_system());

    if (plant == nullptr) {
      throw std::logic_error(
          "This multibody element was not owned by a MultibodyPlant.");
    }

    return *plant;
  }

 protected:
  /// Default constructor made protected so that sub-classes can still declare
  /// their default constructors if they need to.
  MultibodyElement() {}

  /// Constructor which allows specifying a model instance.
  explicit MultibodyElement(ModelInstanceIndex model_instance)
      : model_instance_(model_instance) {}

  /// Returns a constant reference to the parent MultibodyTree that
  /// owns this element.
  const internal::MultibodyTree<T>& get_parent_tree() const {
    DRAKE_ASSERT_VOID(HasParentTreeOrThrow());
    return *parent_tree_;
  }

  /// Gives MultibodyElement-derived objects the opportunity to retrieve their
  /// topology after MultibodyTree::Finalize() is invoked.
  /// NVI to pure virtual method DoSetTopology().
  void SetTopology(const internal::MultibodyTreeTopology& tree) {
    DoSetTopology(tree);
  }

  /// Implementation of the NVI SetTopology(). For advanced use only for
  /// developers implementing new MultibodyTree components.
  virtual void DoSetTopology(const internal::MultibodyTreeTopology& tree) = 0;

 private:
  // MultibodyTree<T> is a natural friend of MultibodyElement objects and
  // therefore it can set the owning parent tree and unique index in that tree.
  friend class internal::MultibodyTree<T>;

  // Give unit tests access to the tree.
  friend class MultibodyElementTester;

  void set_parent_tree(
      const internal::MultibodyTree<T>* tree, ElementIndexType index) {
    index_ = index;
    parent_tree_ = tree;
  }

  void set_model_instance(ModelInstanceIndex model_instance) {
    model_instance_ = model_instance;
  }

  bool has_parent_tree() const { return parent_tree_ != nullptr; }

  // Checks whether this MultibodyElement has been registered into
  // a MultibodyTree and throws an exception if not.
  // @throws std::logic_error if the element is not in a MultibodyTree.
  void HasParentTreeOrThrow() const {
    if (!has_parent_tree()) {
      throw std::logic_error(
          "This multibody element was not added to a MultibodyTree.");
    }
  }

  // Checks whether this MultibodyElement belongs to the provided
  // MultibodyTree `tree` and throws an exception if not.
  // @throws std::logic_error if `this` element is not in the given `tree`.
  void HasThisParentTreeOrThrow(const internal::MultibodyTree<T>* tree) const {
    DRAKE_ASSERT(tree != nullptr);
    if (parent_tree_ != tree) {
      throw std::logic_error("This multibody element does not belong to the "
                             "supplied MultibodyTree.");
    }
  }

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
