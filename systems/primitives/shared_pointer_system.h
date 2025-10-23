#pragma once

#include <memory>
#include <typeindex>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** %SharedPointerSystem holds a single `shared_ptr` that will be released at
System deletion time (i.e., the end of a Diagram lifespan). It has no input,
output, state, nor parameters. This is useful for storing objects that will be
pointed-to by other systems outside of the usual input/output port connections.

Scalar conversion is supported and will simply increment the reference count
for the contained object. The contained object will not be scalar-converted,
so should not depend on `T`.

@tparam_default_scalar */
template <typename T>
class SharedPointerSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SharedPointerSystem);

  /** Creates a system holding the given value.
  The value is allowed to be `nullptr`.
  @note To immediately give up ownership at the call site,
    remember to use `std::move` on the `value_to_hold`.
  @tparam Held The type used to store the given value.
    Calls to get<>() must provide the same type for retrieval. */
  template <typename Held>
  explicit SharedPointerSystem(std::shared_ptr<Held> value_to_hold)
      : SharedPointerSystem(std::move(value_to_hold),
                            std::type_index(typeid(Held))) {}

  /** Creates a system holding the given value.
  This overload accepts a unique_ptr (but still stores it at a shared_ptr).
  The value is allowed to be `nullptr`.
  @tparam Held The type used to store the given value.
    Calls to get<>() must provide the same type for retrieval.
  @exclude_from_pydrake_mkdoc{A unique_ptr input is unworkable in Python} */
  template <typename Held>
  explicit SharedPointerSystem(std::unique_ptr<Held> value_to_hold)
      : SharedPointerSystem(std::shared_ptr<Held>(std::move(value_to_hold))) {}

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit SharedPointerSystem(const SharedPointerSystem<U>&);

  ~SharedPointerSystem() final;

  /** Creates a system holding the given value and adds it to the builder.
  The value is allowed to be `nullptr`. Returns an alias to the value (or
  `nullptr` in case `nullptr` was passed in as the `value_to_hold`).
  @note To immediately give up ownership at the call site,
    remember to use `std::move` on the `value_to_hold`.
  @tparam Held The type used to store the given value.
    Calls to get<>() must provide the same type for retrieval.
  @pre builder is non-null */
  template <typename Held>
  static Held* AddToBuilder(DiagramBuilder<T>* builder,
                            std::shared_ptr<Held> value_to_hold) {
    DRAKE_THROW_UNLESS(builder != nullptr);
    auto holder =
        std::make_unique<SharedPointerSystem<T>>(std::move(value_to_hold));
    auto* result = holder->template get<Held>();
    builder->AddSystem(std::move(holder));
    return result;
  }

  /** Creates a system holding the given value and adds it to the builder.
  This overload accepts a unique_ptr (but still stores it at a shared_ptr).
  The value is allowed to be `nullptr`. Returns an alias to the value (or
  `nullptr` in case `nullptr` was passed in as the `value_to_hold`)
  @tparam Held The type used to store the given value.
    Calls to get<>() must provide the same type for retrieval.
  @pre builder is non-null
  @exclude_from_pydrake_mkdoc{A unique_ptr input is unworkable in Python} */
  template <typename Held>
  static Held* AddToBuilder(DiagramBuilder<T>* builder,
                            std::unique_ptr<Held> value_to_hold) {
    return SharedPointerSystem::AddToBuilder(
        builder, std::shared_ptr<Held>(std::move(value_to_hold)));
  }

  /** (Advanced) Retrieves an alias to the stored value.
  Returns `nullptr` in case `nullptr` was passed in as the `value_to_hold`.
  @tparam Held The type used to store the given value, per our constructor.
  @throws std::bad_cast if Held doesn't match the type used at construction. */
  template <typename Held>
  Held* get() const {
    if (std::type_index(typeid(Held)) != held_type_) {
      throw std::bad_cast();
    }
    return static_cast<Held*>(held_.get());
  }

 private:
  template <typename>
  friend class SharedPointerSystem;

  SharedPointerSystem(std::shared_ptr<void> held, std::type_index held_type);

  typename LeafSystem<T>::GraphvizFragment DoGetGraphvizFragment(
      const typename LeafSystem<T>::GraphvizFragmentParams&) const final;

  const std::shared_ptr<void> held_;
  const std::type_index held_type_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SharedPointerSystem);
