#pragma once

#include <memory>
#include <typeindex>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

// TODO(jwnimmer-tri) This was created as workaround for drake#14011, but is
// useful even beyond that specific circumstance. We plan to promote this to
// Drake (and in doing so, fix it to use the correct filename).

namespace anzu {
/** %SharedPointerSystem holds a single `shared_ptr` that will be released at
System deletion time (i.e., the end of a Diagram lifespan). It has no input,
output, state, nor parameters. This is useful for storing objects that will be
pointed-to by other systems outside of the usual input/output port connections.

Scalar conversion is supported and will simply increment the reference count
for the contained object. Advanced users who need to disable conversion may
access the get_system_scalar_converter() to do so. Note that the contained
object is not scalar-converted, so should not depend on T.

@tparam_default_scalar */
template <typename T>
class SharedPointerSystem final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SharedPointerSystem)

  /** Creates a system holding the given value.
  The value is allowed to be `nullptr`.
  @tparam Held type used to store the given value.
    Calls to get<>() must provide the same type for retrieval. */
  template <typename Held>
  explicit SharedPointerSystem(std::shared_ptr<Held> value_to_hold)
      : SharedPointerSystem(std::move(value_to_hold),
                        std::type_index(typeid(Held))) {}

  /** Creates a system holding the given value.
  This overload accepts a unique_ptr (but still stores it at a shared_ptr).
  The value is allowed to be `nullptr`.
  @tparam Held type used to store the given value.
    Calls to get<>() must provide the same type for retrieval. */
  template <typename Held>
  explicit SharedPointerSystem(std::unique_ptr<Held> value_to_hold)
      : SharedPointerSystem(std::shared_ptr<Held>(std::move(value_to_hold))) {}

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit SharedPointerSystem(const SharedPointerSystem<U>&);

  ~SharedPointerSystem() final;

  /** Creates a system holding the given value and adds it to the builder.
  The value is allowed to be `nullptr`. Returns an alias to the value.
  @tparam Held type used to store the given value.
    Calls to get<>() must provide the same type for retrieval.
  @pre builder is non-null */
  template <typename Held>
  static Held* AddToBuilder(
      drake::systems::DiagramBuilder<T>* builder,
      std::shared_ptr<Held> value_to_hold) {
    DRAKE_THROW_UNLESS(builder != nullptr);
    auto holder = std::make_unique<SharedPointerSystem<T>>(
        std::move(value_to_hold));
    auto* result = holder->template get<Held>();
    builder->AddSystem(std::move(holder));
    return result;
  }

  /** Creates a system holding the given value and adds it to the builder.
  This overload accepts a unique_ptr (but still stores it at a shared_ptr).
  The value is allowed to be `nullptr`. Returns an alias to the value.
  @tparam Held type used to store the given value.
    Calls to get<>() must provide the same type for retrieval.
  @pre builder is non-null */
  template <typename Held>
  static Held* AddToBuilder(
      drake::systems::DiagramBuilder<T>* builder,
      std::unique_ptr<Held> value_to_hold) {
    return SharedPointerSystem::AddToBuilder(
        builder, std::shared_ptr<Held>(std::move(value_to_hold)));
  }

  /** (Advanced) Retrieves an alias to the stored value.
  @tparam Held type used to store the given value, per our constructor.
  @throws std::bad_cast if Held doesn't match the type used at construction. */
  template <typename Held>
  Held* get() const {
    if (std::type_index(typeid(Held)) != held_type_) {
      throw std::bad_cast();
    }
    return static_cast<Held*>(held_.get());
  }

 private:
  template <typename> friend class SharedPointerSystem;

  SharedPointerSystem(std::shared_ptr<void> held, std::type_index held_type);

  const std::shared_ptr<void> held_;
  const std::type_index held_type_;
};
}  // namespace anzu

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::anzu::SharedPointerSystem)
