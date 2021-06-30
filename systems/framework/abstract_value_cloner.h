#pragma once

#include <memory>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {
namespace internal {

/* Provies a ValueProducer::AllocateCallback functor that clones the given
model_value. */
class AbstractValueCloner final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AbstractValueCloner)

  /* Creates a clone functor for the given model value. */
  template <typename SomeOutput>
  explicit AbstractValueCloner(const SomeOutput& model_value)
      : AbstractValueCloner(AbstractValue::Make<SomeOutput>(model_value)) {}

  ~AbstractValueCloner();

  /* Returns a Clone of the model_value passed into the constructor. */
  std::unique_ptr<AbstractValue> operator()() const;

 private:
  /* Creates a clone functor for the given model value. */
  explicit AbstractValueCloner(std::unique_ptr<AbstractValue> model_value);

  copyable_unique_ptr<AbstractValue> model_value_;
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
