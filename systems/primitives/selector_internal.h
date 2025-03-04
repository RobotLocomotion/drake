#pragma once

#include <utility>
#include <vector>

#include "drake/systems/framework/input_port_base.h"

namespace drake {
namespace systems {
namespace internal {

/* This class stores the selection recipe for all output ports in a way that's
convenient and efficient for CalcOutput to use. Its implementation appears in
selector.cc (not selector_internal.cc). */
class SelectorProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SelectorProgram);

  /* Constructs a SelectorProgram from the (already-valiated) params. */
  SelectorProgram(const SelectorParams& params,
                  const std::vector<const InputPortBase*>& input_ports);

  /* The recipe for a span of output elements. Which input port and output port
  these ranges apply to is not stored here; rather, it is inferred from the
  container(s) that store this element. */
  struct Element {
    auto operator<=>(const Element&) const = default;
    int count{};
    int input_start{};
    int output_start{};
    // TODO(jwnimmer-tri) If we find that selections often reflect a stride,
    // we could also add an input_stride and output_stride here.
  };

  /* The group of all recipe elements for a single pair of input and output
  ports. The input port is stored explicitly, but the output port is inferred
  from the container that stores this element. */
  using OutputFromInputs =
      std::pair<const InputPortBase*, std::vector<Element>>;

  /* The entire recipe for a single output port. */
  using PerOutput = std::vector<OutputFromInputs>;

  /* Returns the recipe for the given output port. */
  const PerOutput& get(OutputPortIndex i) const { return outputs_[i]; }

 private:
  std::vector<PerOutput> outputs_;
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
