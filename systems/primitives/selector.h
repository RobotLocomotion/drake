#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** The constructor arguments for a Selector. */
struct SelectorParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(input_sizes));
    a->Visit(DRAKE_NVP(output_selections));
    a->Visit(DRAKE_NVP(input_names));
    a->Visit(DRAKE_NVP(output_names));
  }

  /** Specifies sizes for the input ports, and by implication the total number
  of input ports. There will be `input_sizes.size()` input ports in total and
  the n'th input port will have size `input_sizes[n]`. */
  std::vector<int> input_sizes;

  /** Helper struct for `output_selections`. */
  struct OutputSelection {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(input_port_index));
      a->Visit(DRAKE_NVP(input_offset));
    }

    /** The input port to use for this single output element. */
    int input_port_index{};

    /** The element offset within the given input port to use for this single
    output element. */
    int input_offset{};
  };

  /** Specifies the values for the output ports, and by implication their sizes
  and the total number of output ports. There will be `output_selection.size()`
  output ports in total, the m'th output port will have size
  `output_selection[m].size()`, and its i'th output value will be determined by
  the (input_port_index, input_offset) pair at `output_selections[m][i]`, i.e.:
  ```txt
  n = output_selections[m][i].input_port_index
  j = output_selections[m][i].input_offset
  y_m[i] = u_n[j]
  ``` */
  std::vector<std::vector<OutputSelection>> output_selections;

  /** (Optional) Specifies names for the input ports. When empty, the input
  ports will use the kDefaultName. When non-empty, must have the same size as
  `input_sizes` and each name must be a valid port name. */
  std::vector<std::string> input_names;

  /** (Optional) Specifies names for the output ports. When empty, the output
  ports will use the kDefaultName. When non-empty, must have the same size as
  `output_selections` and each name must be a valid port name. */
  std::vector<std::string> output_names;
};

/** This system combines multiple vector-valued inputs into multiple vector-
valued outputs. The inputs to this system directly feed through to its outputs.
Refer to SelectorParams to understand how the selection is specified.

@system
name: Selector
input_ports:
- u0
- ...
- u(N-1)
output_ports:
- y0
- ...
- y(M-1)
@endsystem

The port names shown in the figure above are the defaults. Custom names may be
specified in the SelectorParams.

@tparam_default_scalar
@ingroup primitive_systems */
template <typename T>
class Selector final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Selector);

  /** Constructs a %Selector with the given parameters. */
  explicit Selector(SelectorParams params);

  // TODO(jwnimmer-tri) In addition to the params constructor, we could also
  // imagine accepting a permutation matrix via SparseMatrix as a convenience.

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit Selector(const Selector<U>&);

  ~Selector() final;

 private:
  template <typename>
  friend class Selector;

  void CalcOutput(const Context<T>& context, OutputPortIndex output_port_index,
                  BasicVector<T>* output) const;

  const SelectorParams params_;

  std::shared_ptr<const void> compiled_program_;
};

}  // namespace systems
}  // namespace drake
