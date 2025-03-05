#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
class SelectorProgram;  // Declared in selector_internal.h.
}  // namespace internal
#endif

/** The constructor arguments for a Selector. */
struct SelectorParams {
  /** Helper struct for `inputs`. */
  struct InputPortParams {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(name));
      a->Visit(DRAKE_NVP(size));
    }

    /** (Optional) Specifies a name for the port. When given, the name must be a
    valid port name. When unset, the port will use kDefaultName. */
    std::optional<std::string> name;

    /** Specifies the size of the port, which must be non-negative. */
    int size{0};
  };

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

  /** Helper struct for `outputs`. */
  struct OutputPortParams {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(name));
      a->Visit(DRAKE_NVP(selections));
    }

    /** (Optional) Specifies a name for the port. When given, the name must be a
    valid port name. When unset, the port will use kDefaultName. */
    std::optional<std::string> name;

    /** Specifies the values for the output port elements, and by implication
    the port's size. The port will have size `selections.size()`, and its i'th
    output value will be determined by the (input_port_index, input_offset) pair
    at `selections[i]`, i.e., for the m'th output port we have:
    ```txt
    n = selections[i].input_port_index
    j = selections[i].input_offset
    y_m[i] = u_n[j]
    ```
    Output elements always come from some input element; it is not possible to
    directly set an output to a constant. (If a constant is necessary, connect
    a ConstantVectorSource to an input port.) */
    std::vector<OutputSelection> selections;
  };

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(inputs));
    a->Visit(DRAKE_NVP(outputs));
  }

  /** Specifies details of the input ports, and by implication the total number
  of input ports. There will be `inputs.size()` ports in total. */
  std::vector<InputPortParams> inputs;

  /** Specifies details of the output ports, and by implication the total number
  of output ports. There will be `outputs.size()` ports in total. */
  std::vector<OutputPortParams> outputs;
};

/** This system combines multiple vector-valued inputs into multiple vector-
valued outputs. It operates at the level of individual elements of the input and
output vectors (i.e., an output port can provide a mixture of data from multiple
input ports). The inputs to this system directly feed through to its outputs.
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
  std::unique_ptr<internal::SelectorProgram> program_;
};

}  // namespace systems
}  // namespace drake
