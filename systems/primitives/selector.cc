#include "drake/systems/primitives/selector.h"

#include <algorithm>
#include <deque>
#include <map>
#include <set>
#include <utility>

#include "drake/systems/primitives/selector_internal.h"

namespace drake {
namespace systems {
namespace {

using InputPortParams = SelectorParams::InputPortParams;
using OutputPortParams = SelectorParams::OutputPortParams;
using OutputSelection = SelectorParams::OutputSelection;
using SelectorProgram = internal::SelectorProgram;

/* Given a list of elements, merges elements as much as possible by replacing
logically abutting elements with a single element that has a larger count. */
void ConsolidateElements(std::vector<SelectorProgram::Element>* elements) {
  using Element = SelectorProgram::Element;
  DRAKE_DEMAND(elements != nullptr);

  // Make a worklist of sorted elements.
  std::sort(elements->begin(), elements->end());
  std::deque<Element> worklist(elements->begin(), elements->end());

  // Move elements from the worklist to the result, merging as possible.
  std::vector<SelectorProgram::Element> result;
  while (!worklist.empty()) {
    const Element candidate = worklist.front();
    worklist.pop_front();
    if (!worklist.empty()) {
      const Element next = worklist.front();
      if ((candidate.input_start + candidate.count == next.input_start) &&
          (candidate.output_start + candidate.count == next.output_start)) {
        // The 'candidate' and 'next' are abutting, so we'll merge them.
        Element replacement = candidate;
        replacement.count += next.count;
        // Overwrite 'next'. This is equivalent to popping 'next' and pushing
        // 'replacement' back onto the front of the worklist.
        worklist.front() = replacement;
        continue;
      }
    }
    // We couldn't merge the candidate, so it goes onto 'result' unchanged.
    result.push_back(candidate);
  }

  // Write out the result.
  *elements = std::move(result);
}

}  // namespace

namespace internal {

SelectorProgram::SelectorProgram(
    const SelectorParams& params,
    const std::vector<const InputPortBase*>& input_ports) {
  DRAKE_DEMAND(params.inputs.size() == input_ports.size());
  for (const OutputPortParams& output : params.outputs) {
    // Group the output elements by the input port they come from.
    std::map<InputPortIndex, std::vector<Element>> per_input;
    int output_offset = 0;
    for (const OutputSelection& selection : output.selections) {
      const InputPortIndex input_port_index{selection.input_port_index};
      per_input[input_port_index].push_back(Element{
          .count = 1,
          .input_start = selection.input_offset,
          .output_start = output_offset,
      });
      ++output_offset;
    }
    // Consolidate each per-input recipe into as few elements as possible.
    for (auto& [_, elements] : per_input) {
      ConsolidateElements(&elements);
    }
    // Pack the recipe into a PerOutput helper struct. We'll pack a pre-fetched
    // input port pointer so that CalcOutput doesn't need to perform all of the
    // extra error checking during an inner loop.
    PerOutput compiled_output(per_input.size());
    int j = 0;
    for (auto& [input_port_index, elements] : per_input) {
      compiled_output[j].first = input_ports[input_port_index];
      compiled_output[j].second = std::move(elements);
      ++j;
    }
    outputs_.push_back(std::move(compiled_output));
  }
}

}  // namespace internal

namespace {

// Returns the framework spelling of the given params port name.
std::variant<std::string, UseDefaultName> ConvertPortName(
    const std::optional<std::string>& name) {
  if (name.has_value()) {
    return name.value();
  }
  return kUseDefaultName;
}

}  // namespace

template <typename T>
Selector<T>::Selector(SelectorParams params)
    : LeafSystem<T>(SystemTypeTag<Selector>{}), params_(std::move(params)) {
  // Validate input parameters and declare input ports.
  const int num_inputs = ssize(params_.inputs);
  std::vector<const InputPortBase*> input_ports;
  for (const InputPortParams& input : params_.inputs) {
    DRAKE_THROW_UNLESS(input.size >= 0);
    const InputPort<T>& port =
        this->DeclareVectorInputPort(ConvertPortName(input.name), input.size);
    input_ports.push_back(&port);
  }

  // Validate output parameters and declare output ports.
  const int num_outputs = ssize(params_.outputs);
  for (OutputPortIndex i{0}; i < num_outputs; ++i) {
    const OutputPortParams& output = params_.outputs[i];
    const int output_size = ssize(output.selections);
    std::set<DependencyTicket> prerequisites_of_calc;
    for (const auto& [input_port_index, input_offset] : output.selections) {
      DRAKE_THROW_UNLESS(input_port_index >= 0);
      DRAKE_THROW_UNLESS(input_port_index < num_inputs);
      DRAKE_THROW_UNLESS(input_offset >= 0);
      DRAKE_THROW_UNLESS(input_offset < params_.inputs[input_port_index].size);
      prerequisites_of_calc.insert(
          this->input_port_ticket(InputPortIndex{input_port_index}));
    }
    if (output_size == 0) {
      prerequisites_of_calc.insert(this->nothing_ticket());
    }
    this->DeclareVectorOutputPort(
        ConvertPortName(output.name), output_size,
        [this, i](const Context<T>& context, BasicVector<T>* result) {
          this->CalcOutput(context, i, result);
        },
        std::move(prerequisites_of_calc));
  }

  // Convert the params to a more suitable form for use by CalcOutput.
  program_ = std::make_unique<SelectorProgram>(params_, input_ports);
}

template <typename T>
template <typename U>
Selector<T>::Selector(const Selector<U>& other) : Selector(other.params_) {}

template <typename T>
Selector<T>::~Selector() = default;

template <typename T>
void Selector<T>::CalcOutput(const Context<T>& context,
                             OutputPortIndex output_port_index,
                             BasicVector<T>* output) const {
  Eigen::VectorBlock<VectorX<T>> output_block = output->get_mutable_value();
  for (const auto& [input_port_base, elements] :
       program_->get(output_port_index)) {
    const InputPort<T>& input_port =
        *static_cast<const InputPort<T>*>(input_port_base);
    const VectorX<T>& input = input_port.Eval(context);
    for (const SelectorProgram::Element& element : elements) {
      for (int j = 0; j < element.count; ++j) {
        output_block[element.output_start + j] = input[element.input_start + j];
      }
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Selector);
