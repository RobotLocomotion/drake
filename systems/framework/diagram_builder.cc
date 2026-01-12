#include "drake/systems/framework/diagram_builder.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

#include <fmt/ranges.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace {

/* Erases the i'th element of vec, shifting everything after it over by one. */
template <typename StdVector>
void VectorErase(StdVector* vec, size_t i) {
  DRAKE_DEMAND(vec != nullptr);
  const size_t size = vec->size();
  DRAKE_DEMAND(i < size);
  for (size_t hole = i; (hole + 1) < size; ++hole) {
    (*vec)[hole] = std::move((*vec)[hole + 1]);
  }
  vec->pop_back();
}

}  // namespace

template <typename T>
DiagramBuilder<T>::DiagramBuilder() {}

template <typename T>
DiagramBuilder<T>::~DiagramBuilder() {}

template <typename T>
void DiagramBuilder<T>::RemoveSystem(const System<T>& system) {
  ThrowIfAlreadyBuilt();
  if (!systems_.contains(&system)) {
    throw std::logic_error(fmt::format(
        "Cannot RemoveSystem on {} because it has not been added to this "
        "DiagramBuilder",
        system.GetSystemPathname()));
  }
  const size_t system_index = std::distance(
      registered_systems_.begin(),
      std::find_if(registered_systems_.begin(), registered_systems_.end(),
                   [&system](const std::shared_ptr<System<T>>& item) {
                     return item.get() == &system;
                   }));
  DRAKE_DEMAND(system_index < registered_systems_.size());

  // Un-export any input ports associated with this system.
  // First, undo the ConnectInput.
  std::set<std::string> disconnected_diagram_input_port_names;
  for (size_t i = 0; i < input_port_ids_.size();) {
    const InputPortLocator& locator = input_port_ids_[i];
    if (locator.first == &system) {
      const size_t num_erased = diagram_input_set_.erase(locator);
      DRAKE_DEMAND(num_erased == 1);
      disconnected_diagram_input_port_names.insert(
          std::move(input_port_names_[i]));
      VectorErase(&input_port_ids_, i);
      VectorErase(&input_port_names_, i);
    } else {
      ++i;
    }
  }
  // Second, undo the DeclareInput (iff it was the last connected system).
  for (const auto& name : disconnected_diagram_input_port_names) {
    const bool num_connections =
        std::count(input_port_names_.begin(), input_port_names_.end(), name);
    if (num_connections == 0) {
      const auto iter = diagram_input_indices_.find(name);
      DRAKE_DEMAND(iter != diagram_input_indices_.end());
      const InputPortIndex removed_index = iter->second;
      VectorErase(&diagram_input_data_, removed_index);
      diagram_input_indices_.erase(iter);
      for (auto& [_, index] : diagram_input_indices_) {
        if (index > removed_index) {
          --index;
        }
      }
    }
  }

  // Un-export any output ports associated with this system.
  for (OutputPortIndex i{0}; i < output_port_ids_.size();) {
    const OutputPortLocator& locator = output_port_ids_[i];
    if (locator.first == &system) {
      VectorErase(&output_port_ids_, i);
      VectorErase(&output_port_names_, i);
    } else {
      ++i;
    }
  }

  // Disconnect any internal connections associated with this system.
  for (auto iter = connection_map_.begin(); iter != connection_map_.end();) {
    const auto& [input_locator, output_locator] = *iter;
    if ((input_locator.first == &system) || (output_locator.first == &system)) {
      iter = connection_map_.erase(iter);
    } else {
      ++iter;
    }
  }

  // Delete the system.
  systems_.erase(&system);
  VectorErase(&registered_systems_, system_index);

  DRAKE_ASSERT_VOID(CheckInvariants());
}

template <typename T>
std::vector<const System<T>*> DiagramBuilder<T>::GetSystems() const {
  ThrowIfAlreadyBuilt();
  std::vector<const System<T>*> result;
  result.reserve(registered_systems_.size());
  for (const auto& system : registered_systems_) {
    result.push_back(system.get());
  }
  return result;
}

template <typename T>
std::vector<System<T>*> DiagramBuilder<T>::GetMutableSystems() {
  ThrowIfAlreadyBuilt();
  std::vector<System<T>*> result;
  result.reserve(registered_systems_.size());
  for (const auto& system : registered_systems_) {
    result.push_back(system.get());
  }
  return result;
}

template <typename T>
bool DiagramBuilder<T>::HasSubsystemNamed(std::string_view name) const {
  for (const auto& child : registered_systems_) {
    if (child->get_name() == name) {
      return true;
    }
  }
  return false;
}

template <typename T>
const System<T>& DiagramBuilder<T>::GetSubsystemByName(
    std::string_view name) const {
  ThrowIfAlreadyBuilt();
  const System<T>* result = nullptr;
  for (const auto& child : registered_systems_) {
    if (child->get_name() == name) {
      if (result != nullptr) {
        throw std::logic_error(fmt::format(
            "DiagramBuilder contains multiple subsystems named {} so cannot "
            "provide a unique answer to a lookup by name",
            name));
      }
      result = child.get();
      // We can't return early here because we need to check the whole list
      // for duplicate names.
    }
  }
  if (result != nullptr) {
    return *result;
  }
  std::vector<std::string> valid_subsystems;
  for (const auto& child : registered_systems_) {
    valid_subsystems.push_back(child->get_name());
  }
  throw std::logic_error(fmt::format(
      "DiagramBuilder does not contain a subsystem named {}. Valid subsystems "
      "are {}.",
      name, fmt::join(valid_subsystems, ", ")));
}

template <typename T>
System<T>& DiagramBuilder<T>::GetMutableSubsystemByName(std::string_view name) {
  ThrowIfAlreadyBuilt();
  return const_cast<System<T>&>(GetSubsystemByName(name));
}

template <typename T>
const std::map<typename DiagramBuilder<T>::InputPortLocator,
               typename DiagramBuilder<T>::OutputPortLocator>&
DiagramBuilder<T>::connection_map() const {
  ThrowIfAlreadyBuilt();
  return connection_map_;
}

template <typename T>
void DiagramBuilder<T>::Connect(const OutputPort<T>& src,
                                const InputPort<T>& dest) {
  ThrowIfAlreadyBuilt();
  InputPortLocator dest_id{&dest.get_system(), dest.get_index()};
  OutputPortLocator src_id{&src.get_system(), src.get_index()};
  ThrowIfSystemNotRegistered(&src.get_system());
  ThrowIfSystemNotRegistered(&dest.get_system());
  ThrowIfInputAlreadyWired(dest_id);
  if (src.get_data_type() != dest.get_data_type()) {
    throw std::logic_error(fmt::format(
        "DiagramBuilder::Connect: Cannot mix vector-valued and abstract-"
        "valued ports while connecting output port {} of System {} to "
        "input port {} of System {}",
        src.get_name(), src.get_system().get_name(), dest.get_name(),
        dest.get_system().get_name()));
  }
  if ((src.get_data_type() != kAbstractValued) && (src.size() != dest.size())) {
    throw std::logic_error(fmt::format(
        "DiagramBuilder::Connect: Mismatched vector sizes while connecting "
        "output port {} of System {} (size {}) to "
        "input port {} of System {} (size {})",
        src.get_name(), src.get_system().get_name(), src.size(),
        dest.get_name(), dest.get_system().get_name(), dest.size()));
  }
  if (src.get_data_type() == kAbstractValued) {
    auto model_output = src.Allocate();
    auto model_input = dest.get_system().AllocateInputAbstract(dest);
    const std::type_info& output_type = model_output->static_type_info();
    const std::type_info& input_type = model_input->static_type_info();
    if (output_type != input_type) {
      throw std::logic_error(fmt::format(
          "DiagramBuilder::Connect: Mismatched value types while connecting "
          "output port {} of System {} (type {}) to "
          "input port {} of System {} (type {})",
          src.get_name(), src.get_system().get_name(),
          NiceTypeName::Get(output_type), dest.get_name(),
          dest.get_system().get_name(), NiceTypeName::Get(input_type)));
    }
  }
  connection_map_[dest_id] = src_id;
}

template <typename T>
void DiagramBuilder<T>::Connect(const System<T>& src, const System<T>& dest) {
  ThrowIfAlreadyBuilt();
  Connect(src.get_output_port(), dest.get_input_port());
}

template <typename T>
void DiagramBuilder<T>::Cascade(const System<T>& src, const System<T>& dest) {
  ThrowIfAlreadyBuilt();
  Connect(src, dest);
}

template <typename T>
InputPortIndex DiagramBuilder<T>::ExportInput(
    const InputPort<T>& input, std::variant<std::string, UseDefaultName> name) {
  ThrowIfAlreadyBuilt();
  const InputPortIndex diagram_port_index = DeclareInput(input, name);
  ConnectInput(diagram_port_index, input);
  return diagram_port_index;
}

template <typename T>
InputPortIndex DiagramBuilder<T>::DeclareInput(
    const InputPort<T>& input, std::variant<std::string, UseDefaultName> name) {
  InputPortLocator id{&input.get_system(), input.get_index()};
  ThrowIfSystemNotRegistered(&input.get_system());

  // The requirement that subsystem names are unique guarantees uniqueness
  // of the port names.
  std::string port_name =
      name == kUseDefaultName
          ? input.get_system().get_name() + "_" + input.get_name()
          : std::get<std::string>(std::move(name));
  DRAKE_DEMAND(!port_name.empty());

  // Reject duplicate declarations.
  if (diagram_input_indices_.contains(port_name)) {
    throw std::logic_error(
        fmt::format("Diagram already has an input port named {}", port_name));
  }

  // Save bookkeeping data.
  const auto return_id = InputPortIndex(diagram_input_data_.size());
  diagram_input_indices_[port_name] = return_id;
  diagram_input_data_.push_back({id, std::move(port_name)});
  return return_id;
}

template <typename T>
void DiagramBuilder<T>::ConnectInput(std::string_view diagram_port_name,
                                     const InputPort<T>& input) {
  ThrowIfAlreadyBuilt();
  DRAKE_THROW_UNLESS(diagram_input_indices_.count(diagram_port_name));
  const InputPortIndex diagram_port_index =
      diagram_input_indices_.find(diagram_port_name)->second;
  ConnectInput(diagram_port_index, input);
}

template <typename T>
void DiagramBuilder<T>::ConnectInput(InputPortIndex diagram_port_index,
                                     const InputPort<T>& input) {
  ThrowIfAlreadyBuilt();
  InputPortLocator id{&input.get_system(), input.get_index()};
  ThrowIfInputAlreadyWired(id);
  ThrowIfSystemNotRegistered(&input.get_system());
  DRAKE_THROW_UNLESS(diagram_port_index <
                     InputPortIndex(diagram_input_data_.size()));

  // Check that port types match.
  const ExportedInputData& data = diagram_input_data_[diagram_port_index];
  const InputPortLocator& model_id = data.model_input;
  const std::string& port_name = data.name;
  const InputPort<T>& model = model_id.first->get_input_port(model_id.second);
  if (model.get_data_type() != input.get_data_type()) {
    throw std::logic_error(fmt::format(
        "DiagramBuilder::ConnectInput: Cannot mix vector-valued and abstract-"
        "valued ports while connecting input port {} of System {} to "
        "input port {} of Diagram",
        input.get_name(), input.get_system().get_name(), port_name));
  }
  if ((model.get_data_type() != kAbstractValued) &&
      (model.size() != input.size())) {
    throw std::logic_error(fmt::format(
        "DiagramBuilder::ConnectInput: Mismatched vector sizes while "
        "connecting input port {} of System {} (size {}) to "
        "input port {} of Diagram (size {})",
        input.get_name(), input.get_system().get_name(), input.size(),
        port_name, model.size()));
  }
  if (model.get_data_type() == kAbstractValued) {
    auto model_model = model.get_system().AllocateInputAbstract(model);
    auto model_input = input.get_system().AllocateInputAbstract(input);
    const std::type_info& model_type = model_model->static_type_info();
    const std::type_info& input_type = model_input->static_type_info();
    if (model_type != input_type) {
      throw std::logic_error(fmt::format(
          "DiagramBuilder::ConnectInput: Mismatched value types while "
          "connecting input port {} of System {} (type {}) to "
          "input port {} of Diagram (type {})",
          input.get_name(), input.get_system().get_name(),
          NiceTypeName::Get(input_type), port_name,
          NiceTypeName::Get(model_type)));
    }
  }

  // Write down connection information.
  input_port_ids_.push_back(id);
  input_port_names_.push_back(port_name);
  diagram_input_set_.insert(id);
}

template <typename T>
bool DiagramBuilder<T>::ConnectToSame(const InputPort<T>& exemplar,
                                      const InputPort<T>& dest) {
  ThrowIfAlreadyBuilt();
  ThrowIfSystemNotRegistered(&exemplar.get_system());
  ThrowIfSystemNotRegistered(&dest.get_system());
  InputPortLocator dest_id{&dest.get_system(), dest.get_index()};
  ThrowIfInputAlreadyWired(dest_id);

  // Check if `exemplar` was connected.
  InputPortLocator exemplar_id{&exemplar.get_system(), exemplar.get_index()};
  const auto iter = connection_map_.find(exemplar_id);
  if (iter != connection_map_.end()) {
    const OutputPortLocator& exemplar_loc = iter->second;
    const OutputPort<T>& exemplar_port =
        exemplar_loc.first->get_output_port(exemplar_loc.second);
    Connect(exemplar_port, dest);
    return true;
  }

  // Check if `exemplar` was exported.
  if (diagram_input_set_.contains(exemplar_id)) {
    for (size_t i = 0; i < input_port_ids_.size(); ++i) {
      if (input_port_ids_[i] == exemplar_id) {
        ConnectInput(input_port_names_[i], dest);
        return true;
      }
    }
    DRAKE_UNREACHABLE();
  }

  // The `exemplar` input was neither connected nor exported.
  return false;
}

template <typename T>
OutputPortIndex DiagramBuilder<T>::ExportOutput(
    const OutputPort<T>& output,
    std::variant<std::string, UseDefaultName> name) {
  ThrowIfAlreadyBuilt();
  ThrowIfSystemNotRegistered(&output.get_system());
  OutputPortIndex return_id(output_port_ids_.size());
  output_port_ids_.push_back(
      OutputPortLocator{&output.get_system(), output.get_index()});

  // The requirement that subsystem names are unique guarantees uniqueness
  // of the port names.
  std::string port_name =
      name == kUseDefaultName
          ? output.get_system().get_name() + "_" + output.get_name()
          : std::get<std::string>(std::move(name));
  DRAKE_DEMAND(!port_name.empty());
  output_port_names_.emplace_back(std::move(port_name));

  return return_id;
}

template <typename T>
void DiagramBuilder<T>::Disconnect(const OutputPort<T>& source,
                                   const InputPort<T>& dest) {
  OutputPortLocator source_id{&source.get_system(), source.get_index()};
  InputPortLocator dest_id{&dest.get_system(), dest.get_index()};
  const auto iter = connection_map_.find(dest_id);
  if (iter != connection_map_.end()) {
    if (iter->second == source_id) {
      connection_map_.erase(iter);
      return;
    }
  }
  throw std::logic_error(fmt::format(
      "DiagramBuilder::Disconnect(source={}, dest={}) error: the ports were "
      "not already connected, so cannot be disconnected.",
      source.get_name(), dest.get_name()));
}

template <typename T>
std::unique_ptr<Diagram<T>> DiagramBuilder<T>::Build() {
  ThrowIfAlreadyBuilt();
  return std::unique_ptr<Diagram<T>>(new Diagram<T>(Compile()));
}

template <typename T>
void DiagramBuilder<T>::BuildInto(Diagram<T>* target) {
  ThrowIfAlreadyBuilt();
  target->Initialize(Compile());
}

template <typename T>
bool DiagramBuilder<T>::IsConnectedOrExported(const InputPort<T>& port) const {
  ThrowIfAlreadyBuilt();
  InputPortLocator id{&port.get_system(), port.get_index()};
  if (this->connection_map_.contains(id) ||
      this->diagram_input_set_.contains(id)) {
    return true;
  }
  return false;
}

template <typename T>
int DiagramBuilder<T>::num_input_ports() const {
  return input_port_names_.size();
}

template <typename T>
int DiagramBuilder<T>::num_output_ports() const {
  return output_port_names_.size();
}

template <typename T>
void DiagramBuilder<T>::ThrowIfAlreadyBuilt() const {
  if (already_built_) {
    throw std::logic_error(
        "DiagramBuilder: Build() or BuildInto() has already been called to "
        "create a Diagram; this DiagramBuilder may no longer be used.");
  }
}

template <typename T>
void DiagramBuilder<T>::AddSystemImpl(std::shared_ptr<System<T>>&& system) {
  DRAKE_THROW_UNLESS(system != nullptr);
  ThrowIfAlreadyBuilt();
  if (system->get_name().empty()) {
    system->set_name(system->GetMemoryObjectName());
  }
  systems_.insert(system.get());
  registered_systems_.push_back(std::move(system));
}

template <typename T>
void DiagramBuilder<T>::AddNamedSystemImpl(
    const std::string& name, std::shared_ptr<System<T>>&& system) {
  DRAKE_THROW_UNLESS(system != nullptr);
  system->set_name(name);
  this->AddSystemImpl(std::move(system));
}

template <typename T>
void DiagramBuilder<T>::ThrowIfInputAlreadyWired(
    const InputPortLocator& id) const {
  if (connection_map_.find(id) != connection_map_.end() ||
      diagram_input_set_.find(id) != diagram_input_set_.end()) {
    // Extract the name of the input port.
    auto iter = std::find(input_port_ids_.begin(), input_port_ids_.end(), id);
    DRAKE_DEMAND(iter != input_port_ids_.end());  // it should always be found.
    int index = std::distance(input_port_ids_.begin(), iter);
    throw std::logic_error(fmt::format("Input port {} is already connected.",
                                       input_port_names_[index]));
  }
}

template <typename T>
void DiagramBuilder<T>::ThrowIfSystemNotRegistered(
    const System<T>* system) const {
  DRAKE_DEMAND(system != nullptr);
  if (!systems_.contains(system)) {
    std::string registered_system_names{};
    for (const auto& sys : registered_systems_) {
      if (!registered_system_names.empty()) {
        registered_system_names += ", ";
      }
      registered_system_names += '\'' + sys->get_name() + '\'';
    }
    if (registered_system_names.empty()) {
      registered_system_names = "NONE";
    }

    throw std::logic_error(fmt::format(
        "DiagramBuilder: System '{}' has not been registered to this "
        "DiagramBuilder using AddSystem nor AddNamedSystem.\n\nThe systems "
        "currently registered to this builder are: {}.\n\nIf '{}' was "
        "registered as a subsystem to one of these, you must export the input "
        "or output port using ExportInput/ExportOutput and then connect to the "
        "exported port.",
        system->get_name(), registered_system_names, system->get_name()));
  }
}

namespace {

using EitherPortIndex = std::variant<InputPortIndex, OutputPortIndex>;

// The PortIdentifier must be appropriate to use in a sorted collection.  Thus,
// we place its two integer indices first, because they form a unique key on
// their own (the variant disambiguates input vs output indices, even though
// their integer values overlap).  The SystemBase* field is supplementary (and
// only used during error reporting).
using PortIdentifier =
    std::tuple<SubsystemIndex, EitherPortIndex, const SystemBase*>;

bool is_input_port(const PortIdentifier& node) {
  const EitherPortIndex& either = std::get<1>(node);
  return either.index() == 0;
}

std::string to_string(const PortIdentifier& port_id) {
  const SystemBase* const system = std::get<2>(port_id);
  const EitherPortIndex& index = std::get<1>(port_id);
  return is_input_port(port_id)
             ? system->get_input_port_base(std::get<0>(index))
                   .GetFullDescription()
             : system->get_output_port_base(std::get<1>(index))
                   .GetFullDescription();
}

// Helper to do the algebraic loop test. It recursively performs the
// depth-first search on the graph to find cycles.
bool HasCycleRecurse(
    const PortIdentifier& n,
    const std::map<PortIdentifier, std::set<PortIdentifier>>& edges,
    std::set<PortIdentifier>* visited, std::vector<PortIdentifier>* stack) {
  DRAKE_ASSERT(!visited->contains(n));
  visited->insert(n);

  auto edge_iter = edges.find(n);
  if (edge_iter != edges.end()) {
    DRAKE_ASSERT(std::find(stack->begin(), stack->end(), n) == stack->end());
    stack->push_back(n);
    for (const auto& target : edge_iter->second) {
      if (!visited->contains(target) &&
          HasCycleRecurse(target, edges, visited, stack)) {
        return true;
      } else if (std::find(stack->begin(), stack->end(), target) !=
                 stack->end()) {
        return true;
      }
    }
    stack->pop_back();
  }
  return false;
}

}  // namespace

template <typename T>
void DiagramBuilder<T>::ThrowIfAlgebraicLoopsExist() const {
  // To discover loops, we will construct a digraph and check it for cycles.

  // The nodes in the digraph are the input and output ports mentioned by the
  // diagram's internal connections.  Ports that are not internally connected
  // cannot participate in a cycle, so we don't include them in the nodes set.
  std::set<PortIdentifier> nodes;

  // The edges in the digraph are a directed "influences" relation: for each
  // `value` in `edges[key]`, the `key` influences `value`.  (This is the
  // opposite of the "depends-on" relation.)
  std::map<PortIdentifier, std::set<PortIdentifier>> edges;

  // Create a lookup table from system pointer to subsystem index.
  std::unordered_map<const SystemBase*, SubsystemIndex> system_to_index;
  for (SubsystemIndex i{0}; i < registered_systems_.size(); ++i) {
    system_to_index.emplace(registered_systems_[i].get(), i);
  }

  // Add the diagram's internal connections to the digraph nodes *and* edges.
  // The output port influences the input port.
  for (const auto& item : connection_map_) {
    const SystemBase* const input_system = item.first.first;
    const InputPortIndex input_index = item.first.second;
    const SystemBase* const output_system = item.second.first;
    const OutputPortIndex output_index = item.second.second;
    const PortIdentifier input{system_to_index.at(input_system), input_index,
                               input_system};
    const PortIdentifier output{system_to_index.at(output_system), output_index,
                                output_system};
    nodes.insert(input);
    nodes.insert(output);
    edges[output].insert(input);
  }

  // Add more edges (*not* nodes) based on each System's direct feedthrough.
  // An input port influences an output port iff there is direct feedthrough
  // from that input to that output.  If a feedthrough edge refers to a port
  // not in `nodes`, we omit it because ports that are not connected inside the
  // diagram cannot participate in a cycle.
  for (const auto& system_ptr : registered_systems_) {
    const SystemBase* const system = system_ptr.get();
    for (const auto& item : system->GetDirectFeedthroughs()) {
      const SubsystemIndex subsystem_index = system_to_index.at(system);
      const PortIdentifier input{subsystem_index, InputPortIndex{item.first},
                                 system};
      const PortIdentifier output{subsystem_index, OutputPortIndex{item.second},
                                  system};
      if (nodes.contains(input) && nodes.contains(output)) {
        edges[input].insert(output);
      }
    }
  }

  static constexpr char kAdvice[] =
      "A System may have conservatively reported that one of its output ports "
      "depends on an input port, making one of the 'is direct-feedthrough to' "
      "lines above spurious.  If that is the case, remove the spurious "
      "dependency per the Drake API documentation for declaring output ports. "
      "https://drake.mit.edu/doxygen_cxx/"
      "classdrake_1_1systems_1_1_leaf_system.html"
      "#DeclareLeafOutputPort_feedthrough";

  // Evaluate the graph for cycles.
  std::set<PortIdentifier> visited;
  std::vector<PortIdentifier> stack;
  for (const auto& node : nodes) {
    if (visited.contains(node)) {
      continue;
    }
    if (HasCycleRecurse(node, edges, &visited, &stack)) {
      std::stringstream message;
      message << "Reported algebraic loop detected in DiagramBuilder:\n";
      for (const auto& item : stack) {
        message << "  " << to_string(item);
        if (is_input_port(item)) {
          message << " is direct-feedthrough to\n";
        } else {
          message << " is connected to\n";
        }
      }
      message << "  " << to_string(stack.front()) << "\n";
      message << kAdvice;
      throw std::runtime_error(message.str());
    }
  }
}

template <typename T>
void DiagramBuilder<T>::CheckInvariants() const {
  auto has_system = [this](const System<T>* system) {
    return systems_.contains(system);
  };

  // The systems_ and registered_systems_ are identical sets.
  DRAKE_DEMAND(systems_.size() == registered_systems_.size());
  for (const auto& item : registered_systems_) {
    DRAKE_DEMAND(has_system(item.get()));
  }

  // The connection_map_ only refers to registered systems.
  for (const auto& [input, output] : connection_map_) {
    DRAKE_DEMAND(has_system(input.first));
    DRAKE_DEMAND(has_system(output.first));
  }

  // The input_port_ids_ and output_port_ids_ only refer to registered systems.
  for (const auto& [system, _] : input_port_ids_) {
    DRAKE_DEMAND(has_system(system));
  }
  for (const auto& [system, _] : output_port_ids_) {
    DRAKE_DEMAND(has_system(system));
  }

  // The input_port_ids_ and diagram_input_set_ are identical sets.
  DRAKE_DEMAND(input_port_ids_.size() == diagram_input_set_.size());
  for (const auto& item : input_port_ids_) {
    DRAKE_DEMAND(diagram_input_set_.find(item) != diagram_input_set_.end());
  }

  // The diagram_input_indices_ is the inverse of diagram_input_data_.
  DRAKE_DEMAND(diagram_input_data_.size() == diagram_input_indices_.size());
  for (const auto& [name, index] : diagram_input_indices_) {
    DRAKE_DEMAND(diagram_input_data_.at(index).name == name);
  }
}

template <typename T>
std::unique_ptr<typename Diagram<T>::Blueprint> DiagramBuilder<T>::Compile() {
  if (registered_systems_.size() == 0) {
    throw std::logic_error("Cannot Compile an empty DiagramBuilder.");
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
  ThrowIfAlgebraicLoopsExist();

  auto blueprint = std::make_unique<typename Diagram<T>::Blueprint>();
  blueprint->input_port_ids = input_port_ids_;
  blueprint->input_port_names = input_port_names_;
  blueprint->output_port_ids = output_port_ids_;
  blueprint->output_port_names = output_port_names_;
  blueprint->connection_map = connection_map_;
  blueprint->systems = std::move(registered_systems_);
  blueprint->life_support = std::move(life_support_);

  already_built_ = true;

  return blueprint;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramBuilder);
