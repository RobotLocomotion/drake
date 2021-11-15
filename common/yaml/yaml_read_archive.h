#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <fmt/format.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/common/name_value.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/unused.h"
#include "drake/common/yaml/yaml_node.h"

// Forward-declaration from "yaml-cpp/yaml.h".
// TODO(jwnimmer-tri) Remove these on 2022-03-01 when the deprecated YAML::Node
// functions in this file are also removed.
namespace YAML {
class Node;
template <typename T> struct convert;
}  // namespace YAML

namespace drake {
namespace yaml {

/// (Advanced) A helper class for @ref yaml_serialization "YAML Serialization"
/// that loads data from a YAML file into a C++ structure.
class YamlReadArchive final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(YamlReadArchive)

  /// Configuration for YamlReadArchive to govern when certain conditions are
  /// errors or not.  Refer to the member fields for details.
  struct Options {
    friend std::ostream& operator<<(std::ostream& os, const Options& x);

    /// Allows yaml Maps to have extra key-value pairs that are not Visited by
    /// the Serializable being parsed into.  In other words, the Serializable
    /// types provide an incomplete schema for the YAML data.  This allows for
    /// parsing only a subset of the YAML data.
    bool allow_yaml_with_no_cpp{false};

    /// Allows Serializables to provide more key-value pairs than are present
    /// in the YAML data.  In other words, the structs have default values that
    /// are left intact unless the YAML data provides a value.
    bool allow_cpp_with_no_yaml{false};

    /// If set to true, when parsing a std::map the Archive will merge the YAML
    /// data into the destination, instead of replacing the std::map contents
    /// entirely.  In other words, a visited std::map can have default values
    /// that are left intact unless the YAML data provides a value *for that
    /// specific key*.
    bool retain_map_defaults{false};
  };

  /// (Deprecated) Creates an archive that reads from @p root.
  /// Prefer to use the functions in yaml_io.h, instead.
  DRAKE_DEPRECATED("2022-03-01", "Use LoadYamlFile or LoadYamlString instead.")
  explicit YamlReadArchive(const YAML::Node& root);

  /// (Deprecated) Creates an archive that reads from @p root,
  /// with @p options that allow for less restrictive parsing.
  /// Prefer to use the functions in yaml_io.h, instead.
  DRAKE_DEPRECATED("2022-03-01", "Use LoadYamlFile or LoadYamlString instead.")
  YamlReadArchive(const YAML::Node& root, const Options& options);

  /// (Internal use only.)
  YamlReadArchive(internal::Node root, const Options& options);

  /// (Internal use only.)
  static internal::Node LoadFileAsNode(
      const std::string& filename,
      const std::optional<std::string>& child_name);

  /// (Internal use only.)
  static internal::Node LoadStringAsNode(
      const std::string& data,
      const std::optional<std::string>& child_name);

  /// (Advanced) Sets the contents `serializable` based on the YAML file
  /// associated this archive.
  template <typename Serializable>
  void Accept(Serializable* serializable) {
    DRAKE_THROW_UNLESS(serializable != nullptr);
    this->DoAccept(serializable, static_cast<int32_t>(0));
    CheckAllAccepted();
  }

  /// (Advanced) Sets the value pointed to by `nvp.value()` based on the YAML
  /// file associated with this archive.  Most users should call Accept, not
  /// Visit.
  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp) {
    this->Visit(nvp, VisitShouldMemorizeType::kYes);
  }

 private:
  // N.B. In the private details below, we use "NVP" to abbreviate the
  // "NameValuePair" template concept.

  // Internal-use constructor during recursion.  This constructor aliases all
  // of its arguments, so all must outlive this object.
  YamlReadArchive(const internal::Node* root, const YamlReadArchive* parent)
      : owned_root_(),
        root_(root),
        mapish_item_key_(nullptr),
        mapish_item_value_(nullptr),
        options_(parent->options_),
        parent_(parent) {
    DRAKE_DEMAND(root != nullptr);
    DRAKE_DEMAND(parent != nullptr);
  }

  // Internal-use constructor during recursion.  This constructor aliases all
  // of its arguments, so all must outlive this object.  The effect is as-if
  // we have a root of type NodeType::Mapping with a single (key, value) entry.
  YamlReadArchive(const char* mapish_item_key,
                  const internal::Node* mapish_item_value,
                  const YamlReadArchive* parent)
      : owned_root_(),
        root_(nullptr),
        mapish_item_key_(mapish_item_key),
        mapish_item_value_(mapish_item_value),
        options_(parent->options_),
        parent_(parent) {
    DRAKE_DEMAND(mapish_item_key != nullptr);
    DRAKE_DEMAND(mapish_item_value != nullptr);
    DRAKE_DEMAND(parent != nullptr);
  }

  enum class VisitShouldMemorizeType { kNo, kYes };

  // Like the 1-arg Visit, except that the (private) caller can opt-out of this
  // visit frame appearing in error reports.
  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp, VisitShouldMemorizeType trace) {
    if (trace == VisitShouldMemorizeType::kYes) {
      debug_visit_name_ = nvp.name();
      debug_visit_type_ = &typeid(*nvp.value());
      visited_names_.insert(nvp.name());
    }
    // Use int32_t for the final argument to prefer the specialized overload.
    this->DoVisit(nvp, *nvp.value(), static_cast<int32_t>(0));
    if (trace == VisitShouldMemorizeType::kYes) {
      debug_visit_name_ = nullptr;
      debug_visit_type_ = nullptr;
    }
  }

  // --------------------------------------------------------------------------
  // @name Overloads for the Accept() implementation

  // This version applies when Serialize is member method.
  template <typename Serializable>
  auto DoAccept(Serializable* serializable, int32_t) ->
      decltype(serializable->Serialize(this)) {
    serializable->Serialize(this);
  }

  // This version applies when `value` is a std::map from std::string to
  // Serializable.  The map's values must be serializable, but there is no
  // Serialize function required for the map itself.
  template <typename Serializable>
  void DoAccept(std::map<std::string, Serializable>* value, int32_t) {
    DRAKE_THROW_UNLESS(root_ != nullptr);
    DRAKE_THROW_UNLESS(root_->IsMapping());
    VisitMapDirectly<Serializable>(*root_, value);
    for (const auto& [name, ignored] : *value) {
      unused(ignored);
      visited_names_.insert(name);
    }
  }

  // This version applies when Serialize is an ADL free function.
  template <typename Serializable>
  void DoAccept(Serializable* serializable, int64_t) {
    Serialize(this, serializable);
  }

  // --------------------------------------------------------------------------
  // @name Overloads for the Visit() implementation

  // This version applies when the type has a Serialize member function.
  template <typename NVP, typename T>
  auto DoVisit(const NVP& nvp, const T&, int32_t) ->
      decltype(nvp.value()->Serialize(
          static_cast<YamlReadArchive*>(nullptr))) {
    this->VisitSerializable(nvp);
  }

  // This version applies when the type has an ADL Serialize function.
  template <typename NVP, typename T>
  auto DoVisit(const NVP& nvp, const T&, int32_t) ->
      decltype(Serialize(static_cast<YamlReadArchive*>(nullptr), nvp.value())) {
    this->VisitSerializable(nvp);
  }

  // For std::vector.
  template <typename NVP, typename T>
  void DoVisit(const NVP& nvp, const std::vector<T>&, int32_t) {
    this->VisitVector(nvp);
  }

  // For std::array.
  template <typename NVP, typename T, std::size_t N>
  void DoVisit(const NVP& nvp, const std::array<T, N>&, int32_t) {
    this->VisitArray(nvp.name(), N, nvp.value()->data());
  }

  // For std::map.
  template <typename NVP, typename K, typename V, typename C>
  void DoVisit(const NVP& nvp, const std::map<K, V, C>&, int32_t) {
    this->VisitMap<K, V>(nvp);
  }

  // For std::unordered_map.
  template <typename NVP, typename K, typename V, typename H, typename E>
  void DoVisit(const NVP& nvp, const std::unordered_map<K, V, H, E>&, int32_t) {
    this->VisitMap<K, V>(nvp);
  }

  // For std::optional.
  template <typename NVP, typename T>
  void DoVisit(const NVP& nvp, const std::optional<T>&, int32_t) {
    this->VisitOptional(nvp);
  }

  // For std::variant.
  template <typename NVP, typename... Types>
  void DoVisit(const NVP& nvp, const std::variant<Types...>&, int32_t) {
    this->VisitVariant(nvp);
  }

  // For Eigen::Matrix or Eigen::Vector.
  template <typename NVP, typename T, int Rows, int Cols,
      int Options = 0, int MaxRows = Rows, int MaxCols = Cols>
  void DoVisit(const NVP& nvp,
               const Eigen::Matrix<T, Rows, Cols, Options, MaxRows, MaxCols>&,
               int32_t) {
    if constexpr (Cols == 1) {
      if constexpr (Rows >= 0) {
        this->VisitArray(nvp.name(), Rows, nvp.value()->data());
      } else {
        this->VisitVector(nvp);
      }
    } else {
      this->VisitMatrix(nvp.name(), nvp.value());
    }
  }

  // If no other DoVisit matched, we'll treat the value as a scalar.
  template <typename NVP, typename T>
  void DoVisit(const NVP& nvp, const T&, int64_t) {
    this->VisitScalar(nvp);
  }

  // --------------------------------------------------------------------------
  // @name Implementations of Visit() once the shape is known

  template <typename NVP>
  void VisitSerializable(const NVP& nvp) {
    const internal::Node* sub_node = GetSubNodeMapping(nvp.name());
    if (sub_node == nullptr) { return; }
    YamlReadArchive sub_archive(sub_node, this);
    auto&& value = *nvp.value();
    sub_archive.Accept(&value);
  }

  template <typename NVP>
  void VisitScalar(const NVP& nvp) {
    const internal::Node* sub_node = GetSubNodeScalar(nvp.name());
    if (sub_node == nullptr) { return; }
    ParseScalar(sub_node->GetScalar(), nvp.value());
  }

  template <typename NVP>
  void VisitOptional(const NVP& nvp) {
    // When visiting an optional, we want to match up the null-ness of the YAML
    // node with the nullopt-ness of the C++ data.  Refer to the unit tests for
    // Optional for a full explanation.
    const internal::Node* sub_node = MaybeGetSubNode(nvp.name());
    if (sub_node == nullptr) {
      if (!options_.allow_cpp_with_no_yaml) {
        *nvp.value() = std::nullopt;
      }
      return;
    }
    if (sub_node->GetTag() == internal::Node::kTagNull) {
      *nvp.value() = std::nullopt;
      return;
    }

    // Visit the unpacked optional as if it weren't wrapped in optional<>.
    using T = typename NVP::value_type::value_type;
    std::optional<T>& storage = *nvp.value();
    if (!storage) { storage = T{}; }
    this->Visit(drake::MakeNameValue(nvp.name(), &storage.value()),
                VisitShouldMemorizeType::kNo);
  }

  template <typename NVP>
  void VisitVariant(const NVP& nvp) {
    const internal::Node* sub_node = MaybeGetSubNode(nvp.name());
    if (sub_node == nullptr) {
      if (!options_.allow_cpp_with_no_yaml) {
        ReportError("is missing");
      }
      return;
    }
    // Figure out which variant<...> type we have based on the node's tag.
    const std::string& tag = sub_node->GetTag();
    VariantHelper(tag, nvp.name(), nvp.value());
  }

  // Steps through Types to extract 'size_t I' and 'typename T' for the Impl.
  template <template <typename...> class Variant, typename... Types>
  void VariantHelper(
      const std::string& tag, const char* name, Variant<Types...>* storage) {
    if (tag == internal::Node::kTagNull) {
      // Our varaint parsing does not yet support nulls.  When the tag indicates
      // null, don't try to match it to a variant type; instead, just parse into
      // the first variant type in order to generate a useful error message.
      // TODO(jwnimmer-tri) Allow for std::monostate as one of the Types...,
      // in case the user wants to permit nullable variants.
      using T = std::variant_alternative_t<0, Variant<Types...>>;
      T& typed_storage =  storage->template emplace<0>();
      this->Visit(drake::MakeNameValue(name, &typed_storage));
      return;
    }
    VariantHelperImpl<0, Variant<Types...>, Types...>(tag, name, storage);
  }

  // Recursive case -- checks if `tag` matches `T` (which was the I'th type in
  // the template parameter pack), or else keeps looking.
  template <size_t I, typename Variant, typename T, typename... Remaining>
  void VariantHelperImpl(
      const std::string& tag, const char* name, Variant* storage) {
    // For the first type declared in the variant<> (I == 0), the tag can be
    // absent; otherwise, the tag must match one of the variant's types.
    if (((I == 0) && (tag.empty() || (tag == "?"))) ||
        IsTagMatch(drake::NiceTypeName::GetFromStorage<T>(), tag)) {
      T& typed_storage = storage->template emplace<I>();
      this->Visit(drake::MakeNameValue(name, &typed_storage));
      return;
    }
    VariantHelperImpl<I + 1, Variant, Remaining...>(tag, name, storage);
  }

  // Base case -- no match.
  template <size_t, typename Variant>
  void VariantHelperImpl(const std::string& tag, const char*, Variant*) {
    ReportError(fmt::format(
        "has unsupported type tag {} while selecting a variant<>",
        tag));
  }

  // Checks if a NiceTypeName matches the yaml type tag.
  bool IsTagMatch(const std::string& name, const std::string& tag) const {
    // Check for the "fail safe schema" YAML types and similar.
    if (name == "std::string") {
      return tag == internal::Node::kTagStr;
    }
    if (name == "double") {
      return tag == internal::Node::kTagFloat;
    }
    if (name == "int") {
      return tag == internal::Node::kTagInt;
    }

    // Check for an "application specific" tag such as "!MyClass", which we
    // will match to our variant item's name such as "my_namespace::MyClass"
    // ignoring the namespace and any template parameters.
    const auto start_offset = name.rfind(':');
    const auto start = name.begin() + (start_offset == std::string::npos ? 0 :
                                       start_offset + 1);
    const auto end = std::find(start, name.end(), '<');
    return (tag[0] == '!') && std::equal(
        // The `tag` without the leading '!'.
        tag.begin() + 1, tag.end(),
        // The `name` without any namespaces or templates.
        start, end);
  }

  template <typename T>
  void VisitArray(const char* name, size_t size, T* data) {
    const internal::Node* sub_node = GetSubNodeSequence(name);
    if (sub_node == nullptr) { return; }
    const std::vector<internal::Node>& elements = sub_node->GetSequence();
    if (elements.size() != size) {
      ReportError(fmt::format(
          "has {}-size entry (wanted {}-size)",
          elements.size(), size));
    }
    for (size_t i = 0; i < size; ++i) {
      const std::string key = fmt::format("{}[{}]", name, i);
      const internal::Node& value = elements[i];
      YamlReadArchive item_archive(key.c_str(), &value, this);
      item_archive.Visit(drake::MakeNameValue(key.c_str(), &data[i]));
    }
  }

  template <typename NVP>
  void VisitVector(const NVP& nvp) {
    const internal::Node* sub_node = GetSubNodeSequence(nvp.name());
    if (sub_node == nullptr) { return; }
    const std::vector<internal::Node>& elements = sub_node->GetSequence();
    const size_t size = elements.size();
    auto&& storage = *nvp.value();
    storage.resize(size);
    if (size > 0) {
      this->VisitArray(nvp.name(), size, &storage[0]);
    }
  }

  template <typename T, int Rows, int Cols,
      int Options = 0, int MaxRows = Rows, int MaxCols = Cols>
  void VisitMatrix(const char* name,
      Eigen::Matrix<T, Rows, Cols, Options, MaxRows, MaxCols>* matrix) {
    const internal::Node* sub_node = GetSubNodeSequence(name);
    if (sub_node == nullptr) { return; }
    const std::vector<internal::Node>& elements = sub_node->GetSequence();

    // Measure the YAML Sequence-of-Sequence dimensions.
    // Take a guess at what rows & cols will be (we might adjust later).
    size_t pending_rows = elements.size();
    std::optional<size_t> pending_cols;
    for (size_t i = 0; i < pending_rows; ++i) {
      const internal::Node& one_row = elements[i];
      if (!one_row.IsSequence()) {
        ReportError(fmt::format(
            "is Sequence-of-{} (not Sequence-of-Sequence)",
            one_row.GetTypeString()));
        return;
      }
      const size_t one_row_size = one_row.GetSequence().size();
      if (pending_cols && one_row_size != *pending_cols) {
        ReportError("has inconsistent cols dimensions");
        return;
      }
      pending_cols = one_row_size;
    }
    // Never return an Nx0 matrix; demote it to 0x0 instead.
    if (pending_cols.value_or(0) == 0) {
      pending_rows = 0;
      pending_cols = 0;
    }
    const size_t rows = pending_rows;
    const size_t cols = *pending_cols;

    // Check the YAML dimensions vs Eigen dimensions, then resize (if dynamic).
    if (((Rows != Eigen::Dynamic) && (static_cast<int>(rows) != Rows)) ||
        ((Cols != Eigen::Dynamic) && (static_cast<int>(cols) != Cols))) {
      ReportError(fmt::format(
          "has dimension {}x{} (wanted {}x{})", rows, cols, Rows, Cols));
      return;
    }
    auto&& storage = *matrix;
    storage.resize(rows, cols);

    // Parse.
    for (size_t i = 0; i < rows; ++i) {
      for (size_t j = 0; j < cols; ++j) {
        const std::string key = fmt::format("{}[{}][{}]", name, i, j);
        const internal::Node& value = elements[i].GetSequence()[j];
        YamlReadArchive item_archive(key.c_str(), &value, this);
        item_archive.Visit(drake::MakeNameValue(key.c_str(), &storage(i, j)));
      }
    }
  }

  template <typename Key, typename Value, typename NVP>
  void VisitMap(const NVP& nvp) {
    // For now, we only allow std::string as the keys of a serialized std::map.
    // In the future, we could imagine handling any other kind of scalar value
    // that was convertible to a string (int, double, string_view, etc.) if we
    // found that useful.  However, to remain compatible with JSON semantics,
    // we should never allow a YAML Sequence or Mapping to be a used as a key.
    static_assert(std::is_same_v<Key, std::string>,
                  "std::map keys must be strings");
    const internal::Node* sub_node = GetSubNodeMapping(nvp.name());
    if (sub_node == nullptr) { return; }
    auto& result = *nvp.value();
    this->VisitMapDirectly<Value>(*sub_node, &result);
  }

  template <typename Value, typename Map>
  void VisitMapDirectly(const internal::Node& node, Map* result) {
    if (!options_.retain_map_defaults) {
      result->clear();
    }
    for (const auto& [key, value] : node.GetMapping()) {
      unused(value);
      auto newiter_inserted = result->emplace(key, Value{});
      auto& newiter = newiter_inserted.first;
      const bool inserted = newiter_inserted.second;
      if (!options_.retain_map_defaults) {
        DRAKE_DEMAND(inserted == true);
      }
      Value& newvalue = newiter->second;
      YamlReadArchive item_archive(&node, this);
      item_archive.Visit(drake::MakeNameValue(key.c_str(), &newvalue));
    }
  }

  // --------------------------------------------------------------------------
  // @name Scalar parsers

  // These are the only scalar types that Drake supports.
  // Users cannot add de-string-ification functions for custom scalars.
  void ParseScalar(const std::string& value, bool* result);
  void ParseScalar(const std::string& value, float* result);
  void ParseScalar(const std::string& value, double* result);
  void ParseScalar(const std::string& value, int32_t* result);
  void ParseScalar(const std::string& value, uint32_t* result);
  void ParseScalar(const std::string& value, int64_t* result);
  void ParseScalar(const std::string& value, uint64_t* result);
  void ParseScalar(const std::string& value, std::string* result);

  // We use DeprecatedYamlNode here to allow YAML::Node to be forward-declared
  // in the typical case where this function is not called by any users' code.
  // If we mentioned YAML::Node in the function body directly (i.e., without
  // the template argument indirection), then the header file would fail to
  // compile when using Clang 9.
  template <typename T, typename DeprecatedYamlNode = YAML::Node>
  DRAKE_DEPRECATED("2022-03-01",
      "YAML loading only supports scalars of specific primitive types. "
      "Please file a Drake issue if you need any additional types.")
  void ParseScalar(const std::string& value, T* result) {
    DRAKE_DEMAND(result != nullptr);
    // For the decode-able types, see /usr/include/yaml-cpp/node/convert.h.
    // Generally, all of the POD types are supported.
    bool success = YAML::convert<T>::decode(DeprecatedYamlNode(value), *result);
    if (!success) {
      ReportError(fmt::format(
          "could not parse {} value", drake::NiceTypeName::Get<T>()));
    }
  }

  // --------------------------------------------------------------------------
  // @name Helpers, utilities, and member variables.

  // If our root is a Mapping and has child with the given name and type,
  // return the child.  Otherwise, report an error and return nullptr.
  //
  // Currently, errors are always reported via an exception, which means that
  // the nullptr return is irrelevant in practice.  However, in the future we
  // imagine logging multiple errors during parsing (not using exceptions).
  // Therefore, calling code should be written to handle the nullptr case.
  const internal::Node* GetSubNodeScalar(const char* name) const;
  const internal::Node* GetSubNodeSequence(const char* name) const;
  const internal::Node* GetSubNodeMapping(const char* name) const;

  // Helper for the prior three functions.
  const internal::Node* GetSubNodeAny(const char*, internal::NodeType) const;

  // If our root is a Mapping and has child with the given name, return the
  // child.  Otherwise, return nullptr.
  const internal::Node* MaybeGetSubNode(const char*) const;

  // To be called after Accept-ing a Serializable to cross-check that all keys
  // in the YAML root's Mapping matched a Visit call from the Serializable.
  // This relates to the Options.allow_yaml_with_no_cpp setting.
  void CheckAllAccepted() const;

  void ReportError(const std::string&) const;
  void PrintNodeSummary(std::ostream& s) const;
  void PrintVisitNameType(std::ostream& s) const;

  // These jointly denote the Node root that our Accept() will read from.
  // For performance reasons, we'll use a few different members all for the
  // same purpose.  Never access these directly from Visit methods -- use
  // GetSubNodeFoo() instead.
  // @{
  // A copy of the root node provided by the user to our public constructor.
  // Our recursive calls to private constructors leave this unset.
  const std::optional<internal::Node> owned_root_;
  // Typically set to alias the Node that Accept() will read from.  If set,
  // this will alias owned_root_ when using the public constructor, or else a
  // temporary root when using the private recursion constructor.  Only ever
  // unset during internal recursion when mapish_item_{key,value}_ are being
  // used instead.
  const internal::Node* const root_;
  // During certain cases of internal recursion, instead of creating a Mapping
  // node with only a single key-value pair as the root_ pointer, instead we'll
  // pass the key-value pointers directly.  This avoids the copying associated
  // with constructing a new Mapping node.  The two representations are
  // mutually exclusive -- when root_ is null, both the key_ and value_ must
  // be non-null, and when root_ is non-null, both key_,value_ must be null.
  const char* const mapish_item_key_;
  const internal::Node* const mapish_item_value_;
  // @}

  // When the C++ structure and YAML structure disagree, these options govern
  // which mismatches are permitted without an error.
  const Options options_;

  // The set of NameValue::name keys that have been Visited by the current
  // Serializable's Accept method so far.
  std::unordered_set<std::string> visited_names_;

  // These are only used for error messages.  The two `debug_...` members are
  // non-nullptr only during Visit()'s lifetime.
  const YamlReadArchive* const parent_;
  const char* debug_visit_name_{};
  const std::type_info* debug_visit_type_{};
};

}  // namespace yaml
}  // namespace drake
