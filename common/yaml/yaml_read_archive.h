#pragma once

#include <algorithm>
#include <array>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <fmt/format.h>
#include "yaml-cpp/yaml.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/name_value.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace yaml {

/// Loads data from a YAML file into a C++ structure, using the Serialize /
/// Archive pattern.
///
/// Sample data:
/// @code{yaml}
/// doc:
///   foo: 1.0
///   bar: [2.0, 3.0]
/// @endcode
///
/// Sample code:
/// @code{cpp}
/// struct MyData {
///   double foo{NAN};
///   std::vector<double> bar;
///
///   template <typename Archive>
///   void Serialize(Archive* a) {
///     a->Visit(DRAKE_NVP(foo));
///     a->Visit(DRAKE_NVP(bar));
///   }
/// };
///
/// MyData LoadData(const std::string& filename) {
///   MyData result;
///   const YAML::Node& root = YAML::LoadFile(filename);
///   common::YamlReadArchive(root).Accept(&result);
///   return result;
/// }
/// @endcode
///
/// Structures can be arbitrarily nested, as long as each `struct` has a
/// `Serialize` method.  Many common built-in types (int, double, std::string,
/// std::vector, std::array, std::optional, std::variant, Eigen::Matrix) may
/// also be used.
///
/// YAML's "merge keys" (https://yaml.org/type/merge.html) are supported.
///
/// For inspiration and background, see:
/// https://www.boost.org/doc/libs/release/libs/serialization/doc/tutorial.html
class YamlReadArchive final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(YamlReadArchive)

  /// Creates an archive that reads from @p root.  See the %YamlReadArchive
  /// class overview for details.
  explicit YamlReadArchive(const YAML::Node& root)
      : owned_root_(root),
        root_(&owned_root_),
        mapish_item_key_(nullptr),
        mapish_item_value_(nullptr),
        parent_(nullptr) {
    // Reprocess the owned_root for merge keys only after all member fields are
    // initialized; otherwise, the method might access invalid member data.
    RewriteMergeKeys(const_cast<YAML::Node*>(&owned_root_));
  }

  /// Sets the contents `serializable` based on the YAML file associated with
  /// this archive.  See the %YamlReadArchive class overview for details.
  template <typename Serializable>
  void Accept(Serializable* serializable) {
    if (!has_root()) {
      // TODO(jwnimmer-tri) This should probably be a ReportMissingYaml error.
      return;
    }
    DoAccept(this, serializable, static_cast<int32_t>(0));
  }

  /// (Advanced.)  Sets the value pointed to by `nvp.value()` based on the YAML
  /// file associated with this archive.  Most users should should call Accept,
  /// not Visit.
  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp) {
    this->Visit(nvp, VisitShouldMemorizeType::kYes);
  }

 private:
  // N.B. In the private details below, we use "NVP" to abbreviate the
  // "NameValuePair" template concept.

  // Internal-use constructor during recursion.  This constructor aliases all
  // of its arguments, so all must outlive this object.
  YamlReadArchive(const YAML::Node* root, const YamlReadArchive* parent)
      : owned_root_(),
        root_(root),
        mapish_item_key_(nullptr),
        mapish_item_value_(nullptr),
        parent_(parent) {
    DRAKE_DEMAND(root != nullptr);
    DRAKE_DEMAND(parent != nullptr);
  }

  // Internal-use constructor during recursion.  This constructor aliases all
  // of its arguments, so all must outlive this object.  The effect is as-if
  // we have a root of type NodeType::Map with a single (key, value) entry.
  YamlReadArchive(const char* mapish_item_key,
                  const YAML::Node* mapish_item_value,
                  const YamlReadArchive* parent)
      : owned_root_(),
        root_(nullptr),
        mapish_item_key_(mapish_item_key),
        mapish_item_value_(mapish_item_value),
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
  template <typename Archive, typename Serializable>
  auto DoAccept(Archive* a, Serializable* serializable, int32_t) ->
      decltype(serializable->Serialize(a)) {
    serializable->Serialize(a);
  }

  // This version applies when Serialize is an ADL free function.
  template <typename Archive, typename Serializable>
  void DoAccept(Archive* a, Serializable* serializable, int64_t) {
    Serialize(a, serializable);
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

  // For Eigen::Vector.
  template <typename NVP, typename T, int Rows>
  void DoVisit(const NVP& nvp, const Eigen::Matrix<T, Rows, 1>&, int32_t) {
    if (Rows >= 0) {
      this->VisitArray(nvp.name(), Rows, nvp.value()->data());
    } else {
      this->VisitVector(nvp);
    }
  }

  // For Eigen::Matrix.
  template <typename NVP, typename T, int Rows, int Cols>
  void DoVisit(const NVP& nvp, const Eigen::Matrix<T, Rows, Cols>&, int32_t) {
    this->VisitMatrix(nvp.name(), nvp.value());
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
    const auto& sub_node = GetSubNode(nvp.name(), YAML::NodeType::Map);
    if (!sub_node) { return; }
    YamlReadArchive sub_archive(&sub_node, this);
    auto&& value = *nvp.value();
    sub_archive.Accept(&value);
  }

  template <typename NVP>
  void VisitScalar(const NVP& nvp) {
    const auto& sub_node = GetSubNode(nvp.name(), YAML::NodeType::Scalar);
    if (!sub_node) { return; }
    // TODO(jwnimmer-tri) Add better reporting of type errors here.
    using T = typename NVP::value_type;
    *nvp.value() = sub_node.template as<T>();
  }

  template <typename NVP>
  void VisitOptional(const NVP& nvp) {
    // When visiting an optional, it's fine if the YAML node is either absent
    // or has an empty value.  In yaml-cpp, presence is denoted by IsDefined(),
    // and empty is denoted by IsNull().
    const auto& sub_node = MaybeGetSubNode(nvp.name());
    if (!sub_node.IsDefined() || sub_node.IsNull()) {
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
    const YAML::Node sub_node = MaybeGetSubNode(nvp.name());
    if (!sub_node) {
      ReportMissingYaml("is missing");
      return;
    }
    // Figure out which variant<...> type we have based on the node's tag.
    const std::string& tag = sub_node.Tag();
    VariantHelper(tag, nvp.name(), nvp.value());
  }

  // Steps through Types to extract 'size_t I' and 'typename T' for the Impl.
  template <template <typename...> class Variant, typename... Types>
  void VariantHelper(
      const std::string& tag, const char* name, Variant<Types...>* storage) {
    VariantHelperImpl<0, Variant<Types...>, Types...>(tag, name, storage);
  }

  // Recursive case -- checks if `tag` matches `T` (which was the I'th type in
  // the template parameter pack), or else keeps looking.
  template <size_t I, typename Variant, typename T, typename... Remaining>
  void VariantHelperImpl(
      const std::string& tag, const char* name, Variant* storage) {
    if (((I == 0) && (tag.empty() || (tag == "?"))) ||
        IsTagMatch(drake::NiceTypeName::Get<T>(), tag)) {
      T typed_storage{};
      this->Visit(drake::MakeNameValue(name, &typed_storage));
      storage->template emplace<I>(std::move(typed_storage));
      return;
    }
    VariantHelperImpl<I + 1, Variant, Remaining...>(tag, name, storage);
  }

  // Base case -- no match.
  template <size_t, typename Variant>
  void VariantHelperImpl(const std::string& tag, const char*, Variant*) {
    ReportMissingYaml(fmt::format(
        "has unsupported type tag {} while selecting a variant<>",
        tag));
  }

  // Checks if a NiceTypeName matches the yaml type tag.
  bool IsTagMatch(const std::string& name, const std::string& tag) const {
    // Check for the "fail safe schema" YAML types and similar.
    if (name == "std::string") {
      return tag == "tag:yaml.org,2002:str";
    }
    if (name == "double") {
      return tag == "tag:yaml.org,2002:float";
    }
    if (name == "int") {
      return tag == "tag:yaml.org,2002:int";
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
    const auto& sub_node = GetSubNode(name, YAML::NodeType::Sequence);
    if (!sub_node) { return; }
    if (sub_node.size() != size) {
      ReportMissingYaml(fmt::format(
          "has {}-size entry (wanted {}-size)",
          sub_node.size(), size));
    }
    for (size_t i = 0; i < size; ++i) {
      const std::string key = fmt::format("{}[{}]", name, i);
      const YAML::Node value = sub_node[i];
      YamlReadArchive item_archive(key.c_str(), &value, this);
      item_archive.Visit(drake::MakeNameValue(key.c_str(), &data[i]));
    }
  }

  template <typename NVP>
  void VisitVector(const NVP& nvp) {
    const auto& sub_node = GetSubNode(nvp.name(), YAML::NodeType::Sequence);
    if (!sub_node) { return; }
    size_t size = sub_node.size();
    auto&& storage = *nvp.value();
    storage.resize(size);
    if (size > 0) {
      this->VisitArray(nvp.name(), size, &storage[0]);
    }
  }

  template <typename T, int Rows, int Cols>
  void VisitMatrix(const char* name, Eigen::Matrix<T, Rows, Cols>* matrix) {
    const auto& sub_node = GetSubNode(name, YAML::NodeType::Sequence);
    if (!sub_node) { return; }

    // Measure the YAML Sequence-of-Sequence dimensions.
    const size_t rows = sub_node.size();
    const size_t cols = sub_node[0].size();
    for (size_t i = 0; i < rows; ++i) {
      const YAML::Node one_row = sub_node[i];
      const size_t one_row_size = one_row.size();
      if (one_row.Type() != YAML::NodeType::Sequence) {
        ReportMissingYaml(fmt::format(
            "is Sequence-of-{} (not Sequence-of-Sequence)",
            to_string(one_row.Type())));
        return;
      }
      if (one_row_size != cols) {
        ReportMissingYaml("has inconsistent cols dimensions");
        return;
      }
    }

    // Check the YAML dimensions vs Eigen dimensions, then resize (if dynamic).
    if (((Rows != Eigen::Dynamic) && (static_cast<int>(rows) != Rows)) ||
        ((Cols != Eigen::Dynamic) && (static_cast<int>(cols) != Cols))) {
      ReportMissingYaml(fmt::format("has dimension {}x{} (wanted {}x{})",
                                    rows, cols, Rows, Cols));
      return;
    }
    auto&& storage = *matrix;
    storage.resize(rows, cols);

    // Parse.
    for (size_t i = 0; i < rows; ++i) {
      for (size_t j = 0; j < cols; ++j) {
        const std::string key = fmt::format("{}[{}][{}]", name, i, j);
        const YAML::Node value = sub_node[i][j];
        YamlReadArchive item_archive(key.c_str(), &value, this);
        item_archive.Visit(drake::MakeNameValue(key.c_str(), &storage(i, j)));
      }
    }
  }

  template <typename Key, typename Value, typename NVP>
  void VisitMap(const NVP& nvp) {
    static_assert(std::is_same<Key, std::string>::value,
                  "std::map keys must be strings");
    const auto& sub_node = GetSubNode(nvp.name(), YAML::NodeType::Map);
    if (!sub_node) { return; }
    auto& result = *nvp.value();
    result.clear();
    for (const auto& yaml_key_value : sub_node) {
      const std::string& key = yaml_key_value.first.Scalar();
      auto newiter_inserted = result.emplace(key, Value{});
      auto& newiter = newiter_inserted.first;
      const bool inserted = newiter_inserted.second;
      DRAKE_DEMAND(inserted == true);
      Value& newvalue = newiter->second;
      YamlReadArchive item_archive(&sub_node, this);
      item_archive.Visit(drake::MakeNameValue(key.c_str(), &newvalue));
    }
  }

  // --------------------------------------------------------------------------
  // @name Helpers, utilities, and member variables.

  // Do we have a root Node?
  bool has_root() const;

  // Move the merge key values (if any) into the given node using the merge key
  // semantics; see https://yaml.org/type/merge.html for details.  If yaml-cpp
  // adds native support for merge keys, then we should remove this helper.
  void RewriteMergeKeys(YAML::Node*) const;

  // If our root is a Map and has child with the given name and type, return
  // the child.  Otherwise, report an error and return an undefined node.
  YAML::Node GetSubNode(const char*, YAML::NodeType::value) const;

  // If our root is a Map and has child with the given name and type, return
  // the child.  Otherwise, return an undefined node.
  YAML::Node MaybeGetSubNode(const char*) const;

  void ReportMissingYaml(const std::string&) const;
  void PrintNodeSummary(std::ostream& s) const;
  void PrintVisitNameType(std::ostream& s) const;
  static const char* to_string(YAML::NodeType::value);

  // These jointly denote the YAML::Node root that our Accept() will read from.
  // For performance reasons, we'll use a few different members all for the
  // same purpose.  Never access these directly from Visit methods -- use
  // GetSubNode() instead.
  // @{
  // A copy of the root node provided by the user to our public constructor.
  // Our recursive calls to private constructors leave this unset.
  const YAML::Node owned_root_;
  // Typically set to alias the Node that Accept() will read from.  If set,
  // this will alias owned_root_ when using the public constructor, or else a
  // temporary root when using the private recursion constructor.  Only ever
  // unset during internal recursion when mapish_item_{key,value}_ are being
  // used instead.
  const YAML::Node* const root_;
  // During certain cases of internal recursion, instead of creating a Map node
  // with only a single key-value pair as the root_ pointer, instead we'll pass
  // the key-value pointers directly.  This avoids the copying associated with
  // constructing a new Map node.  The root_ vs key_,value_ representations are
  // mutially exclusive -- when root_ is null, both the key_ and value_ must be
  // non-null, and when root_ is non-null, both key_,value_ must be null.
  const char* const mapish_item_key_;
  const YAML::Node* const mapish_item_value_;
  // @}

  // These are only used for error messages.  The two `debug_...` members are
  // non-nullptr only during Visit()'s lifetime.
  const YamlReadArchive* const parent_;
  const char* debug_visit_name_{};
  const std::type_info* debug_visit_type_{};
};

}  // namespace yaml
}  // namespace drake
