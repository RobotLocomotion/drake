#pragma once

#include <algorithm>
#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <fmt/format.h>
#include "yaml-cpp/yaml.h"

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/common/drake_variant.h"
#include "drake/common/name_value.h"
#include "drake/common/nice_type_name.h"

namespace anzu {
namespace common {

/// YAML-backed implementation of the Serialize/Archive pattern.
///
/// TODO(jeremy.nimmer) Add documentation of YAML / struct mapping.
/// TODO(jeremy.nimmer) Add much more diagnostics / error reporting.
/// TODO(jeremy.nimmer) Add much more input data test coverage.
class YamlReadArchive final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(YamlReadArchive)

  explicit YamlReadArchive(const YAML::Node& root)
      : YamlReadArchive(root, nullptr) {}

  template <typename Serializable>
  YamlReadArchive& Accept(Serializable* serializable) {
    if (!root_) {
      // TODO(jeremy.nimmer) This should probably be a ReportMissingYaml error.
      return *this;
    }
    DoAccept(this, serializable, static_cast<int32_t>(0));
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp) {
    this->Visit(nvp, VisitShouldMemorizeType::kYes);
  }

 private:
  // N.B. In the private details below, we use "NVP" to abbreviate the
  // "NameValuePair" template concept.

  YamlReadArchive(const YAML::Node& root, const YamlReadArchive* parent)
      : root_(root), parent_(parent) {}

  enum class VisitShouldMemorizeType { kNo, kYes };

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
  /// @name Overloads for the Accept() implementation
  //@{

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

  //@}
  // --------------------------------------------------------------------------
  /// @name Overloads for the Visit() implementation
  //@{

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

  // TODO(jeremy.nimmer) Implement these.
#if 0
  // For std::map.
  template <typename NVP, typename K, typename V, typename C>
  void DoVisit(const NVP& nvp, const std::map<K, V, C>&, int32_t) {
    this->VisitMap(nvp);
  }

  // For std::unordered_map.
  template <typename NVP, typename K, typename V, typename H, typename E>
  void DoVisit(const NVP& nvp, const std::unordered_map<K, V, H, E>&, int32_t) {
    this->VisitMap(nvp);
  }
#endif

  // For drake::optional (which is std::optional iff we have new enough C++).
  template <typename NVP, typename T>
  void DoVisit(const NVP& nvp, const drake::optional<T>&, int32_t) {
    this->VisitOptional(nvp);
  }

  // For drake::variant (which is std::variant iff we have new enough C++).
  template <typename NVP, typename... Types>
  void DoVisit(const NVP& nvp, const drake::variant<Types...>&, int32_t) {
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

  //@}
  // --------------------------------------------------------------------------
  /// @name Implementations of Visit() once the shape is known
  //@{

  template <typename NVP>
  void VisitSerializable(const NVP& nvp) {
    const auto& sub_node = GetSubNode(nvp.name(), YAML::NodeType::Map);
    if (!sub_node) { return; }
    YamlReadArchive sub_archive(sub_node, this);
    auto&& value = *nvp.value();
    sub_archive.Accept(&value);
  }

  template <typename NVP>
  void VisitScalar(const NVP& nvp) {
    const auto& sub_node = GetSubNode(nvp.name(), YAML::NodeType::Scalar);
    if (!sub_node) { return; }
    // TODO(jeremy.nimmer) Add better reporting of type errors here.
    using T = typename NVP::value_type;
    *nvp.value() = sub_node.template as<T>();
  }

  template <typename NVP>
  void VisitOptional(const NVP& nvp) {
    // When visiting an optional, it's fine if the YAML node is either absent
    // or has an empty value.  In yaml-cpp, presence is denoted by IsDefined(),
    // and empty is denoted by IsNull().
    const auto& sub_node = root_[nvp.name()];
    if (!sub_node.IsDefined() || sub_node.IsNull()) {
      *nvp.value() = drake::nullopt;
      return;
    }

    // Visit the unpacked optional as if it weren't wrapped in optional<>.
    using T = typename NVP::value_type::value_type;
    drake::optional<T>& storage = *nvp.value();
    if (!storage) { storage = T{}; }
    this->Visit(drake::MakeNameValue(nvp.name(), &storage.value()),
                VisitShouldMemorizeType::kNo);
  }

  template <typename NVP>
  void VisitVariant(const NVP& nvp) {
    const YAML::Node sub_node = root_[nvp.name()];
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
      const std::string item_name = fmt::format("{}[{}]", name, i);
      YAML::Node item_node(YAML::NodeType::Map);
      item_node[item_name] = sub_node[i];
      YamlReadArchive item_archive(item_node, this);
      item_archive.Visit(drake::MakeNameValue(item_name.c_str(), &data[i]));
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
        const std::string item_name =
            fmt::format("{}[{}][{}]", name, i, j);
        YAML::Node item_node(YAML::NodeType::Map);
        item_node[item_name] = sub_node[i][j];
        YamlReadArchive item_archive(item_node, this);
        item_archive.Visit(drake::MakeNameValue(
            item_name.c_str(), &storage(i, j)));
      }
    }
  }

  //@}

  // If root_ is a Map and has child with the given name and type, return the
  // child.  Otherwise, report an error and return an undefined node.
  YAML::Node GetSubNode(const char*, YAML::NodeType::value) const;

  void ReportMissingYaml(const std::string&) const;
  void PrintNodeSummary(std::ostream& s) const;
  void PrintVisitNameType(std::ostream& s) const;
  static const char* to_string(YAML::NodeType::value);

  const YAML::Node root_;
  const YamlReadArchive* const parent_;

  // These are non-nullptr only during Visit()'s lifetime.
  const char* debug_visit_name_{};
  const std::type_info* debug_visit_type_{};
};

}  // namespace common
}  // namespace anzu
