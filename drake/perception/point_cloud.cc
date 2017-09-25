#include "drake/perception/point_cloud.h"

#include <sstream>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

using Eigen::Map;
using Eigen::NoChange;

namespace drake {
namespace perception {

namespace {

// Convenience aliases.
typedef PointCloud::T T;
typedef PointCloud::E E;

// Utility for `ToString`.
std::string join(const std::vector<std::string>& elements,
                 const std::string &delim) {
  std::ostringstream os;
  for (size_t i = 0; i < elements.size(); ++i) {
    os << elements[i];
    if (i + 1 < elements.size())
      os << delim;
  }
  return os.str();
}

// Convert a PointCloud's set of fields to a string vector (for use
// with `ToString`).
std::vector<std::string> ToStringVector(
    pc_flags::Fields fields, const pc_flags::ExtraType& extra_type) {
  std::vector<std::string> out;
  if (fields & pc_flags::kXYZs)
    out.push_back("kXYZs");
  if (fields & pc_flags::kExtras)
      out.push_back("kExtras::" + extra_type.name());
  return out;
}

}  // namespace

namespace pc_flags {

std::string ToString(Fields fields, const ExtraType& extra_type) {
  return "(" + join(ToStringVector(fields, extra_type), " | ") + ")";
}

}  // namespace pc_flags


/*
 * Provides encapsulated storage for a `PointCloud`.
 *
 * This storage is not responsible for initializing default values.
 */
class PointCloud::Storage {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Storage)

  Storage(int new_size, pc_flags::Fields fields,
          const pc_flags::ExtraType& extra_type)
      : fields_(fields) {
    // Ensure that we incorporate the size of the extras.
    extras_.resize(extra_type.size(), 0);
    // Resize as normal.
    resize(new_size);
  }

  // Returns size of the storage.
  int size() const { return size_; }

  // Resize to parent cloud's size.
  void resize(int new_size) {
    size_ = new_size;
    if (fields_ & pc_flags::kXYZs)
      xyzs_.conservativeResize(NoChange, new_size);
    if (fields_ & pc_flags::kExtras)
      extras_.conservativeResize(NoChange, new_size);
    CheckInvariants();
  }

  Eigen::Ref<Matrix3X<T>> xyzs() { return xyzs_; }
  Eigen::Ref<MatrixX<T>> extras() { return extras_; }

 private:
  void CheckInvariants() const {
    if (fields_ & pc_flags::kXYZs) {
      const int xyz_size = xyzs_.cols();
      DRAKE_DEMAND(xyz_size == size());
    }
    if (fields_ & pc_flags::kExtras) {
      const int extra_size = extras_.cols();
      DRAKE_DEMAND(extra_size == size());
    }
  }

  const pc_flags::Fields fields_;
  int size_{};
  Matrix3X<T> xyzs_;
  MatrixX<T> extras_;
};

namespace {

// Ensure that a field set is complete valid (does not have extra bits).
void ValidateFields(pc_flags::Fields c) {
  pc_flags::Fields full_mask = pc_flags::kXYZs | pc_flags::kExtras;
  if (c <= 0 || c > full_mask) {
    throw std::runtime_error("Invalid Fields");
  }
}

pc_flags::Fields ResolveFields(
    const PointCloud& other, pc_flags::Fields fields) {
  ValidateFields(fields);
  if (fields == pc_flags::kInherit) {
    return other.fields();
  } else {
    return fields;
  }
}

// If kExtraInherit is used, then we take on the other's type is used.
// If another extra is used, and we wish to copy the other's extras,
// ensure they are compatible.
pc_flags::ExtraType ResolveExtraType(const PointCloud& other,
                               pc_flags::Fields fields,
                               const pc_flags::ExtraType& extra_type) {
  if (extra_type == pc_flags::kExtraInherit) {
    return other.extra_type();
  } else {
    if (fields & pc_flags::kExtras) {
      DRAKE_DEMAND(extra_type == other.extra_type());
    }
    return extra_type;
  }
}

// Implements the rules set forth in `SetFrom`.
// @pre Valid point clouds `a` and `b`.
// @post The returned fields will be valid for both point clouds. If
//   `c` enables extras, then the extra type will be shared by both point
//   clouds.
pc_flags::Fields ResolveFields(
    const PointCloud& a,
    const PointCloud& b,
    pc_flags::Fields fields) {
  ValidateFields(fields);
  if (fields == pc_flags::kInherit) {
    // If we do not permit a subset, expect the exact same fields.
    a.RequireExactFields(b.fields(), b.extra_type());
    return a.fields();
  } else {
    pc_flags::ExtraType extra_resolved =
        (fields & pc_flags::kExtras) ? a.extra_type() : pc_flags::kExtraNone;
    a.RequireFields(fields, extra_resolved);
    b.RequireFields(fields, extra_resolved);
    return fields;
  }
}

}  // namespace

PointCloud::PointCloud(
    int new_size,
    pc_flags::Fields fields,
    const pc_flags::ExtraType& extra_type)
    : size_(new_size),
      fields_(fields),
      extra_type_(extra_type) {
  ValidateFields(fields_);

  if (fields_ & pc_flags::kInherit)
    throw std::runtime_error("Cannot construct a PointCloud with kInherit");
  if (extra_type == pc_flags::kExtraInherit)
    throw std::runtime_error(
        "Cannot construct a PointCloud with kExtraInherit");
  if (has_extras()) {
    if (extra_type_ == pc_flags::kExtraNone)
      throw std::runtime_error("Cannot specify kExtraNone with kExtras");
  } else if (extra_type != pc_flags::kExtraNone) {
    throw std::runtime_error(
        "Must specify kExtraNone if kExtras is not present");
  }

  storage_.reset(new Storage(size_, fields_, extra_type_));
  SetDefault(0, size_);
}

PointCloud::PointCloud(const PointCloud& other,
                       pc_flags::Fields copy_fields,
                       const pc_flags::ExtraType& extra_type)
    : PointCloud(other.size(),
                 ResolveFields(other, copy_fields),
                 ResolveExtraType(other, copy_fields, extra_type)) {
  SetFrom(other);
}

PointCloud& PointCloud::operator=(const PointCloud& other) {
  SetFrom(other);
  return *this;
}

// Define destructor here to use complete definition of `Storage`.
PointCloud::~PointCloud() {}

void PointCloud::resize(int new_size, bool skip_initialization) {
  DRAKE_DEMAND(new_size >= 0);
  int old_size = size();
  size_ = new_size;
  storage_->resize(new_size);
  DRAKE_DEMAND(storage_->size() == new_size);
  if (new_size > old_size && !skip_initialization) {
    int size_diff = new_size - old_size;
    SetDefault(old_size, size_diff);
  }
}

void PointCloud::SetDefault(int start, int num) {
  auto set = [=](auto ref, auto value) {
    ref.middleCols(start, num).setConstant(value);
  };
  if (has_xyzs()) {
    set(mutable_xyzs(), kDefaultValue);
  }
  if (has_extras()) {
    set(mutable_extras(), kDefaultValue);
  }
}

void PointCloud::SetFrom(const PointCloud& other,
                         pc_flags::Fields fields_in,
                         bool allow_resize) {
  int old_size = size();
  int new_size = other.size();
  if (allow_resize) {
    resize(new_size);
  } else if (new_size != old_size) {
    throw std::runtime_error(
        fmt::format("CopyFrom: {} != {}", new_size, old_size));
  }
  pc_flags::Fields c_resolved = ResolveFields(*this, other, fields_in);
  if (c_resolved & pc_flags::kXYZs) {
    mutable_xyzs() = other.xyzs();
  }
  if (c_resolved & pc_flags::kExtras) {
    mutable_extras() = other.extras();
  }
}

void PointCloud::AddPoints(
    int add_size,
    bool skip_initialization) {
  DRAKE_DEMAND(add_size >= 0);
  const int new_size = size() + add_size;
  resize(new_size, skip_initialization);
}

bool PointCloud::has_xyzs() const {
  return fields_ & pc_flags::kXYZs;
}
Eigen::Ref<const Matrix3X<T>> PointCloud::xyzs() const {
  DRAKE_DEMAND(has_xyzs());
  return storage_->xyzs();
}
Eigen::Ref<Matrix3X<T>> PointCloud::mutable_xyzs() {
  DRAKE_DEMAND(has_xyzs());
  return storage_->xyzs();
}

bool PointCloud::has_extras() const {
  return fields_ & pc_flags::kExtras;
}
bool PointCloud::has_extras(const pc_flags::ExtraType& extra_type) const {
  return has_extras() && extra_type_ == extra_type;
}
Eigen::Ref<const MatrixX<E>> PointCloud::extras() const {
  DRAKE_DEMAND(has_extras());
  return storage_->extras();
}
Eigen::Ref<MatrixX<E>> PointCloud::mutable_extras() {
  DRAKE_DEMAND(has_extras());
  return storage_->extras();
}

bool PointCloud::HasFields(
    pc_flags::Fields fields_in,
    const pc_flags::ExtraType& extra_type_in) const {
  ValidateFields(fields_);
  bool good = true;
  if ((fields() & fields_in) != fields_in) {
    good = false;
  } else {
    if (fields_in & pc_flags::kExtras) {
      DRAKE_DEMAND(extra_type_in != pc_flags::kExtraNone);
      DRAKE_DEMAND(extra_type_in != pc_flags::kExtraInherit);
      if (extra_type() != extra_type_in) {
        good = false;
      }
    } else {
      DRAKE_DEMAND(extra_type_in == pc_flags::kExtraNone);
    }
  }
  return good;
}

void PointCloud::RequireFields(
    pc_flags::Fields fields_in,
    const pc_flags::ExtraType& extra_type_in) const {
  if (!HasFields(fields_in, extra_type_in)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have expected fields.\n"
                    "Expected {}, got {}",
                    pc_flags::ToString(fields_in, extra_type_in),
                    pc_flags::ToString(fields(), extra_type())));
  }
}

bool PointCloud::HasExactFields(
    pc_flags::Fields fields_in,
    const pc_flags::ExtraType& extra_type_in) const {
  return HasFields(fields_in, extra_type_in) && fields() == fields_in;
}

void PointCloud::RequireExactFields(
    pc_flags::Fields fields_in,
    const pc_flags::ExtraType& extra_type_in) const {
  if (!HasExactFields(fields_in, extra_type_in)) {
    throw std::runtime_error(
        fmt::format("PointCloud does not have the exact expected fields."
                    "\nExpected {}, got {}",
                    pc_flags::ToString(fields_in, extra_type_in),
                    pc_flags::ToString(fields(), extra_type())));
  }
}

}  // namespace perception
}  // namespace drake
