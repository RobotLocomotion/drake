#include "drake/planning/dof_mask.h"

#include <fmt/format.h>
#include <fmt/ranges.h>

namespace drake {
namespace planning {

using multibody::Joint;
using multibody::JointIndex;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;

DofMask::DofMask() = default;

DofMask::DofMask(int size, bool value)
    : data_(size >= 0 ? size : 0, value), count_(value ? size : 0) {
  DRAKE_THROW_UNLESS(size >= 0);
}

DofMask::DofMask(std::initializer_list<bool> values)
    : DofMask(std::vector(std::move(values))) {}

DofMask::DofMask(std::vector<bool> values) : data_(std::move(values)) {
  for (int i = 0; i < ssize(data_); ++i) {
    if (data_[i]) {
      ++count_;
    }
  }
}

DofMask DofMask::MakeFromModel(const MultibodyPlant<double>& plant,
                               ModelInstanceIndex model_index) {
  ThrowIfNotCompatible(plant);
  std::vector<bool> bits(plant.num_positions(), false);
  for (const JointIndex& j : plant.GetJointIndices(model_index)) {
    const Joint<double>& joint = plant.get_joint(j);
    if (joint.num_positions() == 0) continue;
    for (int i = joint.position_start();
         i < joint.position_start() + joint.num_positions(); ++i) {
      bits[i] = true;
    }
  }
  return DofMask(std::move(bits));
}

DofMask DofMask::MakeFromModel(const MultibodyPlant<double>& plant,
                               const std::string& model_name) {
  ThrowIfNotCompatible(plant);
  const ModelInstanceIndex model_index =
      plant.GetModelInstanceByName(model_name);
  return DofMask::MakeFromModel(plant, model_index);
}

void DofMask::ThrowIfNotCompatible(const MultibodyPlant<double>& plant) {
  if (!plant.IsVelocityEqualToQDot()) {
    throw std::runtime_error(fmt::format(
        "To use a plant with DofMask, the plant's ith velocity must be the "
        "time derivative of the ith position for all i. This isn't true for "
        "the given plant: '{}'.",
        plant.get_name()));
  }
  // N.B. Since many of our things are purely kinematic, we do not check
  // plant.num_actuated_dofs().
}

std::string DofMask::to_string() const {
  // In future versions (>9), fmt::to_string(data_) will work, but will require
  // including fmt/std.h.
  return fmt::to_string(std::vector<int>(data_.begin(), data_.end()));
}

DofMask DofMask::Complement() const {
  DofMask result(*this);
  result.data_.flip();
  result.count_ = this->size() - this->count();
  return result;
}

DofMask DofMask::Union(const DofMask& other) const {
  const int mask_size = size();
  DRAKE_THROW_UNLESS(other.size() == mask_size);
  DofMask result(mask_size, false);
  for (int i = 0; i < mask_size; ++i) {
    const bool bit = data_[i] || other[i];
    result.data_[i] = bit;
    if (bit) {
      ++result.count_;
    }
  }
  return result;
}

DofMask DofMask::Intersect(const DofMask& other) const {
  const int mask_size = size();
  DRAKE_THROW_UNLESS(other.size() == mask_size);
  DofMask result(mask_size, false);
  for (int i = 0; i < mask_size; ++i) {
    const bool bit = data_[i] && other[i];
    result.data_[i] = bit;
    if (bit) {
      ++result.count_;
    }
  }
  return result;
}

DofMask DofMask::Subtract(const DofMask& other) const {
  const int mask_size = size();
  DRAKE_THROW_UNLESS(other.size() == mask_size);
  DofMask result(mask_size, false);
  for (int i = 0; i < mask_size; ++i) {
    const bool bit = data_[i] && !other[i];
    result.data_[i] = bit;
    if (bit) {
      ++result.count_;
    }
  }
  return result;
}

void DofMask::GetFromArray(const Eigen::Ref<const Eigen::VectorXd>& full_vec,
                           drake::EigenPtr<Eigen::VectorXd> output) const {
  DRAKE_THROW_UNLESS(output != nullptr);
  DRAKE_THROW_UNLESS(output->size() == count());
  DRAKE_THROW_UNLESS(full_vec.size() == size());
  int out_index = -1;
  for (int i = 0; i < size(); ++i) {
    if (data_[i]) {
      (*output)[++out_index] = full_vec[i];
      if (out_index >= output->size()) break;
    }
  }
  DRAKE_DEMAND(out_index + 1 == output->size());
}

Eigen::VectorXd DofMask::GetFromArray(
    const Eigen::Ref<const Eigen::VectorXd>& full_vec) const {
  Eigen::VectorXd subset_vec = Eigen::VectorXd::Zero(count());
  GetFromArray(full_vec, &subset_vec);
  return subset_vec;
}

void DofMask::GetColumnsFromMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& full_mat,
    drake::EigenPtr<Eigen::MatrixXd> output) const {
  DRAKE_THROW_UNLESS(output != nullptr);
  DRAKE_THROW_UNLESS(output->cols() == count());
  DRAKE_THROW_UNLESS(full_mat.rows() == output->rows());
  DRAKE_THROW_UNLESS(full_mat.cols() == size());
  int out_index = -1;
  for (int i = 0; i < size(); ++i) {
    if (data_[i]) {
      output->col(++out_index) = full_mat.col(i);
      if (out_index >= output->cols()) break;
    }
  }
  DRAKE_DEMAND(out_index + 1 == output->cols());
}

Eigen::MatrixXd DofMask::GetColumnsFromMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& full_mat) const {
  Eigen::MatrixXd columns(full_mat.rows(), count());
  GetColumnsFromMatrix(full_mat, &columns);
  return columns;
}

void DofMask::SetInArray(const Eigen::Ref<const Eigen::VectorXd>& vec,
                         drake::EigenPtr<Eigen::VectorXd> output) const {
  DRAKE_THROW_UNLESS(vec.size() == count());
  DRAKE_THROW_UNLESS(output != nullptr);
  DRAKE_THROW_UNLESS(output->size() == size());
  int input_index = -1;
  for (int i = 0; i < size(); ++i) {
    if (data_[i]) {
      (*output)[i] = vec[++input_index];
    }
  }
  DRAKE_DEMAND(input_index + 1 == vec.size());
}

std::vector<JointIndex> DofMask::GetJoints(
    const MultibodyPlant<double>& plant) const {
  DofMask::ThrowIfNotCompatible(plant);
  DRAKE_THROW_UNLESS(size() == plant.num_positions());
  std::vector<JointIndex> result;
  for (const JointIndex& j : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(j);
    for (int i = joint.position_start();
         i < joint.position_start() + joint.num_positions(); ++i) {
      if (data_[i]) {
        result.push_back(j);
        break;
      }
    }
  }
  return result;
}

bool DofMask::operator==(const DofMask& o) const {
  const bool result = data_ == o.data_;
  DRAKE_ASSERT((result == false) || (this->count() == o.count()));
  return result;
}

std::vector<int> DofMask::GetSelectedToFullIndex() const {
  std::vector<int> ret(count_);
  int count = 0;
  for (int i = 0; i < std::ssize(data_); ++i) {
    if (data_[i]) {
      ret[count++] = i;
      if (count >= count_) {
        break;
      }
    }
  }
  return ret;
}

std::vector<std::optional<int>> DofMask::GetFullToSelectedIndex() const {
  std::vector<std::optional<int>> ret(std::ssize(data_), std::nullopt);
  int count = 0;
  for (int i = 0; i < std::ssize(data_); ++i) {
    if (data_[i]) {
      ret[i] = count++;
      if (count >= count_) {
        break;
      }
    }
  }
  return ret;
}

}  // namespace planning
}  // namespace drake
