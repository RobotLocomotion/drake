#pragma once

#include <initializer_list>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/reset_after_move.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {

/** A mask on the degrees of freedom (dofs) of a MultibodyPlant instance,
partitioning the plant's dofs into "selected" and "unselected" dofs.

A %DofMask has two measures: its "size" and its "count". The size is the number
of dofs it can mask in total (the selected and unselected dofs together). It
must be equal to the number of dofs contained in a MultibodyPlant instance (its
`q`s). The interpretation of a %DofMask's indices come from the plant's ordering
of its own dofs and their indices. A %DofMask instance's valid index values lie
in the range `[0, size())`. The _count_ of the %DofMask is the number of
_selected_ dofs.

Currently, %DofMask is only compatible with MultibodyPlant instances where the
generalized velocities *are* the same as the time derivatives of the generalized
positions. Or, in other words, `vᵢ = q̇ᵢ`. This property holds for all
single-dof joints but is not generally true for all joints (e.g., floating
joints or roll-pitch-yaw ball joints). */
class DofMask {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DofMask);

  /** @name  Constructors

  There are a number of constructors available. Using these constructors puts
  the burden on the caller to know what the appropriate size is and what the
  interpretation of the indices is. If used in conjunction with a MultibodyPlant
  instance, the caller must guarantee that the %DofMask is constructed to a
  compatible _size_ with the plant. */
  //@{

  /** Default constructor; creates a mask with no dofs (size() = count() = 0 ).
  @pydrake_mkdoc_identifier{default}
  */
  DofMask();

  /** Full/empty constructor.

  This allows for construction of a mask with the given `size` where all
  dofs are either selected (`value = true`) or unselected.

  @pre size >= 0
  @pydrake_mkdoc_identifier{by_size} */
  DofMask(int size, bool value);

  // Note: The following constructors allow implicit conversion for general
  // compatibility with vector<bool>.

  /** Constructs a %DofMask from an initializer list of bool.
  @pydrake_mkdoc_identifier{init_list} */
  // NOLINTNEXTLINE(runtime/explicit)
  DofMask(std::initializer_list<bool> values);

  /** Constructs a %DofMask from a vector of bool.
  @pydrake_mkdoc_identifier{vector_bool} */
  // NOLINTNEXTLINE(runtime/explicit)
  DofMask(std::vector<bool> values);

  //@}

  /** @name Factory methods

  These factory methods construct %DofMask instances from a MultibodyPlant.
  Which dofs are marked as "selected" depends on the particular factory method.
  */
  //@{

  /** Creates the %DofMask associated with a model instance (indicated by
  `model_index`) in `plant`.

  @throws std::exception if `plant` doesn't satisfy the compatibility
  requirements for %DofMask. */
  static DofMask MakeFromModel(const multibody::MultibodyPlant<double>& plant,
                               multibody::ModelInstanceIndex model_index);

  /** Creates the %DofMask associated with the model instance (named by
  `model_name`) in `plant`.

  @throws std::exception if `plant` doesn't satisfy the compatibility
  requirements for %DofMask. */
  static DofMask MakeFromModel(const multibody::MultibodyPlant<double>& plant,
                               const std::string& model_name);
  //@}

  /** @name Introspection */
  //{

  /** Reports this %DofMask instance's total number of indexable dofs.  */
  int size() const { return ssize(data_); }

  /** Reports this %DofMask instance's number of _selected_ dofs. */
  int count() const { return count_; }

  /** Creates a collection of all of the joints implied by the selected dofs in
  `this`. The returned joint indices are reported in increasing order.
  @pre `plant` is compatible with %DofMask.
  @pre `plant.num_positions() == size()`. */
  std::vector<multibody::JointIndex> GetJoints(
      const multibody::MultibodyPlant<double>& plant) const;

  /** Note: `o.size()` may be different from `this->size()`. They will, by
  definition report as not equal. */
  bool operator==(const DofMask& o) const;

  /** @pre `index` is in the range [0, size()). */
  bool operator[](int index) const { return data_.at(index); }

  /** The string representation of the mask -- it encodes the full mask size
  clearly indicating which dofs are selected and which are unselected. The exact
  format is not guaranteed to remain fixed, but always clear. */
  std::string to_string() const;

  //}

  /** @name Combining masks

  We can think of the masks as sets -- the iᵗʰ dof is in the set if it's
  selected. Therefore, we can apply set operations to create new masks from
  existing masks. Each operation creates a new %DofMask.

  The parameter for these methods, when present, must have the same size() as
  `this->size()`. */
  //{

  /** Returns a new mask such that its selected dofs are `this` mask's
  unselected dofs and vice versa. */
  [[nodiscard]] DofMask Complement() const;

  /** Creates the union of `this` and `other`. The result includes all selected
  dofs in either `this` or `other`.
  @pre `size() == other.size()`. */
  [[nodiscard]] DofMask Union(const DofMask& other) const;

  /** Creates the intersection of `this` and `other`. The result includes all
  selected dofs that are in both `this` and `other`.
  @pre `size() == other.size()`. */
  [[nodiscard]] DofMask Intersect(const DofMask& other) const;

  /** Creates the set difference of `this` and `other`. The result includes
  only those selected dofs in `this` that are _not_ in `other`.
  @pre `size() == other.size()`. */
  [[nodiscard]] DofMask Subtract(const DofMask& other) const;

  //}

  /** @name Scattering and gathering

  These functions allow the mask to selectively write dof-specific values into
  a full state vector or read dof-specific values from a full state vector into
  a compact, selected-dofs-only vector. Only the _selected_ dofs in a %DofMask
  instance participate in the operation. */
  //@{

  /** Gets a subset of the values in `full_vec` and writes them to `output`.
  The values read from `full_vec` associated with the _selected_ dofs in `this`
  are written into `output` with the same relative ordering as they appear in
  `full_vec`.

  For example:

  @code{c++}
  // A mask into five total dofs, with indices 0, 1, & 4 selected.
  const DofMask mask({true, true, false, false, true});
  const auto full_vec = (VectorX(5) << 0, 10, 20, 30, 40).finished();
  VectorX selected(mask.count());
  mask.GetFromArray(full_vec, &selected);
  // Print out the vector: [0, 10, 40].
  std::cout << selected.transpose() << "\n";
  @endcode

  @pre `full_vec.size() == size()`.
  @pre `output` is not null.
  @pre `output.size() == count()`. */
  void GetFromArray(const Eigen::Ref<const Eigen::VectorXd>& full_vec,
                    drake::EigenPtr<Eigen::VectorXd> output) const;

  /** Overload for GetFromArray() which returns the read values in a new vector
  instead of writing to an output argument.
  @pre `full_vec.size() == size()`. */
  [[nodiscard]] Eigen::VectorXd GetFromArray(
      const Eigen::Ref<const Eigen::VectorXd>& full_vec) const;

  /** Gets a subset of the _columns_ of `full_mat` and writes them to `output`.
  Similar to GetFromArray(), but instead of single scalar values, whole columns
  are read from `full_mat` and written to `output`.

  @pre `output` is not null.
  @pre `output.cols() == count()`.
  @pre `full_mat.cols() == size()`.
  @pre `full_mat.rows() = output->rows()`. */
  void GetColumnsFromMatrix(const Eigen::Ref<const Eigen::MatrixXd>& full_mat,
                            drake::EigenPtr<Eigen::MatrixXd> output) const;

  /** Overload for GetColumnsFromMatrix() which returns the read values in a new
  matrix instead of writing to an output argument.
  @pre `full_mat.cols() == size()`. */
  [[nodiscard]] Eigen::MatrixXd GetColumnsFromMatrix(
      const Eigen::Ref<const Eigen::MatrixXd>& full_mat) const;

  /** Sets the values given in `vec` into `output`. This is the inverse of
  GetFromArray(). `vec` contains count() values and the iᵗʰ value in `vec` is
  written to `output` at the index corresponding to the iᵗʰ _selected_ dof in
  `this`. Entries in `output` which correspond to the _unselected_ dofs will
  remain unchanged.

  For example:

  @code{c++}
  // A mask into five total dofs, with indices 0, 1, & 4 selected.
  const DofMask mask({true, true, false, false, true});
  const auto dof_vec = (VectorX(3) << -10, -20, -30).finished();
  auto full_vec = (VectorX(5) << 0, 1, 2, 3, 4).finished();
  mask.SetInArray(dof_vec, &full_vec);
  // Print out the vector: [-10, -20, 2, 3, -30].
  std::cout << full_vec.transpose() << "\n";
  @endcode

  @pre `vec.size() = this->count()`.
  @pre `output` is not null.
  @pre `output.size() == size()`. */
  void SetInArray(const Eigen::Ref<const Eigen::VectorXd>& vec,
                  drake::EigenPtr<Eigen::VectorXd> output) const;

  /** If we have q_selected = dof_mask.GetFromArray(q_full), then this function
   returns a mapping from q_selected index to q_full index, such that
   q_selected[i] is the same as q_full[dof_mask.GetSelectedToFullIndex()[i]]. */
  [[nodiscard]] std::vector<int> GetSelectedToFullIndex() const;

  /** The inverse mapping of GetSelectedToFullIndex().
   If we have q_selected = dof_mask.GetFromArray(q_full), the this function
   returns the mapping from q_full index to q_selected index. Namely
   if dof_mask[i] is true, namely q_full[i] is selected, then
   q_selected[*dof_mask.GetFullToSelectedIndex()[i]] is the same as q_full[i];
   if dof_mask[i] is false, then dof_mask.GetFullToSelectedIndex()[i] is
   nullopt.
   */
  [[nodiscard]] std::vector<std::optional<int>> GetFullToSelectedIndex() const;
  //@}

 private:
  /* Throws if `plant` is *not* compatible with %DofMask's assumptions.
  Specifically, the iᵗʰ velocity corresponds to the iᵗʰ position for all
  `i`; `vᵢ = q̇ᵢ`. */
  static void ThrowIfNotCompatible(
      const multibody::MultibodyPlant<double>& plant);

  // These member fields are almost "const" -- we have no member functions that
  // mutate them, other than the two default assignment operators.
  std::vector<bool> data_;
  reset_after_move<int> count_{0};
};

}  // namespace planning
}  // namespace drake
