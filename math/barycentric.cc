#include "drake/math/barycentric.h"

#include <algorithm>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

namespace drake {
namespace math {

template <typename T>
BarycentricMesh<T>::BarycentricMesh(MeshGrid input_grid)
    : input_grid_(std::move(input_grid)),
      stride_(input_grid_.size()),
      num_interpolants_{1} {
  DRAKE_DEMAND(input_grid_.size() > 0);
  for (int i = 0; i < get_input_size(); i++) {
    // Must define at least one mesh point per dimension.
    DRAKE_DEMAND(!input_grid_[i].empty());

    // Gain one interpolant for every non-singleton dimension.
    if (input_grid_[i].size() > 1) num_interpolants_++;

    stride_[i] = (i == 0) ? 1 : input_grid_[i - 1].size() * stride_[i - 1];
  }
}

template <typename T>
void BarycentricMesh<T>::get_mesh_point(int index,
                                        EigenPtr<Eigen::VectorXd> point) const {
  DRAKE_DEMAND(index >= 0);
  DRAKE_DEMAND(point != nullptr);
  point->resize(get_input_size());
  // Iterate through the input dimensions, assigning the value and reducing
  // the index to be relevant only to the remaining dimensions.
  for (int i = 0; i < get_input_size(); i++) {
    const auto& coords = input_grid_[i];
    const int dim_index = index % coords.size();
    (*point)[i] = *(std::next(input_grid_[i].begin(), dim_index));
    index /= coords.size();  // intentionally truncate to int.
  }
  DRAKE_DEMAND(index == 0);  // otherwise the index was out of range.
}

template <typename T>
VectorX<T> BarycentricMesh<T>::get_mesh_point(int index) const {
  VectorX<T> point(get_input_size());
  get_mesh_point(index, &point);
  return point;
}

template <typename T>
MatrixX<T> BarycentricMesh<T>::get_all_mesh_points() const {
  const int M = get_input_size();
  const int N = get_num_mesh_points();
  VectorX<T> point(M);
  MatrixX<T> points(M, N);
  for (int i = 0; i < N; i++) {
    get_mesh_point(i, &point);
    points.col(i) = point;
  }
  return points;
}

template <typename T>
void BarycentricMesh<T>::EvalBarycentricWeights(
    const Eigen::Ref<const VectorX<T>>& input,
    EigenPtr<Eigen::VectorXi> mesh_indices,
    EigenPtr<VectorX<T>> weights) const {
  DRAKE_DEMAND(input.size() == static_cast<int>(input_grid_.size()));
  DRAKE_DEMAND(mesh_indices != nullptr && weights != nullptr);

  // std::pair of fractional position [0,1] and dimension index (position first,
  // so that std::pair's default operator< works for us).
  // There is one relative position for every non-singleton input dimension.  In
  // the case of triangular meshes, there is one interpolant for every
  // non-singular dimension + one additional, so num_interpolants-1 is the size
  // we need.
  std::vector<std::pair<T, int>> relative_position(num_interpolants_ - 1);
  // Bounding box on the input grid containing the sample input.
  std::vector<bool> has_volume(get_input_size());

  int current_index = 0;

  // Loop through input dimensions and compute the relative positions and
  // indices.  Set current_index to the "top right" corner index.
  int count = 0;
  for (int i = 0; i < get_input_size(); i++) {
    const auto& coords = input_grid_[i];

    // Skip over singleton dimensions.
    if (coords.size() == 1) continue;

    // Tag the positions with the dimension index.
    relative_position[count].second = i;

    // Find the right side of the bounding box.
    // Recall that lower_bound returns the first grid element that is NOT less
    // than the sample.
    auto right_iter = coords.lower_bound(input[i]);
    int right_index = 0;

    if (right_iter == coords.end()) {
      // ... then input is off the right end of the grid;
      // move it to the right boundary.
      has_volume[i] = false;
      right_index = coords.size() - 1;
      relative_position[count].first = T(1.);
    } else if (right_iter == coords.begin()) {
      // ... then input is at the first element or left of it;
      // move it to the left boundary.
      has_volume[i] = false;
      right_index = 0;
      relative_position[count].first = T(1.);
    } else {
      // ... then input is inside the grid.
      has_volume[i] = true;
      right_index = std::distance(coords.begin(), right_iter);
      const T& right_value = *(right_iter);
      const T& left_value = *(std::prev(right_iter));
      relative_position[count].first =
          (input[i] - left_value) / (right_value - left_value);
    }

    current_index += stride_[i] * right_index;
    count++;
  }
  DRAKE_ASSERT(count == (num_interpolants_ - 1));

  // Sort the dimensions by their relative position.  We identify which triangle
  // of the mesh we are in by moving along the faces in order of their relative
  // position.
  std::sort(relative_position.begin(), relative_position.end());

  mesh_indices->resize(num_interpolants_);
  weights->resize(num_interpolants_);
  (*mesh_indices)[0] = current_index;
  (*weights)[0] = relative_position[0].first;

  for (int i = 1; i < num_interpolants_; i++) {
    int dim = relative_position[i - 1].second;
    if (has_volume[dim]) {
      current_index -= stride_[dim];
    }
    (*mesh_indices)[i] = current_index;
    if (i == (num_interpolants_ - 1)) {
      (*weights)[i] = 1.0 - relative_position[i - 1].first;
    } else {
      (*weights)[i] =
          relative_position[i].first - relative_position[i - 1].first;
    }
  }
}

template <typename T>
void BarycentricMesh<T>::Eval(const Eigen::Ref<const MatrixX<T>>& mesh_values,
                              const Eigen::Ref<const VectorX<T>>& input,
                              EigenPtr<VectorX<T>> output) const {
  EvalWithMixedScalars<T>(mesh_values, input, output);
}

template <typename T>
VectorX<T> BarycentricMesh<T>::Eval(
    const Eigen::Ref<const MatrixX<T>>& mesh_values,
    const Eigen::Ref<const VectorX<T>>& input) const {
  return EvalWithMixedScalars<T>(mesh_values, input);
}

template <typename T>
MatrixX<T> BarycentricMesh<T>::MeshValuesFrom(
    const std::function<VectorX<T>(const Eigen::Ref<const VectorX<T>>&)>&
        vector_func) const {
  VectorX<T> sample(get_input_size());
  const int N = get_num_mesh_points();

  // Call it once to determine the size of the output and initialize memory.
  get_mesh_point(0, &sample);
  VectorX<T> value = vector_func(sample);
  MatrixX<T> mesh_values(value.rows(), N);
  mesh_values.col(0) = value;

  // Now loop through all of the other points.
  for (int i = 1; i < N; i++) {
    get_mesh_point(i, &sample);
    mesh_values.col(i) = vector_func(sample);
  }

  return mesh_values;
}

}  // namespace math
}  // namespace drake

template class ::drake::math::BarycentricMesh<double>;
