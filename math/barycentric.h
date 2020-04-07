#pragma once

#include <iterator>
#include <memory>
#include <set>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/// Represents a multi-linear function (from vector inputs to vector outputs) by
/// interpolating between points on a mesh using (triangular) barycentric
/// interpolation.
///
/// For a technical description of barycentric interpolation, see e.g.
///    Remi Munos and Andrew Moore, "Barycentric Interpolators for Continuous
///    Space and Time Reinforcement Learning", NIPS 1998
///
/// @tparam_double_only
template <typename T>
class BarycentricMesh {
  // TODO(russt): This is also an instance of a "linear function approximator"
  // -- a class of parameterized functions that take the form
  //    output = parameters*φ(input),
  // where
  //    parameters is a matrix of size num_outputs x num_features, with column i
  //               the vector value of ith mesh point.
  //    φ(input) is a vector of length num_features, where element i is the
  //             interpolation coefficient of the ith mesh point.
  // Only num_interpolants of these features are non-zero in any query.
  //
  // If we implement more function approximators, then I'm tempted to
  // call the base classes e.g. ParameterizedFunction and
  // LinearlyParameterizedFunction.
  //
  // Note: here we have a matrix of parameters and a feature vector, when the
  // more typical case of linear function approximators would use a vector of
  // parameters and a feature matrix.

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BarycentricMesh);

  /// The mesh is represented by a std::set (to ensure uniqueness and provide
  /// logarithmic lookups) of coordinates in each input dimension. Note: The
  /// values are type double, not T (We do not plan to take gradients, etc w/
  /// respect to them).
  typedef std::set<double> Coordinates;
  typedef std::vector<Coordinates> MeshGrid;

  /// Constructs the mesh.
  explicit BarycentricMesh(MeshGrid input_grid);

  // Accessor methods.
  const MeshGrid& get_input_grid() const { return input_grid_; }
  int get_input_size() const { return input_grid_.size(); }
  int get_num_mesh_points() const {
    int num_mesh_points = 1;
    for (const auto& coords : input_grid_) {
      num_mesh_points *= coords.size();
    }
    return num_mesh_points;
  }
  int get_num_interpolants() const { return num_interpolants_; }

  /// Writes the position of a mesh point in the input space referenced by its
  /// scalar index to @p point.
  /// @param index must be in [0, get_num_mesh_points).
  /// @param point is set to the num_inputs-by-1 location of the mesh point.
  void get_mesh_point(int index, EigenPtr<Eigen::VectorXd> point) const;

  /// Returns the position of a mesh point in the input space referenced by its
  /// scalar index to @p point.
  /// @param index must be in [0, get_num_mesh_points).
  VectorX<T> get_mesh_point(int index) const;

  /// Returns a matrix with all of the mesh points, one per column.
  MatrixX<T> get_all_mesh_points() const;

  /// Writes the mesh indices used for interpolation to @p mesh_indices, and the
  /// interpolating coefficients to @p weights.  Inputs that are outside the
  /// bounding box of the input_grid are interpolated as though they were
  /// projected (elementwise) to the closest face of the defined mesh.
  ///
  /// @param input must be a vector of length get_num_inputs().
  /// @param mesh_indices is a pointer to a vector of length
  /// get_num_interpolants().
  /// @param weights is a vector of coefficients (which sum to 1) of length
  /// get_num_interpolants().
  void EvalBarycentricWeights(const Eigen::Ref<const VectorX<T>>& input,
                              EigenPtr<Eigen::VectorXi> mesh_indices,
                              EigenPtr<VectorX<T>> weights) const;

  /// Evaluates the function at the @p input values, by interpolating between
  /// the values at @p mesh_values.  Inputs that are outside the
  /// bounding box of the input_grid are interpolated as though they were
  /// projected (elementwise) to the closest face of the defined mesh.
  ///
  /// Note that the dimension of the output vector is completely defined by the
  /// mesh_values argument.  This class does not maintain any information
  /// related to the size of the output.
  ///
  /// @param mesh_values is a num_outputs by get_num_mesh_points() matrix
  /// containing the points to interpolate between.  The order of the columns
  /// must be consistent with the mesh indices curated by this class, as exposed
  /// by get_mesh_point().
  /// @param input must be a vector of length get_num_inputs().
  /// @param output is the interpolated vector of length num_outputs
  void Eval(const Eigen::Ref<const MatrixX<T>>& mesh_values,
            const Eigen::Ref<const VectorX<T>>& input,
            EigenPtr<VectorX<T>> output) const;

  /// Returns the function evaluated at @p input.
  VectorX<T> Eval(const Eigen::Ref<const MatrixX<T>>& mesh_values,
                  const Eigen::Ref<const VectorX<T>>& input) const;

  /// Performs Eval, but with the possibility of the values on the mesh
  /// having a different scalar type than the values defining the mesh
  /// (symbolic::Expression containing decision variables for an optimization
  /// problem is an important example)
  /// @tparam ValueT defines the scalar type of the mesh_values and the output.
  /// @see Eval
  template <typename ValueT = T>
  void EvalWithMixedScalars(
      const Eigen::Ref<const MatrixX<ValueT>>& mesh_values,
      const Eigen::Ref<const VectorX<T>>& input,
      EigenPtr<VectorX<ValueT>> output) const {
    DRAKE_DEMAND(input.size() == get_input_size());
    DRAKE_DEMAND(mesh_values.cols() == get_num_mesh_points());

    Eigen::VectorXi mesh_indices(num_interpolants_);
    VectorX<T> weights(num_interpolants_);

    EvalBarycentricWeights(input, &mesh_indices, &weights);

    *output = weights[0] * mesh_values.col(mesh_indices[0]);
    for (int i = 1; i < num_interpolants_; i++) {
      *output += weights[i] * mesh_values.col(mesh_indices[i]);
    }
  }

  /// Returns the function evaluated at @p input.
  template <typename ValueT = T>
  VectorX<ValueT> EvalWithMixedScalars(
      const Eigen::Ref<const MatrixX<ValueT>>& mesh_values,
      const Eigen::Ref<const VectorX<T>>& input) const {
    VectorX<ValueT> output(mesh_values.rows());
    EvalWithMixedScalars<ValueT>(mesh_values, input, &output);
    return output;
  }

  /// Evaluates @p vector_func at all input mesh points and extracts the mesh
  /// value matrix that should be used to approximate the function with this
  /// barycentric interpolation.
  ///
  /// @code
  ///   MatrixXd mesh_values = bary.MeshValuesFrom(
  ///     [](const auto& x) { return Vector1d(std::sin(x[0])); });
  /// @endcode
  MatrixX<T> MeshValuesFrom(
      const std::function<VectorX<T>(const Eigen::Ref<const VectorX<T>>&)>&
          vector_func) const;

 private:
  MeshGrid input_grid_;      // Specifies the location of the mesh points in
                             // the input space.
  std::vector<int> stride_;  // The number of elements to skip to arrive at the
                             // next value (per input dimension)
  int num_interpolants_{1};  // The number of points used in any interpolation.
};

}  // namespace math
}  // namespace drake

extern template class ::drake::math::BarycentricMesh<double>;
