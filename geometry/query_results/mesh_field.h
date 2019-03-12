#pragma once

#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {

/**
  %SurfaceMeshField represents a field variable defined on a referenced
  SurfaceMesh.

  SurfaeMeshField can evaluate the field value at any position on any triangle
  of the SurfaceMesh.

  We store one field value per one vertex of the SurfaceMesh. Currently we
  support two kinds of values: scalar and 3-d vector.

  @tparam FieldValueType a valid Eigen scalar or vector for the field value.
  @tparam CoordType a valid Eigen scalar for coordinates.
*/
template <class FieldValueType, class CoordType>
class SurfaceMeshField {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceMeshField)

  /** Constructs a SurfaceMeshField that references a surface mesh.
    @param name    The name of the field variable.
    @param values  The field values that will be moved into this MeshField.
                   We require an rvalue reference.
    @param mesh    The surface mesh to which this MeshField refers.
                   We require an lvalue reference.
    @pre The number of entries in values is the same as the number of
         vertices of the mesh.
   */
  SurfaceMeshField(const std::string& name,
            std::vector<FieldValueType>&& values,
            const SurfaceMesh<CoordType>& mesh)
  : name_(name), values_(std::move(values)), mesh_(mesh)
  {}

  /** Evaluates the field value at a position on a triangular face.
    @param f The face index of the triangle.
    @param s The standard coordinates (s1, s2).
    @pre s1, s2 ∈ [0,1] and s1 + s2 ≤ 1.
   */
  FieldValueType Evaluate(SurfaceFaceIndex f, const Vector2<CoordType>& s) {
    DRAKE_DEMAND(CoordType(0.0) <= s(0) && s(1) <= CoordType(1.0));
    DRAKE_DEMAND(CoordType(0.0) <= s(1) && s(1) <= CoordType(1.0));
    DRAKE_DEMAND(s(0) + s(1) <= CoordType(1.0));
    const auto& face = mesh_.face(f);
    CoordType barycentric[3] = {CoordType(1.0) - s(0) - s(1), s(0), s(1)};
    FieldValueType value = barycentric[0] * values_[face.vertex(0)];
    for (int i = 1; i < 3; ++i)
      value += barycentric[i] * values_[face.vertex(i)];
    return value;
  }

 private:
  std::string name_;
  std::vector<FieldValueType> values_;
  const SurfaceMesh<CoordType>& mesh_;
};

// TODO(DamrongGuoy): Establish a system of classes for several
//  characteristics of mesh fields. I don't have a clear picture how to do
//  it yet.  It might be templated like this:
//    MeshField<TypeOfMesh, TypeOfEvaluation, FieldValueType, CoordType>
//    TypeOfMesh = VolumeMesh, SurfaceMesh
//    TypeOfEvaluation = LinearInterp, QuadraticInterp,
//                       PiecewiseConstant, AnalyticFunction
//    FieldValueType = Eigen scalar or Vector3
//    CoordType = Eigen scalar, AutoDiff?
//  It might use a class hierarchy like this:
//    virtual class MeshField
//    class PiecewiseConstantFiled : public MeshField;
//    class LinearInterpField : public MeshField;
//    class QuadraticInterpField : public MeshField;
//    class AnalyticFuncField : public MeshField;
//  It could be combinations of both ideas.
//  We have immediate need of mesh fields for:
//    1. SurfaceMesh, LinearInterp, scalar + vector
//    2. VolumeMesh, LinearInterp, scalar + vector
//  We might also need:
//    3. VolumeMesh, QuadraticInterp, scalar + vector
//  to generate "smoother" fields.
}  // namespace geometry
}  // namespace drake

