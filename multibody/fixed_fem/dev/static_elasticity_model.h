#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** The FEM model for static 3D elasticity problems. Implements the interface in
 FemModel.
 @tparam Element    The type of StaticElasticityElement used in this
 %StaticElasticityModel, must be an instantiation of StaticElasticityElement. */
template <class Element>
class StaticElasticityModel : public ElasticityModel<Element> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticElasticityModel);

  using T = typename Element::Traits::T;
  using ConstitutiveModelType = typename Element::ConstitutiveModelType;

  StaticElasticityModel() = default;
  ~StaticElasticityModel() = default;

  /** Add StaticElasticityElements to the %StaticElasticityModel from a mesh.
   @param mesh    The input tetrahedral mesh that describes the connectivity and
   the positions of the vertices. Each geometry::VolumeElement in the input
   `mesh` will generate an StaticElasticityElement in this
   %StaticElasticityModel.
   @param constitutive_model    The ConstitutiveModel to be used for all the
   StaticElasticityElements created.
   @pre The vertices in the `mesh` must be indexed consecutively from 0 to
   mesh.num_vertices()-1. */
  void AddStaticElasticityElementsFromTetMesh(
      const geometry::VolumeMesh<T>& mesh,
      const ConstitutiveModelType& constitutive_model);

 private:
  /* Implements FemModel::DoMakeFemState(). */
  FemState<Element> DoMakeFemState() const final;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
