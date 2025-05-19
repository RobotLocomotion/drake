#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
class PooledSapBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PooledSapBuilder);

  explicit PooledSapBuilder(const MultibodyPlant<T>& plant);

  void UpdateModel(const systems::Context<T>& context, double time_step,
                   PooledSapModel<T>* model) const;

  const MultibodyPlant<T>& plant() const { return *plant_; }

 private:
  void AccumulateActuationInput(const systems::Context<T>& context,
                                VectorX<T>* actuation_w_pd,
                                VectorX<T>* actuation_wo_pd) const;
  void CalcGeometryContactData(const systems::Context<T>& context) const;
  void AddPatchConstraintsForPointContact(const systems::Context<T>& context,
                                          PooledSapModel<T>* model) const;
  void AddPatchConstraintsForHydroelasticContact(
      const systems::Context<T>& context, PooledSapModel<T>* model) const;

  const MultibodyPlant<T>* plant_{nullptr};

  /* Scratch workspace data to build the model.  */
  struct Scratch {
    VectorX<T> u_no_pd;
    VectorX<T> u_w_pd;
    std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
    std::vector<geometry::ContactSurface<T>> surfaces;
  };
  /* This space is not intended for long-term storage and is often cleared or
   overwritten as needed. */
  mutable Scratch scratch_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::pooled_sap::PooledSapBuilder);
