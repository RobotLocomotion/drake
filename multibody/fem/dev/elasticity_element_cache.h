#pragma once

#include <type_traits>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/constitutive_model_cache.h"
#include "drake/multibody/fem/dev/element_cache.h"

namespace drake {
namespace multibody {
namespace fem {
/** Cached quantities per element that are used in the element routine for
 elasticity.
 @tparam_nonsymbolic_scalar T.
 @tparam SpatialDim The spatial dimension of the domain.
 @tparam ModelCacheType The type for constitutive model cache, must be
 derived from ConstitutiveModelCache<T, SpatialDim>. */
// TODO(xuchenhan-tri) link the doc of this class to FemElasticity when
// FemElasticity is checked in.
template <typename T, int SpatialDim, class ModelCacheType>
class ElasticityElementCache : public ElementCache<T, SpatialDim> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ElasticityElementCache);

  // The ModelType must be derived from ConstitutiveModelCache<T, SpatialDim>.
  // This also asserts that the scalar type for the ModelType must agree with
  // the scalar type of this cache and that their spatial dimensions must also
  // agree.
  static_assert(
      std::is_convertible<ModelCacheType*,
                          ConstitutiveModelCache<T, SpatialDim>*>::value);
  ElasticityElementCache(int element_index, int num_quads)
      : ElementCache<T, SpatialDim>(element_index, num_quads),
        F_(num_quads),
        F0_(num_quads),
        constitutive_model_cache_(element_index, num_quads),
        Psi_(num_quads),
        P_(num_quads),
        dPdF_(num_quads) {}

  using MatrixD = Eigen::Matrix<T, SpatialDim, SpatialDim>;

  virtual ~ElasticityElementCache() = default;

  void mark_v0_cache_stale() final {}

  void mark_v_cache_stale() final {}

  void mark_x0_cache_stale() final {
    mark_F0_stale(true);
    mark_Psi_stale(true);
    mark_P_stale(true);
    mark_dPdF_stale(true);
    mark_constitutive_model_x0_cache_stale(true);
  }

  void mark_x_cache_stale() final {
    mark_F_stale(true);
    mark_Psi_stale(true);
    mark_P_stale(true);
    mark_dPdF_stale(true);
    mark_constitutive_model_x_cache_stale(true);
  };

  /// Getters for the const cache entries.
  /// @{
  const std::vector<MatrixD>& get_F() const { return F_; }

  const std::vector<MatrixD>& get_F0() const { return F0_; }

  const ModelCacheType& get_constitutive_model_cache() const {
    return constitutive_model_cache_;
  }

  const std::vector<T>& get_Psi() const { return Psi_; }

  const std::vector<MatrixD>& get_P() const { return P_; }

  const std::vector<
      Eigen::Matrix<T, SpatialDim * SpatialDim, SpatialDim * SpatialDim>>&
  get_dPdF() const {
    return dPdF_;
  }
  /// @}

  /// Getters for the mutable cache entries.
  /// @{
  std::vector<MatrixD>& get_mutable_F() {
    mark_F_stale(true);
    return F_;
  }

  std::vector<MatrixD>& get_mutable_F0() {
    mark_F0_stale(true);
    return F0_;
  }

  ModelCacheType& get_mutable_constitutive_model_cache() {
    mark_constitutive_model_cache_stale(true);
    return constitutive_model_cache_;
  }

  std::vector<T>& get_mutable_Psi() {
    mark_Psi_stale(true);
    return Psi_;
  }

  std::vector<MatrixD>& get_mutable_P() {
    mark_P_stale(true);
    return P_;
  }

  std::vector<
      Eigen::Matrix<T, SpatialDim * SpatialDim, SpatialDim * SpatialDim>>&
  get_mutable_dPdF() {
    mark_dPdF_stale(true);
    return dPdF_;
  }
  /// @}

  /// Getters for the status of the cache entries.
  /// @{
  bool F_stale() const { return F_stale_; }
  bool F0_stale() const { return F0_stale_; }
  bool constitutive_model_cache_stale() const {
    return constitutive_model_x_cache_stale_ ||
           constitutive_model_x0_cache_stale_;
  }
  bool constitutive_model_x_cache_stale() const {
    return constitutive_model_x_cache_stale_;
  }
  bool constitutive_model_x0_cache_stale() const {
    return constitutive_model_x0_cache_stale_;
  }
  bool Psi_stale() const { return Psi_stale_; }
  bool P_stale() const { return P_stale_; }
  bool dPdF_stale() const { return dPdF_stale_; }
  /// @}

 private:
  // Facilitates testing.
  friend class ElasticityElementCacheTest;
  // TODO(xuchenhan-tri): Add FemElasticity as a friend.

  // Setters for the status of the cache entries.
  // @{
  void mark_F_stale(bool flag) { F_stale_ = flag; }
  void mark_F0_stale(bool flag) { F0_stale_ = flag; }
  void mark_constitutive_model_cache_stale(bool flag) {
    mark_constitutive_model_x_cache_stale(flag);
    mark_constitutive_model_x0_cache_stale(flag);
  }
  void mark_constitutive_model_x_cache_stale(bool flag) {
    constitutive_model_x_cache_stale_ = flag;
  }
  void mark_constitutive_model_x0_cache_stale(bool flag) {
    constitutive_model_x0_cache_stale_ = flag;
  }
  void mark_Psi_stale(bool flag) { Psi_stale_ = flag; }
  void mark_P_stale(bool flag) { P_stale_ = flag; }
  void mark_dPdF_stale(bool flag) { dPdF_stale_ = flag; }
  // @}

  // Deformation gradient. Depends on x.
  std::vector<MatrixD> F_;
  bool F_stale_{true};
  // Deformation gradient of the previous time step. Depends on x0.
  std::vector<MatrixD> F0_;
  bool F0_stale_{true};
  // Scratch for computing energy/stress/stress-derivatives. Depends on x0 and
  // x. We separate the cache values depending on x and x0 respectively so that
  // a change in x does not thrash cache depending solely on x0 and vice versa.
  ModelCacheType constitutive_model_cache_;
  bool constitutive_model_x_cache_stale_{true};
  bool constitutive_model_x0_cache_stale_{true};
  // Elastic energy density. Depends on x0 and x.
  std::vector<T> Psi_;
  bool Psi_stale_{true};
  // First Piola stress. Depends on x0 and x.
  std::vector<MatrixD> P_;
  bool P_stale_{true};
  // First Piola stress derivative. Depends on x0 and x.
  std::vector<
      Eigen::Matrix<T, SpatialDim * SpatialDim, SpatialDim * SpatialDim>>
      dPdF_;
  bool dPdF_stale_{true};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
