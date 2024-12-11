#include "drake/multibody/mpm/particle_data.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

template <typename T>
ConstitutiveModelVariant<T> MakeConstitutiveModel(
    const fem::DeformableBodyConfig<double>& config) {
  switch (config.material_model()) {
    case fem::MaterialModel::kCorotated:
      return fem::internal::CorotatedModel<T>(config.youngs_modulus(),
                                              config.poissons_ratio());
    case fem::MaterialModel::kLinearCorotated:
      return fem::internal::LinearCorotatedModel<T>(config.youngs_modulus(),
                                                    config.poissons_ratio());
    case fem::MaterialModel::kLinear:
      return fem::internal::LinearConstitutiveModel<T>(config.youngs_modulus(),
                                                       config.poissons_ratio());
  }
  DRAKE_UNREACHABLE();
}

template <typename T>
DeformationGradientDataVariant<T> MakeStrainData(
    const fem::DeformableBodyConfig<double>& config) {
  switch (config.material_model()) {
    case fem::MaterialModel::kCorotated:
      return fem::internal::CorotatedModelData<T>();
    case fem::MaterialModel::kLinearCorotated:
      return fem::internal::LinearCorotatedModelData<T>();
    case fem::MaterialModel::kLinear:
      return fem::internal::LinearConstitutiveModelData<T>();
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

template <typename T>
ParticleData<T>::ParticleData() = default;

template <typename T>
T ParticleData<T>::ComputeTotalEnergy(const std::vector<Matrix3<T>>& F,
                                      const std::vector<Matrix3<T>>& F0) const {
  T result = 0;
  for (int p = 0; p < num_particles(); ++p) {
    std::visit(
        [&, this](auto&& model) {
          const Matrix3<T>& F_p = F[p];
          const Matrix3<T>& F0_p = F0[p];
          using StrainDataType = typename std::decay_t<decltype(model)>::Data;
          StrainDataType& strain_data_p =
              std::get<StrainDataType>(strain_data_[p]);
          strain_data_p.UpdateData(F_p, F0_p);
          T Psi;
          model.CalcElasticEnergyDensity(strain_data_p, &Psi);
          result += Psi * volume_[p];
        },
        constitutive_models_[p]);
  }
  return result;
}

template <typename T>
void ParticleData<T>::ComputeKirchhoffStress(
    const std::vector<Matrix3<T>>& F, const std::vector<Matrix3<T>>& F0,
    std::vector<Matrix3<T>>* volume_scaled_stress,
    Parallelism parallelism) const {
  [[maybe_unused]] const int num_threads = parallelism.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int p = 0; p < num_particles(); ++p) {
    std::visit(
        [&, this](auto& model) {
          const Matrix3<T>& F_p = F[p];
          const Matrix3<T>& F0_p = F0[p];
          using StrainDataType = typename std::decay_t<decltype(model)>::Data;
          StrainDataType& strain_data_p =
              std::get<StrainDataType>(strain_data_[p]);
          strain_data_p.UpdateData(F_p, F0_p);
          auto& tau_volume_p = (*volume_scaled_stress)[p];
          const Matrix3<T>& particle_F = F_[p];
          model.CalcFirstPiolaStress(strain_data_p, &tau_volume_p);
          tau_volume_p *= volume_[p] * particle_F.transpose();
        },
        constitutive_models_[p]);
  }
}

template <typename T>
void ParticleData<T>::ComputePK1StressDerivatives(
    const std::vector<Matrix3<T>>& F, const std::vector<Matrix3<T>>& F0,
    std::vector<math::internal::FourthOrderTensor<T>>* dPdF_volume,
    Parallelism parallelism) const {
  [[maybe_unused]] const int num_threads = parallelism.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int p = 0; p < num_particles(); ++p) {
    std::visit(
        [&, this](auto& model) {
          const Matrix3<T>& F_p = F[p];
          const Matrix3<T>& F0_p = F0[p];
          using StrainDataType = typename std::decay_t<decltype(model)>::Data;
          StrainDataType& strain_data_p =
              std::get<StrainDataType>(strain_data_[p]);
          strain_data_p.UpdateData(F_p, F0_p);
          auto& dPdF_volume_p = (*dPdF_volume)[p];
          model.CalcFirstPiolaStressDerivative(strain_data_p, &dPdF_volume_p);
          dPdF_volume_p.mutable_data() *= volume_[p];
        },
        constitutive_models_[p]);
  }
}

template <typename T>
void ParticleData<T>::Sample(const std::vector<Vector3<double>>& positions,
                             double total_volume,
                             const fem::DeformableBodyConfig<double>& config) {
  DRAKE_THROW_UNLESS(total_volume > 0);
  DRAKE_THROW_UNLESS(!positions.empty());
  const int num_new_particles = ssize(positions);
  const double mass_density = config.mass_density();
  const double volume_per_particle = total_volume / num_new_particles;
  const ConstitutiveModelVariant<T> constitutive_model =
      MakeConstitutiveModel<T>(config);
  const DeformationGradientDataVariant<T> strain_data =
      MakeStrainData<T>(config);
  for (int i = 0; i < num_new_particles; ++i) {
    m_.push_back(mass_density * volume_per_particle);
    x_.emplace_back(positions[i].cast<T>());
    v_.emplace_back(Vector3<T>::Zero());
    F_.emplace_back(Matrix3<T>::Identity());
    F0_.emplace_back(Matrix3<T>::Identity());
    C_.emplace_back(Matrix3<T>::Zero());
    volume_.push_back(volume_per_particle);
    constitutive_models_.push_back(constitutive_model);
    tau_volume_.emplace_back(Matrix3<T>::Zero());
    in_constraint_.push_back(false);
    strain_data_.push_back(strain_data);
  }
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::ParticleData<float>;
template class drake::multibody::mpm::internal::ParticleData<double>;
template class drake::multibody::mpm::internal::ParticleData<drake::AutoDiffXd>;
