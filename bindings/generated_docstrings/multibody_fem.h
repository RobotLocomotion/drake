#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/multibody/fem/acceleration_newmark_scheme.h"
// #include "drake/multibody/fem/calc_lame_parameters.h"
// #include "drake/multibody/fem/constitutive_model.h"
// #include "drake/multibody/fem/corotated_model.h"
// #include "drake/multibody/fem/corotated_model_data.h"
// #include "drake/multibody/fem/damping_model.h"
// #include "drake/multibody/fem/deformable_body_config.h"
// #include "drake/multibody/fem/deformation_gradient_data.h"
// #include "drake/multibody/fem/dirichlet_boundary_condition.h"
// #include "drake/multibody/fem/discrete_time_integrator.h"
// #include "drake/multibody/fem/fem_element.h"
// #include "drake/multibody/fem/fem_indexes.h"
// #include "drake/multibody/fem/fem_model.h"
// #include "drake/multibody/fem/fem_model_impl.h"
// #include "drake/multibody/fem/fem_plant_data.h"
// #include "drake/multibody/fem/fem_solver.h"
// #include "drake/multibody/fem/fem_state.h"
// #include "drake/multibody/fem/fem_state_system.h"
// #include "drake/multibody/fem/force_density_field_base.h"
// #include "drake/multibody/fem/isoparametric_element.h"
// #include "drake/multibody/fem/linear_constitutive_model.h"
// #include "drake/multibody/fem/linear_constitutive_model_data.h"
// #include "drake/multibody/fem/linear_corotated_model.h"
// #include "drake/multibody/fem/linear_corotated_model_data.h"
// #include "drake/multibody/fem/linear_simplex_element.h"
// #include "drake/multibody/fem/matrix_utilities.h"
// #include "drake/multibody/fem/neohookean_model.h"
// #include "drake/multibody/fem/neohookean_model_data.h"
// #include "drake/multibody/fem/quadrature.h"
// #include "drake/multibody/fem/schur_complement.h"
// #include "drake/multibody/fem/simplex_gaussian_quadrature.h"
// #include "drake/multibody/fem/tet_subdivision_quadrature.h"
// #include "drake/multibody/fem/velocity_newmark_scheme.h"
// #include "drake/multibody/fem/volumetric_element.h"
// #include "drake/multibody/fem/volumetric_model.h"

// Symbol: pydrake_doc_multibody_fem
constexpr struct /* pydrake_doc_multibody_fem */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::ForceDensityFieldBase
      struct /* ForceDensityFieldBase */ {
        // Source: drake/multibody/fem/force_density_field_base.h
        const char* doc =
R"""(The ForceDensityFieldBase class is an abstract base class that
represents a force density field affecting deformable bodies in a
MultibodyPlant. The force field is described by the member function
EvaluateAt() which takes as input a position in the world frame and
returns the force density from the force density field at the given
location, with unit [N/m³]. To create a concrete ForceDensityFieldBase
class, inherit from ForceDensityField instead of directly inheriting
from ForceDensityFieldBase.)""";
        // Symbol: drake::multibody::ForceDensityFieldBase::Clone
        struct /* Clone */ {
          // Source: drake/multibody/fem/force_density_field_base.h
          const char* doc =
R"""(Returns an identical copy of ``this`` ForceDensityFieldBase.)""";
        } Clone;
        // Symbol: drake::multibody::ForceDensityFieldBase::DoClone
        struct /* DoClone */ {
          // Source: drake/multibody/fem/force_density_field_base.h
          const char* doc =
R"""(Derived classes must override this function to implement the NVI
Clone().)""";
        } DoClone;
        // Symbol: drake::multibody::ForceDensityFieldBase::DoEvaluateAt
        struct /* DoEvaluateAt */ {
          // Source: drake/multibody/fem/force_density_field_base.h
          const char* doc =
R"""(Derived classes must override this function to provide a threadsafe
implemention to the NVI EvaluateAt().)""";
        } DoEvaluateAt;
        // Symbol: drake::multibody::ForceDensityFieldBase::EvaluateAt
        struct /* EvaluateAt */ {
          // Source: drake/multibody/fem/force_density_field_base.h
          const char* doc =
R"""(Evaluates the force density [N/m³] with the given ``context`` of the
owning MultibodyPlant and a position in world, ``p_WQ``.)""";
        } EvaluateAt;
        // Symbol: drake::multibody::ForceDensityFieldBase::ForceDensityFieldBase<T>
        struct /* ctor */ {
          // Source: drake/multibody/fem/force_density_field_base.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::multibody::ForceDensityFieldBase::density_type
        struct /* density_type */ {
          // Source: drake/multibody/fem/force_density_field_base.h
          const char* doc = R"""()""";
        } density_type;
      } ForceDensityFieldBase;
      // Symbol: drake::multibody::ForceDensityType
      struct /* ForceDensityType */ {
        // Source: drake/multibody/fem/force_density_field_base.h
        const char* doc =
R"""((Advanced) Enum for the type of force density in
ForceDensityFieldBase.)""";
        // Symbol: drake::multibody::ForceDensityType::kPerCurrentVolume
        struct /* kPerCurrentVolume */ {
          // Source: drake/multibody/fem/force_density_field_base.h
          const char* doc =
R"""(ForceDensityFieldBase∷EvaluateAt() returns the force per unit of
*current* (deformed) configuration volume.)""";
        } kPerCurrentVolume;
        // Symbol: drake::multibody::ForceDensityType::kPerReferenceVolume
        struct /* kPerReferenceVolume */ {
          // Source: drake/multibody/fem/force_density_field_base.h
          const char* doc =
R"""(ForceDensityFieldBase∷EvaluateAt() returns the force per unit of
*reference* configuration volume where the reference undeformed
configuration is defined by the input mesh provided by the user.)""";
        } kPerReferenceVolume;
      } ForceDensityType;
      // Symbol: drake::multibody::fem
      struct /* fem */ {
        // Symbol: drake::multibody::fem::DampingModel
        struct /* DampingModel */ {
          // Source: drake/multibody/fem/damping_model.h
          const char* doc =
R"""(A viscous Rayleigh damping model in which the damping matrix D is a
linear combination of mass and stiffness matrices, as, D = αM + βK
where α and β are nonnegative. The damping ratio ζ for a given natural
frequency ωₙ of a mode of vibration can be calculated as ζ = (α/ωₙ +
βωₙ)/2. Notice the contribution of the stiffness term βK to the
damping matrix D causes ζ to be proportional to a mode's natural
frequency ωₙ whereas the contribution of the mass term αM to the
damping matrix D causes ζ to be inversely proportional to a mode's
natural frequency ωₙ. In the context of rigid body motion (ωₙ = 0),
only the αM term contributes to the damping matrix D, hence if rigid
body motion (or low value of ωₙ) is expected, α should be kept small.
One way to determine numerical values for α and β is to somehow obtain
reasonable estimates of the range of ωₙ, and then choose numerical
values for ζ (e.g 0 ≤ ζ < 0.05). Thereafter calculate the associated
numerical values of α and β.)""";
          // Symbol: drake::multibody::fem::DampingModel::DampingModel<T>
          struct /* ctor */ {
            // Source: drake/multibody/fem/damping_model.h
            const char* doc =
R"""(Constructs a Rayleigh damping model by storing the mass coefficient α
and the stiffness coefficient β that appears in the damping matrix D =
αM + βK.

Raises:
    RuntimeError if either ``mass_coeff_alpha`` or
    ``stiffness_coeff_beta`` is negative.)""";
          } ctor;
          // Symbol: drake::multibody::fem::DampingModel::mass_coeff_alpha
          struct /* mass_coeff_alpha */ {
            // Source: drake/multibody/fem/damping_model.h
            const char* doc = R"""()""";
          } mass_coeff_alpha;
          // Symbol: drake::multibody::fem::DampingModel::stiffness_coeff_beta
          struct /* stiffness_coeff_beta */ {
            // Source: drake/multibody/fem/damping_model.h
            const char* doc = R"""()""";
          } stiffness_coeff_beta;
        } DampingModel;
        // Symbol: drake::multibody::fem::DeformableBodyConfig
        struct /* DeformableBodyConfig */ {
          // Source: drake/multibody/fem/deformable_body_config.h
          const char* doc =
R"""(DeformableBodyConfig stores the physical parameters for a deformable
body. A default constructed configuration approximately represents a
hard rubber material (density, elasticity, and poisson's ratio)
without any damping. Damping coefficients are generally difficult to
measure and we expect users will typically start with zero damping and
tune the values to achieve reasonable dynamics. The config contains
the following fields with their corresponding valid ranges: - Young's
modulus: Measures the stiffness of the material, has unit N/m². Must
be positive. Default to 1e8. - Poisson's ratio: Measures the Poisson
effect (how much the material expands or contracts in directions
perpendicular to the direction of loading) of the material, unitless.
Must be greater than -1 and less than 0.5. Default to 0.49.

- Mass damping coefficient: Controls the strength of mass damping, has unit
1/s. Must be non-negative. Default to 0. See DampingModel.
- Stiffness damping coefficient: Controls the strength of stiffness damping,
has unit s. Must be non-negative. Default to 0. See DampingModel.
- Mass density: Has unit kg/m³. Must be positive. Default to 1.5e3.
- Material model: The constitutive model that describes the stress-strain
relationship of the body, see MaterialModel. Default to
MaterialModel∷kLinearCorotated.
- Element subdivision count: to integrate external volumetric forces, elements
can be subdivided to resolve large non-linearities within the domain of the
element. The number of resulting quadrature points is equal to 4^N where N is
the subdivision count. Zero means no subdivision. Default to 0.)""";
          // Symbol: drake::multibody::fem::DeformableBodyConfig::DeformableBodyConfig<T>
          struct /* ctor */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::element_subdivision_count
          struct /* element_subdivision_count */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Returns the number of times each element is subdivided when evaluating
the external forces. Useful when elements are too coarse to resolve
external force fields.)""";
          } element_subdivision_count;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::mass_damping_coefficient
          struct /* mass_damping_coefficient */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Returns the mass damping coefficient. See DampingModel.)""";
          } mass_damping_coefficient;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::mass_density
          struct /* mass_density */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Returns the mass density, with unit of kg/m³.)""";
          } mass_density;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::material_model
          struct /* material_model */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Returns the constitutive model of the material.)""";
          } material_model;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::poissons_ratio
          struct /* poissons_ratio */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc = R"""(Returns the Poisson's ratio, unitless.)""";
          } poissons_ratio;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::set_element_subdivision_count
          struct /* set_element_subdivision_count */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Precondition:
    0 <= element_subdivision_count <= 4.)""";
          } set_element_subdivision_count;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::set_mass_damping_coefficient
          struct /* set_mass_damping_coefficient */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Precondition:
    mass_damping_coefficient >= 0.)""";
          } set_mass_damping_coefficient;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::set_mass_density
          struct /* set_mass_density */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Precondition:
    mass_density > 0.)""";
          } set_mass_density;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::set_material_model
          struct /* set_material_model */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc = R"""()""";
          } set_material_model;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::set_poissons_ratio
          struct /* set_poissons_ratio */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Precondition:
    -1 < poissons_ratio < 0.5.)""";
          } set_poissons_ratio;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::set_stiffness_damping_coefficient
          struct /* set_stiffness_damping_coefficient */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Precondition:
    stiffness_damping_coefficient >= 0.)""";
          } set_stiffness_damping_coefficient;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::set_youngs_modulus
          struct /* set_youngs_modulus */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Precondition:
    youngs_modulus > 0.)""";
          } set_youngs_modulus;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::stiffness_damping_coefficient
          struct /* stiffness_damping_coefficient */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Returns the stiffness damping coefficient. See DampingModel.)""";
          } stiffness_damping_coefficient;
          // Symbol: drake::multibody::fem::DeformableBodyConfig::youngs_modulus
          struct /* youngs_modulus */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Returns the Young's modulus, with unit of N/m².)""";
          } youngs_modulus;
        } DeformableBodyConfig;
        // Symbol: drake::multibody::fem::FemModel
        struct /* FemModel */ {
          // Source: drake/multibody/fem/fem_model.h
          const char* doc =
R"""(FemModel calculates the components of the spatially discretized FEM
equations for dynamic elasticity problems. Typically, in dynamic
elasticity problems, we are interested in the mapping that describes
the motion of a material

ϕ(⋅,t) : Ω⁰ → Ωᵗ,

where Ω⁰ and Ωᵗ are subsets of R³, along with its first and second
derivatives (velocity and acceleration respectively):

V(⋅,t) = ∂ϕ(⋅,t)/∂t, A(⋅,t) = ∂²ϕ(⋅,t)/∂t².

We call Ω⁰ the reference domain and Ωᵗ the current domain. We use
upper case letters to denote states (positions, velocities, and
accelerations) in reference domain (X, V, A) and lower case letters to
denote their current domain counterparts (x, v, a). In particular,
x(X,t) = ϕ(X,t). The deformation gradient F(X,t) is given by
∂ϕ(X,t)/∂X.

The governing equations of interest are conservation of mass and
conservation of momentum:

R(X,t)J(X,t) = R(X,0), R(X,0)A(X,t) = fᵢₙₜ(X,t) + fₑₓₜ(X,t),

where R is mass density, fᵢₙₜ and fₑₓₜ are internal and external force
densities respectively, and J is the determinant of the deformation
gradient. Using finite element method to discretize in space, one gets

ϕ(X,t) = ∑ᵢ xᵢ(t)Nᵢ(X) V(X,t) = ∑ᵢ vᵢ(t)Nᵢ(X) A(X,t) = ∑ᵢ aᵢ(t)Nᵢ(X)

where xᵢ, vᵢ, aᵢ ∈ R³ are nodal values of the spatially discretized
position, velocity and acceleration, and Nᵢ(X):Ω⁰ → R are the the
basis functions. With this spatial discretization, the PDE is turned
into an ODE of the form

G(x, v, a) = 0, (1)

where x, v, a are the stacked xᵢ, vᵢ, aᵢ. FemModel provides methods to
query various information about equation (1) and its derivatives given
an FEM state (x, v, a).

We implement FemModel in FemModelImpl that templatizes on the type of
FemElement. Many functionalities provided by FemModel (e.g.
CalcTangentMatrix()) involve evaluating computationally intensive
loops over FEM elements, and the overhead caused by virtual methods
may be significant. We implement those functions in FemModelImpl
templated on the FemElement to avoid the overhead of virtual methods.
The type information at compile time also helps eliminate heap
allocations.

Sifakis, Eftychios, and Jernej Barbič. "Finite element method
simulation of 3d deformable solids." Synthesis Lectures on Visual
Computing: Computer Graphics, Animation, Computational Photography,
and Imaging 1.1 (2015): 1-69.)""";
          // Symbol: drake::multibody::fem::FemModel::ApplyBoundaryCondition
          struct /* ApplyBoundaryCondition */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Applies boundary condition set for this FemModel to the input
``state``. No-op if no boundary condition is set.

Precondition:
    fem_state != nullptr.

Raises:
    RuntimeError if the FEM state is incompatible with this model.)""";
          } ApplyBoundaryCondition;
          // Symbol: drake::multibody::fem::FemModel::Builder
          struct /* Builder */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Builder that builds the FemModel. Each concrete FemModel must define
its own builder, subclassing this class, to add new elements to the
model.)""";
            // Symbol: drake::multibody::fem::FemModel::Builder::Build
            struct /* Build */ {
              // Source: drake/multibody/fem/fem_model.h
              const char* doc =
R"""(Adds the FEM elements described by calls to this builder to this
associated FemModel. The builder is left in an invalid state after
Build() is invoked, and should thus be discarded and not reused again.)""";
            } Build;
            // Symbol: drake::multibody::fem::FemModel::Builder::Builder
            struct /* ctor */ {
              // Source: drake/multibody/fem/fem_model.h
              const char* doc =
R"""(Constructs a new builder that builds into the given ``model``.

Precondition:
    model != nullptr.

Note:
    The ``model`` pointer is persisted and the pointed to FemModel
    must outlive ``this`` *Builder.)""";
            } ctor;
            // Symbol: drake::multibody::fem::FemModel::Builder::DoBuild
            struct /* DoBuild */ {
              // Source: drake/multibody/fem/fem_model.h
              const char* doc =
R"""(Derived builders must provide implementations for this function to add
the FEM elements described by calls to the builder to the associated
FemModel.)""";
            } DoBuild;
            // Symbol: drake::multibody::fem::FemModel::Builder::ThrowIfBuilt
            struct /* ThrowIfBuilt */ {
              // Source: drake/multibody/fem/fem_model.h
              const char* doc =
R"""(Throws an exception if Build() has been called on this Builder.)""";
            } ThrowIfBuilt;
          } Builder;
          // Symbol: drake::multibody::fem::FemModel::CalcCenterOfMassPositionInWorld
          struct /* CalcCenterOfMassPositionInWorld */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Calculates the position vector from the world origin Wo to the center
of mass of all bodies in this FemModel S, expressed in the world frame
W.

Parameter ``fem_state``:
    The FemState used to evaluate the center of mass.

Returns ``p_WoScm_W``:
    position vector from Wo to Scm expressed in world frame W, where
    Scm is the center of mass of the system S stored by ``this``
    FemModel.

Raises:
    RuntimeError if the FEM state is incompatible with this model.)""";
          } CalcCenterOfMassPositionInWorld;
          // Symbol: drake::multibody::fem::FemModel::CalcCenterOfMassTranslationalVelocityInWorld
          struct /* CalcCenterOfMassTranslationalVelocityInWorld */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Calculates system center of mass translational velocity in world frame
W.

Parameter ``fem_state``:
    The FemState used for this calculation.

Returns ``v_WScm_W``:
    Scm's translational velocity in frame W, expressed in W, where Scm
    is the center of mass of the system S stored by this FemModel.

Raises:
    RuntimeError if the FEM state is incompatible with this model.)""";
          } CalcCenterOfMassTranslationalVelocityInWorld;
          // Symbol: drake::multibody::fem::FemModel::CalcEffectiveAngularVelocity
          struct /* CalcEffectiveAngularVelocity */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Using an angular momentum analogy, calculates an "effective" angular
velocity for this FemModel S, measured and expressed in the world
frame W. The effective angular velocity is computed using an angular
momentum equation that assumes S is a rigid body (albeit we know S is
deformable).

H_WSSm_W = I_SScm_W * w_WScm_W

for which when solved for w_WScm_W gives

w_WScm_W = inverse(I_SScm_W) * H_WSSm_W

where H_WSSm_W is the FemModel S's angular momentum about its center
of mass Scm measured and expressed in the world frame W.

Parameter ``fem_state``:
    The FemState used for this calculation.

Returns ``w_WScm_W``:
    the FemModel S's effective angular velocity for Scm, measured and
    expressed in the world frame W.

Raises:
    RuntimeError if the FEM state is incompatible with this model.)""";
          } CalcEffectiveAngularVelocity;
          // Symbol: drake::multibody::fem::FemModel::CalcResidual
          struct /* CalcResidual */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Calculates the residual G(x, v, a) (see class doc) evaluated at the
given FEM state using the given ``plant_data``. The residual for
degrees of freedom with Dirichlet boundary conditions is set to zero.
Therefore their residual should not be used as a metric for the error
on the boundary condition.

Precondition:
    residual != nullptr.

Raises:
    RuntimeError if the FEM state is incompatible with this model.)""";
          } CalcResidual;
          // Symbol: drake::multibody::fem::FemModel::CalcTangentMatrix
          struct /* CalcTangentMatrix */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Calculates an approximated tangent matrix evaluated at the given FEM
state. The tangent matrix is given by a weighted sum of stiffness
matrix (∂G/∂x), damping matrix (∂G/∂v), and mass matrix (∂G/∂a). The
corresponding row and column for a degree of freedom with Dirichlet
boundary condition in the tangent matrix is set to zero with the
exception of the diagonal entries which is set to a scalar multiple of
identity.

Parameter ``fem_state``:
    The FemState used to evaluate the tangent matrix.

Parameter ``tangent_matrix``:
    The output tangent_matrix.

Precondition:
    tangent_matrix != nullptr.

Precondition:
    The size of ``tangent_matrix`` is ``num_dofs()`` * ``num_dofs()``.

Precondition:
    All nonzero entries in the resulting tangent matrix have been
    allocated. See MakeTangentMatrix().

Warning:
    This function sometimes makes simplifying approximations to avoid
    taking overly complicated derivatives. As such, the resulting
    tangent matrix may be an approximation of the actual value
    depending on the constitutive model used.

Raises:
    RuntimeError if the FEM state is incompatible with this model.

Raises:
    RuntimeError if T is not double.)""";
          } CalcTangentMatrix;
          // Symbol: drake::multibody::fem::FemModel::Clone
          struct /* Clone */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc = R"""()""";
          } Clone;
          // Symbol: drake::multibody::fem::FemModel::DeclareCacheEntries
          struct /* DeclareCacheEntries */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Derived classes should override this method to declare cache entries
in the given ``fem_state_system``.)""";
          } DeclareCacheEntries;
          // Symbol: drake::multibody::fem::FemModel::DoCalcCenterOfMassPositionInWorld
          struct /* DoCalcCenterOfMassPositionInWorld */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(FemModelImpl must override this method to provide an implementation
for the NVI CalcCenterOfMassPositionInWorld(). The input ``fem_state``
is guaranteed to be compatible with ``this`` FEM model.)""";
          } DoCalcCenterOfMassPositionInWorld;
          // Symbol: drake::multibody::fem::FemModel::DoCalcCenterOfMassTranslationalVelocityInWorld
          struct /* DoCalcCenterOfMassTranslationalVelocityInWorld */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(FemModelImpl must override this method to provide an implementation
for the NVI CalcCenterOfMassTranslationalVelocityInWorld(). The input
``fem_state`` is guaranteed to be compatible with ``this`` FEM model.)""";
          } DoCalcCenterOfMassTranslationalVelocityInWorld;
          // Symbol: drake::multibody::fem::FemModel::DoCalcEffectiveAngularVelocity
          struct /* DoCalcEffectiveAngularVelocity */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(FemModelImpl must override this method to provide an implementation
for the NVI CalcEffectiveAngularVelocity(). The input ``fem_state`` is
guaranteed to be compatible with ``this`` FEM model.)""";
          } DoCalcEffectiveAngularVelocity;
          // Symbol: drake::multibody::fem::FemModel::DoCalcResidual
          struct /* DoCalcResidual */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(FemModelImpl must override this method to provide an implementation
for the NVI CalcResidual(). The input ``fem_state`` is guaranteed to
be compatible with ``this`` FEM model, and the input ``residual`` is
guaranteed to be non-null and properly sized.)""";
          } DoCalcResidual;
          // Symbol: drake::multibody::fem::FemModel::DoCalcTangentMatrix
          struct /* DoCalcTangentMatrix */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(FemModelImpl must override this method to provide an implementation
for the NVI CalcTangentMatrix(). The input ``fem_state`` is guaranteed
to be compatible with ``this`` FEM model, and the input
``tangent_matrix`` is guaranteed to be non-null and properly sized.)""";
          } DoCalcTangentMatrix;
          // Symbol: drake::multibody::fem::FemModel::DoClone
          struct /* DoClone */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(FemModelImpl must override this method to provide an implementation to
make a deep copy of the concrete FemModel.)""";
          } DoClone;
          // Symbol: drake::multibody::fem::FemModel::DoMakeTangentMatrix
          struct /* DoMakeTangentMatrix */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(FemModelImpl must override this method to provide an implementation
for the NVI MakeTangentMatrix().)""";
          } DoMakeTangentMatrix;
          // Symbol: drake::multibody::fem::FemModel::FemModel<T>
          struct /* ctor */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Constructs an empty FEM model.

Precondition:
    tangent_matrix_weights.minCoeff() >= 0.0.)""";
          } ctor;
          // Symbol: drake::multibody::fem::FemModel::MakeFemState
          struct /* MakeFemState */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Creates a default FemState compatible with this model.)""";
          } MakeFemState;
          // Symbol: drake::multibody::fem::FemModel::MakeReferencePositions
          struct /* MakeReferencePositions */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Returns the reference positions of this model.)""";
          } MakeReferencePositions;
          // Symbol: drake::multibody::fem::FemModel::MakeTangentMatrix
          struct /* MakeTangentMatrix */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Creates a symmetric block sparse matrix that has the sparsity pattern
of the tangent matrix of this FEM model. In particular, the size of
the tangent matrix is ``num_dofs()`` by ``num_dofs()``. All entries
are initialized to zero.

Raises:
    RuntimeError if T is not double.)""";
          } MakeTangentMatrix;
          // Symbol: drake::multibody::fem::FemModel::SetDirichletBoundaryCondition
          struct /* SetDirichletBoundaryCondition */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Sets the Dirichlet boundary condition that this model is subject to.
If dirichlet_bc specifies the boundary condition for a node for which
a boundary condition has already been specified, the lastest one will
be used.)""";
          } SetDirichletBoundaryCondition;
          // Symbol: drake::multibody::fem::FemModel::ThrowIfModelStateIncompatible
          struct /* ThrowIfModelStateIncompatible */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""((Internal use only) Throws RuntimeError to report a mismatch between
the FEM model and state that were passed to API method ``func``.)""";
          } ThrowIfModelStateIncompatible;
          // Symbol: drake::multibody::fem::FemModel::UpdateFemStateSystem
          struct /* UpdateFemStateSystem */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Updates the system that manages the states and the cache entries of
this FEM model. Must be called before calling MakeFemState() after the
FEM model changes (e.g. adding new elements).)""";
          } UpdateFemStateSystem;
          // Symbol: drake::multibody::fem::FemModel::dirichlet_boundary_condition
          struct /* dirichlet_boundary_condition */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Returns the Dirichlet boundary condition that this model is subject
to.)""";
          } dirichlet_boundary_condition;
          // Symbol: drake::multibody::fem::FemModel::do_is_linear
          struct /* do_is_linear */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Derived classes should override this method to indicate if the model
is linear.)""";
          } do_is_linear;
          // Symbol: drake::multibody::fem::FemModel::fem_state_system
          struct /* fem_state_system */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Returns the FemStateSystem that manages the states and cache entries
in this FemModel.)""";
          } fem_state_system;
          // Symbol: drake::multibody::fem::FemModel::get_total_mass
          struct /* get_total_mass */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc = R"""(Returns the total mass of the system.)""";
          } get_total_mass;
          // Symbol: drake::multibody::fem::FemModel::is_compatible_with
          struct /* is_compatible_with */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Returns true if the given FEM state is compatible with ``this`` FEM
model.)""";
          } is_compatible_with;
          // Symbol: drake::multibody::fem::FemModel::is_linear
          struct /* is_linear */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(Returns true the equation G(x, v, a) = 0 (see class documentation)
corresponding to this FemModel is linear.)""";
          } is_linear;
          // Symbol: drake::multibody::fem::FemModel::num_dofs
          struct /* num_dofs */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(The number of degrees of freedom in this model.)""";
          } num_dofs;
          // Symbol: drake::multibody::fem::FemModel::num_elements
          struct /* num_elements */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(The number of FEM elements in this model.)""";
          } num_elements;
          // Symbol: drake::multibody::fem::FemModel::num_nodes
          struct /* num_nodes */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""(The number of nodes that are associated with this model.)""";
          } num_nodes;
          // Symbol: drake::multibody::fem::FemModel::parallelism
          struct /* parallelism */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""((Internal use only) Returns the parallelism that ``this`` FemModel
uses when opportunities for parallel computation arises.)""";
          } parallelism;
          // Symbol: drake::multibody::fem::FemModel::set_parallelism
          struct /* set_parallelism */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc =
R"""((Internal use only) Configures the parallelism that ``this`` FemModel
uses when opportunities for parallel computation arises.)""";
          } set_parallelism;
          // Symbol: drake::multibody::fem::FemModel::tangent_matrix_weights
          struct /* tangent_matrix_weights */ {
            // Source: drake/multibody/fem/fem_model.h
            const char* doc = R"""()""";
          } tangent_matrix_weights;
        } FemModel;
        // Symbol: drake::multibody::fem::FemModelId
        struct /* FemModelId */ {
          // Source: drake/multibody/fem/fem_indexes.h
          const char* doc = R"""(Type used to identify FEM models.)""";
        } FemModelId;
        // Symbol: drake::multibody::fem::FemNodeIndex
        struct /* FemNodeIndex */ {
          // Source: drake/multibody/fem/fem_indexes.h
          const char* doc = R"""(Type used to index FEM nodes.)""";
        } FemNodeIndex;
        // Symbol: drake::multibody::fem::FemPlantData
        struct /* FemPlantData */ {
          // Source: drake/multibody/fem/fem_plant_data.h
          const char* doc = R"""()""";
          // Symbol: drake::multibody::fem::FemPlantData::FemPlantData<T>
          struct /* ctor */ {
            // Source: drake/multibody/fem/fem_plant_data.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::multibody::fem::FemPlantData::force_density_fields
          struct /* force_density_fields */ {
            // Source: drake/multibody/fem/fem_plant_data.h
            const char* doc = R"""()""";
          } force_density_fields;
          // Symbol: drake::multibody::fem::FemPlantData::plant_context
          struct /* plant_context */ {
            // Source: drake/multibody/fem/fem_plant_data.h
            const char* doc = R"""()""";
          } plant_context;
        } FemPlantData;
        // Symbol: drake::multibody::fem::FemState
        struct /* FemState */ {
          // Source: drake/multibody/fem/fem_state.h
          const char* doc =
R"""(FemState provides access to private workspace FEM state and
per-element state-dependent data. FemState comes in two flavors, an
"owned" version that owns the state and data stored in the private
workspace, and a "shared" version that doesn't own them. Because the
"owned" version owns the state, one can modify it (e.g. through
SetPositions). On the other hand, the "shared" version only provides a
window into the const state and data, so one cannot modify it. For
example, calling SetPositions on a "shared" FemState throws an
exception. A "shared" FemState is cheap to construct and should never
persist. It is advisable to acquire it for evaluation in a limited
scope (e.g., in a calc method of a cache entry) and then discard it.)""";
          // Symbol: drake::multibody::fem::FemState::Clone
          struct /* Clone */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc =
R"""(Returns an identical copy of ``this`` FemState.)""";
          } Clone;
          // Symbol: drake::multibody::fem::FemState::CopyFrom
          struct /* CopyFrom */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc =
R"""(Makes ``this`` FemState an exact copy of the given ``other`` FemState.

Raises:
    RuntimeError if num_dofs() of ``this`` FemState and ``other``
    FemState are not the same.

Raises:
    RuntimeError if ``this`` FemState is not owned (see class
    documentation).)""";
          } CopyFrom;
          // Symbol: drake::multibody::fem::FemState::EvalElementData
          struct /* EvalElementData */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc =
R"""(Returns an std∷vector of per-element data in ``this`` FemState.

Parameter ``cache_index``:
    The cache index of the per-element data.

Template parameter ``Data``:
    the per-element data type.

Raises:
    RuntimeError if the per-element data value doesn't actually have
    type ``Data``.)""";
          } EvalElementData;
          // Symbol: drake::multibody::fem::FemState::FemState<T>
          struct /* ctor */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc_1args =
R"""(Creates an "owned" version of FemState that allocates and accesses
states and cached data using the provided ``system``. The FemState
created with this constructor owns the states and data.

Precondition:
    system != nullptr.)""";
            // Source: drake/multibody/fem/fem_state.h
            const char* doc_2args =
R"""(Creates a "shared" version of FemState that accesses states and cached
data in the given ``context``. The FemState created with this
constructor doesn't own the states and data.

Precondition:
    system != nullptr.

Precondition:
    context != nullptr.

Precondition:
    system and context are compatible.)""";
          } ctor;
          // Symbol: drake::multibody::fem::FemState::GetAccelerations
          struct /* GetAccelerations */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc = R"""()""";
          } GetAccelerations;
          // Symbol: drake::multibody::fem::FemState::GetPositions
          struct /* GetPositions */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc =
R"""(@name Getters and setters for the FEM states The FEM states include
positions, time step positions (the positions at the previous time
step t₀), velocities, and accelerations. Positions and velocities are
actual state variables, while the accelerations are the saved result
of previous calculations required by certain integrators such as
NewmarkScheme.)""";
          } GetPositions;
          // Symbol: drake::multibody::fem::FemState::GetPreviousStepPositions
          struct /* GetPreviousStepPositions */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc = R"""()""";
          } GetPreviousStepPositions;
          // Symbol: drake::multibody::fem::FemState::GetVelocities
          struct /* GetVelocities */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc = R"""()""";
          } GetVelocities;
          // Symbol: drake::multibody::fem::FemState::SetAccelerations
          struct /* SetAccelerations */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc = R"""()""";
          } SetAccelerations;
          // Symbol: drake::multibody::fem::FemState::SetPositions
          struct /* SetPositions */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc = R"""()""";
          } SetPositions;
          // Symbol: drake::multibody::fem::FemState::SetTimeStepPositions
          struct /* SetTimeStepPositions */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc = R"""()""";
          } SetTimeStepPositions;
          // Symbol: drake::multibody::fem::FemState::SetVelocities
          struct /* SetVelocities */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc = R"""()""";
          } SetVelocities;
          // Symbol: drake::multibody::fem::FemState::is_created_from_system
          struct /* is_created_from_system */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc =
R"""(Returns true if this FemState is constructed from the given system.)""";
          } is_created_from_system;
          // Symbol: drake::multibody::fem::FemState::num_dofs
          struct /* num_dofs */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc =
R"""(Returns the number of degrees of freedom in the FEM model and state.)""";
          } num_dofs;
          // Symbol: drake::multibody::fem::FemState::num_nodes
          struct /* num_nodes */ {
            // Source: drake/multibody/fem/fem_state.h
            const char* doc =
R"""(Returns the number of nodes in the FEM model.)""";
          } num_nodes;
        } FemState;
        // Symbol: drake::multibody::fem::MaterialModel
        struct /* MaterialModel */ {
          // Source: drake/multibody/fem/deformable_body_config.h
          const char* doc =
R"""(Types of material models for the deformable body.)""";
          // Symbol: drake::multibody::fem::MaterialModel::kCorotated
          struct /* kCorotated */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Corotational model. More computationally expensive. Recommended when
capturing large rotation velocity is important.

[Stomakhin et al., 2012] Stomakhin, Alexey, et al. "Energetically
consistent invertible elasticity." Proceedings of the 11th ACM
SIGGRAPH/Eurographics conference on Computer Animation. 2012.)""";
          } kCorotated;
          // Symbol: drake::multibody::fem::MaterialModel::kLinear
          struct /* kLinear */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Linear elasticity model (rarely used). Less computationally expensive
than other models but leads to artifacts when large rotational
deformations occur.)""";
          } kLinear;
          // Symbol: drake::multibody::fem::MaterialModel::kLinearCorotated
          struct /* kLinearCorotated */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Linear corotational model as described in [Han et al., 2023]. It
provides a combination of accuracy, robustness, and speed. Recommended
in most scenarios. [Han et al., 2023] Han, Xuchen, Joseph Masterjohn,
and Alejandro Castro. "A Convex Formulation of Frictional Contact
between Rigid and Deformable Bodies." arXiv preprint arXiv:2303.08912
(2023).)""";
          } kLinearCorotated;
          // Symbol: drake::multibody::fem::MaterialModel::kNeoHookean
          struct /* kNeoHookean */ {
            // Source: drake/multibody/fem/deformable_body_config.h
            const char* doc =
R"""(Neohookean model. More computationally expensive. Recommended when
capturing large rotation velocity is important. There are subtle
differences in physical behavior between the Corotated model and the
Neo-Hookean model. See [Smith et al., 2019; Stomakhin et al., 2012]
for details. In practice, we often choose the Neo-Hookean model over
the Corotated model simply because it's computationally more
efficient.

[Smith et al., 2019] Smith, Breannan, Fernando De Goes, and Theodore
Kim. "Stable Neo-Hookean flesh simulation." ACM Transactions on
Graphics (TOG) 37.2 (2018): 1-15.)""";
          } kNeoHookean;
        } MaterialModel;
      } fem;
    } multibody;
  } drake;
} pydrake_doc_multibody_fem;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
