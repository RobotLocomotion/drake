#pragma once

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Sparse>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/parallelism.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/fem/dirichlet_boundary_condition.h"
#include "drake/multibody/fem/fem_plant_data.h"
#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {

// TODO(xuchenhan-tri): Provide some details on spatial integration orders.
/** %FemModel calculates the components of the spatially discretized FEM
 equations for dynamic elasticity problems. Typically, in dynamic elasticity
 problems, we are interested in the mapping that describes the motion of a
 material

     ϕ(⋅,t) : Ω⁰ → Ωᵗ,

 where Ω⁰ and Ωᵗ are subsets of R³, along with its first and second derivatives
 (velocity and acceleration respectively):

     V(⋅,t) = ∂ϕ(⋅,t)/∂t,
     A(⋅,t) = ∂²ϕ(⋅,t)/∂t².

 We call Ω⁰ the reference domain and Ωᵗ the current domain. We use upper case
 letters to denote states (positions, velocities, and accelerations) in
 reference domain (X, V, A) and lower case letters to denote their current
 domain counterparts (x, v, a). In particular, x(X,t) = ϕ(X,t). The deformation
 gradient F(X,t) is given by ∂ϕ(X,t)/∂X.

 The governing equations of interest are conservation of mass and conservation
 of momentum:

     R(X,t)J(X,t) = R(X,0),
     R(X,0)A(X,t) = fᵢₙₜ(X,t) + fₑₓₜ(X,t),

 where R is mass density, fᵢₙₜ and fₑₓₜ are internal and external force
 densities respectively, and J is the determinant of the deformation gradient.
 Using finite element method to discretize in space, one gets

     ϕ(X,t) = ∑ᵢ xᵢ(t)Nᵢ(X)
     V(X,t) = ∑ᵢ vᵢ(t)Nᵢ(X)
     A(X,t) = ∑ᵢ aᵢ(t)Nᵢ(X)

 where xᵢ, vᵢ, aᵢ ∈ R³ are nodal values of the spatially discretized position,
 velocity and acceleration, and Nᵢ(X):Ω⁰ → R are the the basis functions. With
 this spatial discretization, the PDE is turned into an ODE of the form

     G(x, v, a) = 0,            (1)

 where x, v, a are the stacked xᵢ, vᵢ, aᵢ. %FemModel provides methods to
 query various information about equation (1) and its derivatives given an FEM
 state (x, v, a).

 We implement %FemModel in FemModelImpl that templatizes on the type of
 FemElement. Many functionalities provided by %FemModel (e.g.
 CalcTangentMatrix()) involve evaluating computationally intensive loops
 over FEM elements, and the overhead caused by virtual methods may be
 significant. We implement those functions in FemModelImpl templated on the
 FemElement to avoid the overhead of virtual methods. The type information at
 compile time also helps eliminate heap allocations.

 Sifakis, Eftychios, and Jernej Barbič. "Finite element method simulation of 3d
 deformable solids." Synthesis Lectures on Visual Computing: Computer Graphics,
 Animation, Computational Photography, and Imaging 1.1 (2015): 1-69.

 @tparam_default_scalar */
template <typename T>
class FemModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemModel);

  /** %Builder that builds the FemModel. Each concrete %FemModel must define its
   own builder, subclassing this class, to add new elements to the model. */
  class Builder {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Builder);

    virtual ~Builder();

    /** Adds the FEM elements described by calls to this builder to this
     associated FemModel. The builder is left in an invalid state after Build()
     is invoked, and should thus be discarded and not reused again. */
    void Build();

   protected:
    /** Throws an exception if Build() has been called on this %Builder. */
    void ThrowIfBuilt() const;

    /** Constructs a new builder that builds into the given `model`.
     @pre model != nullptr.
     @note The `model` pointer is persisted and the pointed to FemModel must
     outlive `this` *Builder. */
    explicit Builder(FemModel<T>* model) : model_{model} {
      DRAKE_DEMAND(model_ != nullptr);
    }

    /** Derived builders must provide implementations for this function to add
     the FEM elements described by calls to the builder to the associated
     FemModel. */
    virtual void DoBuild() = 0;

   private:
    /* The model that `this` builder builds into. */
    FemModel<T>* model_{nullptr};
    /* Flag to keep track of whether Build() has been called on this builder. */
    bool built_{false};
  };

  virtual ~FemModel();

  std::unique_ptr<FemModel<T>> Clone() const;

  /* The `num_dofs()` is always a multiple of 3. It is enforced by
   FemStateSystem. */
  /** The number of nodes that are associated with this model. */
  int num_nodes() const { return num_dofs() / 3; }

  /** The number of degrees of freedom in this model. */
  int num_dofs() const { return fem_state_system_->num_dofs(); }

  /** The number of FEM elements in this model. */
  virtual int num_elements() const = 0;

  /** Creates a default FemState compatible with this model. */
  std::unique_ptr<FemState<T>> MakeFemState() const;

  /** Calculates the residual G(x, v, a) (see class doc) evaluated at the
   given FEM state using the given `plant_data`. The residual for degrees of
   freedom with Dirichlet boundary conditions is set to zero. Therefore their
   residual should not be used as a metric for the error on the boundary
   condition.
   @pre residual != nullptr.
   @throws std::exception if the FEM state is incompatible with this model. */
  void CalcResidual(const FemState<T>& fem_state,
                    const FemPlantData<T>& plant_data,
                    EigenPtr<VectorX<T>> residual) const;

  /** Calculates an approximated tangent matrix evaluated at the given FEM
   state. The tangent matrix is given by a weighted sum of stiffness matrix
   (∂G/∂x), damping matrix (∂G/∂v), and mass matrix (∂G/∂a). The corresponding
   row and column for a degree of freedom with Dirichlet boundary condition in
   the tangent matrix is set to zero with the exception of the diagonal entries
   which is set to a scalar multiple of identity.
   @param[in] fem_state        The FemState used to evaluate the tangent matrix.
   @param[out] tangent_matrix  The output tangent_matrix.
   @pre tangent_matrix != nullptr.
   @pre The size of `tangent_matrix` is `num_dofs()` * `num_dofs()`.
   @pre All nonzero entries in the resulting tangent matrix have been allocated.
   See MakeTangentMatrix().
   @warning This function sometimes makes simplifying approximations to avoid
   taking overly complicated derivatives. As such, the resulting tangent
   matrix may be an approximation of the actual value depending on the
   constitutive model used.
   @throws std::exception if the FEM state is incompatible with this model.
   @throws std::exception if T is not double. */
  void CalcTangentMatrix(
      const FemState<T>& fem_state,
      contact_solvers::internal::BlockSparseSymmetricMatrix3d* tangent_matrix)
      const;

  /** Calculates the position vector from the world origin Wo to the center
   of mass of all bodies in this FemModel S, expressed in the world frame W.
   @param[in] fem_state The FemState used to evaluate the center of mass.
   @retval p_WoScm_W position vector from Wo to Scm expressed in world frame W,
   where Scm is the center of mass of the system S stored by `this` FemModel.
   @throws std::exception if the FEM state is incompatible with this model. */
  Vector3<T> CalcCenterOfMassPositionInWorld(
      const FemState<T>& fem_state) const;

  /** Calculates system center of mass translational velocity in world frame W.
   @param[in] fem_state The FemState used for this calculation.
   @retval v_WScm_W Scm's translational velocity in frame W, expressed in W,
   where Scm is the center of mass of the system S stored by this FemModel.
   @throws std::exception if the FEM state is incompatible with this model. */
  Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const FemState<T>& fem_state) const;

  /** Using an angular momentum analogy, calculates an "effective" angular
   velocity for this FemModel S, measured and expressed in the world frame W.
   The effective angular velocity is computed using an angular momentum equation
   that assumes S is a rigid body (albeit we know S is deformable).

        H_WSSm_W = I_SScm_W * w_WScm_W

   for which when solved for w_WScm_W gives

        w_WScm_W = inverse(I_SScm_W) * H_WSSm_W

   where H_WSSm_W is the FemModel S's angular momentum about its center of mass
   Scm measured and expressed in the world frame W.
   @param[in] fem_state The FemState used for this calculation.
   @retval w_WScm_W the FemModel S's effective angular velocity for Scm,
   measured and expressed in the world frame W.
   @throws std::exception if the FEM state is incompatible with this model. */
  Vector3<T> CalcEffectiveAngularVelocity(const FemState<T>& fem_state) const;

  /** Creates a symmetric block sparse matrix that has the sparsity pattern
   of the tangent matrix of this FEM model. In particular, the size of the
   tangent matrix is `num_dofs()` by `num_dofs()`. All entries are initialized
   to zero.
   @throws std::exception if T is not double. */
  std::unique_ptr<contact_solvers::internal::BlockSparseSymmetricMatrix3d>
  MakeTangentMatrix() const;

  /** Applies boundary condition set for this %FemModel to the input `state`.
   No-op if no boundary condition is set.
   @pre fem_state != nullptr.
   @throws std::exception if the FEM state is incompatible with this model. */
  void ApplyBoundaryCondition(FemState<T>* fem_state) const;

  // TODO(xuchenhan-tri): Internal object in public signature in non-internal
  //  class.
  /** Sets the Dirichlet boundary condition that this model is subject to.
   If dirichlet_bc specifies the boundary condition for a node for which a
   boundary condition has already been specified, the lastest one will be used.
  */
  void SetDirichletBoundaryCondition(
      const internal::DirichletBoundaryCondition<T>& dirichlet_bc) {
    dirichlet_bc_.Merge(dirichlet_bc);
  }

  /** Returns the Dirichlet boundary condition that this model is subject to. */
  const internal::DirichletBoundaryCondition<T>& dirichlet_boundary_condition()
      const {
    return dirichlet_bc_;
  }

  /** Returns true the equation G(x, v, a) = 0 (see class documentation)
   corresponding to this %FemModel is linear. */
  bool is_linear() const { return do_is_linear(); }

  /** Returns true if the given FEM state is compatible with `this` FEM model.
   */
  bool is_compatible_with(const FemState<T>& state) const {
    return state.is_created_from_system(*fem_state_system_);
  }

  /* The weights used to combine stiffness, damping, and mass matrices (in that
   order) to form the tangent matrix . */
  const Vector3<T>& tangent_matrix_weights() const {
    return tangent_matrix_weights_;
  }

  /** (Internal use only) Throws std::exception to report a mismatch between
  the FEM model and state that were passed to API method `func`. */
  void ThrowIfModelStateIncompatible(const char* func,
                                     const FemState<T>& fem_state) const;

  // TODO(xuchenhan-tri): Restrict the entry point to parallelism configuration
  // so that's more difficult to misuse. We want to avoid, for example, a user
  // calling SetParallelism(Parallelism::Max()) on a model that has already been
  // parallelized at a higher level.
  /** (Internal use only) Configures the parallelism that `this` %FemModel uses
   when opportunities for parallel computation arises. */
  void set_parallelism(Parallelism parallelism) { parallelism_ = parallelism; }

  /** (Internal use only) Returns the parallelism that `this` %FemModel uses
   when opportunities for parallel computation arises. */
  Parallelism parallelism() const { return parallelism_; }

  /** Returns the total mass of the system. */
  T get_total_mass() const { return total_mass_; }

 protected:
  /** Constructs an empty FEM model.
   @pre tangent_matrix_weights.minCoeff() >= 0.0. */
  explicit FemModel(const Vector3<T>& tangent_matrix_weights);

  /** FemModelImpl must override this method to provide an implementation to
   make a deep copy of the concrete FemModel. */
  virtual std::unique_ptr<FemModel<T>> DoClone() const = 0;

  /** Returns the reference positions of this model. */
  virtual VectorX<T> MakeReferencePositions() const = 0;

  /** FemModelImpl must override this method to provide an implementation
   for the NVI CalcResidual(). The input `fem_state` is guaranteed to be
   compatible with `this` FEM model, and the input `residual` is guaranteed to
   be non-null and properly sized. */
  virtual void DoCalcResidual(const FemState<T>& fem_state,
                              const FemPlantData<T>& plant_data,
                              EigenPtr<VectorX<T>> residual) const = 0;

  /** FemModelImpl must override this method to provide an implementation for
   the NVI CalcTangentMatrix(). The input `fem_state` is guaranteed to be
   compatible with `this` FEM model, and the input `tangent_matrix` is
   guaranteed to be non-null and properly sized. */
  virtual void DoCalcTangentMatrix(
      const FemState<T>& fem_state,
      contact_solvers::internal::BlockSparseSymmetricMatrix3d* tangent_matrix)
      const = 0;

  /** FemModelImpl must override this method to provide an implementation for
   the NVI CalcCenterOfMassPositionInWorld(). The input `fem_state` is
   guaranteed to be compatible with `this` FEM model. */
  virtual Vector3<T> DoCalcCenterOfMassPositionInWorld(
      const FemState<T>& fem_state) const = 0;

  /** FemModelImpl must override this method to provide an implementation for
   the NVI CalcCenterOfMassTranslationalVelocityInWorld(). The input `fem_state`
   is guaranteed to be compatible with `this` FEM model. */
  virtual Vector3<T> DoCalcCenterOfMassTranslationalVelocityInWorld(
      const FemState<T>& fem_state) const = 0;

  /** FemModelImpl must override this method to provide an implementation for
   the NVI CalcEffectiveAngularVelocity(). The input `fem_state`
   is guaranteed to be compatible with `this` FEM model. */
  virtual Vector3<T> DoCalcEffectiveAngularVelocity(
      const FemState<T>& fem_state) const = 0;

  /** FemModelImpl must override this method to provide an implementation for
   the NVI MakeTangentMatrix(). */
  virtual std::unique_ptr<
      contact_solvers::internal::BlockSparseSymmetricMatrix3d>
  DoMakeTangentMatrix() const = 0;

  /** Updates the system that manages the states and the cache entries of this
   FEM model. Must be called before calling MakeFemState() after the FEM model
   changes (e.g. adding new elements). */
  void UpdateFemStateSystem();

  /** Derived classes should override this method to declare cache entries in
   the given `fem_state_system`. */
  virtual void DeclareCacheEntries(
      internal::FemStateSystem<T>* fem_state_system) = 0;

  /** Derived classes should override this method to indicate if the model is
   linear. */
  virtual bool do_is_linear() const = 0;

  /** Returns the FemStateSystem that manages the states and cache entries in
   this %FemModel. */
  const internal::FemStateSystem<T>& fem_state_system() const {
    return *fem_state_system_;
  }

 private:
  /** Computes the total mass of the system. FemModelImpl must override this
   method to provide an implementation for the NVI CalcTotalMass(). */
  virtual T DoCalcTotalMass() const = 0;

  /* The system that manages the states and cache entries of this FEM model.
   */
  std::unique_ptr<internal::FemStateSystem<T>> fem_state_system_;
  /* The Dirichlet boundary condition that the model is subject to. */
  internal::DirichletBoundaryCondition<T> dirichlet_bc_;
  /* The weights used to combine stiffness, damping, and mass matrices (in that
   order) to form the tangent matrix. */
  Vector3<T> tangent_matrix_weights_;
  Parallelism parallelism_{false};
  /* The total mass of the system. */
  T total_mass_{};
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
