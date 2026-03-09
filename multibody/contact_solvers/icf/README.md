# Irrotational Contact Fields (ICF)

ICF solves a convex optimization problem to advance the state ($q$, $v$) of a
multibody plant

```math
   M(q)\dot{v} + k(q, v) = J(q)^Tf,
````
where $q, v$ are generalized positions and velocities, $M$ is the mass matrix,
$k$ collects nonlinear and external forces (e.g., gravity, gyroscopic and
Coriolis terms, etc.), $f$ are constraint (contact) forces, and $J$ is the
constraint Jacobian.

Full details can be found in [Castro et al., 2023]. The CENIC integrator [Kurtz
et al. 2025] builds on ICF to perform error-controlled integration.

## Math Overview

ICF computes next-step velocities $v_{n+1}$ via unconstrained convex
optimization,
```math
  v_{n+1} = \arg\min_v \ell(v;q_n, v_n, \delta t),
```
where $\delta t$ is the time step and ($q_n, v_n$) is the previous state. The
cost is
```math
\ell(v; q_n, v_n, \delta t) = \frac{1}{2}v^TAv - r^Tv + \ell_c(v;q_n, v_n, \delta t),
```
where $A = M(q_n)$, $r = A v_n - \delta t k(q_n, v_n)$, and $\ell_c$ is a convex
contact constraint potential. 

> **Note**
> In practice we use $A = M(q_n) + \delta t K$, where $K$ is a diagonal positive
> semi-definite matrix, to implicitly account for terms like joint damping and
> rotor inertia. This document assumes $K = 0$ for simplicity of presentation.

The optimality conditions $\nabla_v \ell = 0$
ensure that $v_{n+1}$ respects the discrete momentum balance (assuming $K = 0$
for simplicity)
```math
  M(q_n)(v_{n+1} - v_n) + \delta t k(q_n, v_n) = J(q_n)^T\gamma(v_{n+1}),
```
where $\gamma$ are contact impulses defined such that $\nabla_v \ell_c(v; q_n,
v_n \delta t) = - J(q_n)^T\gamma(v)$. Having solved for $v_{n+1}$, we can update
configurations with the semi-explicit Euler strategy
```math
  q_{n+1} = q_n + \delta t N(q_n) v_{n+1},
```
where $N(q_n)$ is the kinematic map such that $\dot{q} = N(q)v$.

In addition to contact, ICF can support other constraint types (e.g., joint
limits, couplers, etc.) via appropriately designed convex potentials. For full
details, see [Castro et al., 2023] and [Kurtz and Castro, 2025].

## About this Implementation

This implementation of ICF focuses on simplicity and performance:
   - Contiguous memory layouts via constraint "pools" for better cache locality.
   - Memory allocations are performed once at simulation startup, typically with
     overestimation. Reallocation is permitted but should occur minimally.
   - Contact constraints are grouped into patches, enabling re-use of per-patch
     computations and reducing FLOP count.
   - Contact constraints are expressed directly in world coordinates,
     eliminating expensive intermediate frame computations.

Key components and how they talk to each other:

   - `IcfModel`: Represents the convex ICF problem $\min \ell(v; q_n, v_n,
     \delta t)$ for a given state $q_n, v_n$ and time step $\delta t$. The model
     is constant during the optimization process, e.g., for different values of
     decision variables $v$.
   - `IcfData`: Holds all variable data that changes during the optimization
     process, e.g., $v$ and derived quantities.
   - `IcfSolver`: Solves the convex problem itself, e.g., does
     Newton iterations.
   - TODO(#23769): `IcfBuilder`: Constructs the optimization problem. The
     builder (along with the related `IcfExternalSystemsLinearizer`) are the
     only components that know about `MultibodyPlant`.

Other components:

   - Constraint pools (`CouplerConstraintsPool`, `GainConstraintsPool`,
     `LimitConstraintsPool`, `PatchConstraintsPool`) are part of
     `IcfModel`, and hold constraints of various types.
   - Similarly, constraint data pools (`CouplerConstraintsDataPool`,
     `GainConstraintsDataPool`, `LimitConstraintsDataPool`,
     `PatchConstraintsDataPool`) are part of `IcfData`, and hold data that
     change with $v$ for the corresponding constraints.
   - The underlying `EigenPool` datatype is used to store constraint quantities
     in contiguous memory blocks. `EigenPool` essentially provides something
     like `std::vector<MatrixX<T>>`, but with contiguous storage.

## References:

[Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023. Irrotational
Contact Fields. https://arxiv.org/abs/2312.03908.

[Hairer and Wanner, 1996] Hairer E. and Wanner G., 1996. Solving Ordinary
Differential Equations II: Stiff and Differential-Algebraic Problems. Springer
Series in Computational Mathematics, Vol. 14. Springer-Verlag, Berlin, 2nd
edition.

[Kurtz and Castro, 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
Error-controlled Numerical Integration for Contact.
https://arxiv.org/abs/2511.08771.

[Masterjohn et al., 2022] Masterjohn, J., Guoy, D., Shepherd, J. and Castro,
A., 2022. Velocity level approximation of pressure field contact patches. IEEE
Robotics and Automation Letters, 7(4), pp.11593-11600.
