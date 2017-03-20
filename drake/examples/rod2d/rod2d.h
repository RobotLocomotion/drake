#pragma once

#include <memory>
#include <utility>

#include "drake/solvers/moby_lcp_solver.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace rod2d {

/** Dynamical system representation of a rod contacting a half-space in
two dimensions.

<h3>Notation</h3>
In the discussion below and in code comments, we will use the 2D analog of our
standard multibody notation as described in detail here:
@ref multibody_notation.
<!-- http://drake.mit.edu/doxygen_cxx/group__multibody__notation.html -->

For a quick summary and translation to 2D:
 - When we combine rotational and translational quantities into a single
   quantity in 3D, we call those "spatial" quantities. In 2D those combined
   quantities are actually planar, but we will continue to refer to them as
   "spatial" to keep the notation analogous and promote easy extension of 2D
   pedagogical examples to 3D.
 - We use capital letters to represent bodies and coordinate frames. Frame F has
   an origin point Fo, and a basis formed by orthogonal unit vector axes Fx and
   Fy, with an implicit `Fz=Fx × Fy` always pointing out of the screen for a 2D
   system. The inertial frame World is W, and the rod frame is R.
 - We also use capitals to represent points, and we allow a frame name F to be
   used where a point is expected to represent its origin Fo.
 - We use `p_CD` to represent the position vector from point C to point D. Note
   that if A and B are frames, `p_AB` means `p_AoBo`.
 - If we need to be explicit about the expressed-in frame F for any quantity, we
   add the suffix `_F` to its symbol. So the position vector from C to D,
   expressed in W, is `p_CD_W`.
 - R_AB is the rotation matrix giving frame B's orientation in frame A.
 - X_AB is the transformation matrix giving frame B's pose in frame A, combining
   both a rotation and a translation; this is conventionally called a
   "transform". A transform is a spatial quantity.

In 2D, with frames A and B the above quantities are (conceptually) matrices
with the indicated dimensions: <pre>
    p_AB = Bo-Ao = |x|      R_AB=| cθ -sθ |       X_AB=| R_AB p_AB |
                   |y|₂ₓ₁        | sθ  cθ |₂ₓ₂         | 0  0   1  |₃ₓ₃
</pre>
where x,y are B's Cartesian coordinates in the A frame, and θ is the
counterclockwise angle from Ax to Bx, measured about Az (and Bz). In practice,
2D rotations are represented just by the scalar angle θ, and 2D transforms are
represented by (x,y,θ).

We use v for translational velocity of a point and w (ω) for rotational
velocity of a frame. The symbols are:
 - `v_AP` is point P's velocity in frame A, expressed in frame A if no
   other frame is given as a suffix.
 - `w_AB` is frame B's angular velocity in frame A, expressed in frame A
   if no other frame is given as a suffix.
 - `V_AB` is frame B's spatial velocity in A, meaning `v_ABo` and `w_AB`.


These quantities are conceptually: <pre>
    v_AB = |vx|      w_AB=|0|       V_AB=| w_AB |
           |vy|           |0|            | v_AB |₆ₓ₁
           | 0|₃ₓ₁        |ω|₃ₓ₁
</pre>
but in 2D we represent translational velocity with just (vx,vy), angular
velocity with just the scalar w=ω=@f$\dot{\theta}@f$ (that is, d/dt θ), and
spatial velocity as (vx,vy,ω).

Forces f and torques τ are represented similarly:
 - `f_P` is an in-plane force applied to a point P fixed to some rigid body.
 - `t_A` is an in-plane torque applied to frame A (meaning it is about Az).
 - `F_A` is a spatial force including both `f_Ao` and `t_A`.

The above symbols can be suffixed with an expressed-in frame if the frame is
not already obvious, so `F_A_W` is a spatial force applied to frame A (at Ao)
but expressed in W. These quantities are conceptually: <pre>
    f_A = |fx|      t_A=|0|       F_A=| t_A |
          |fy|          |0|           | f_A |₆ₓ₁
          | 0|₃ₓ₁       |τ|₃ₓ₁
</pre>
but in 2D we represent translational force with just (fx,fy), torque with just
the scalar t=τ, and spatial force as (fx,fy,τ).

<h3>The 2D rod model</h3>
The rod's coordinate frame R is placed at the rod's center point Ro, which
is also its center of mass Rcm. R's planar pose is given by a planar
transform X_WR=(x,y,θ). When X_WR=0 (identity transform), R is coincident
with the World frame W, and aligned horizontally as shown: <pre>

       +Wy                                  +Ry
        |                                    |
        |                                    |<---- h ----->
        |                      ==============|==============
      Wo*-----> +Wx         Rl*            Ro*-----> +Rx    *Rr
                               =============================
       World frame                      Rod R, θ=0
</pre>
θ is the angle between Rx and Wx, measured using the right hand rule about
Wz (out of the screen), that is, counterclockwise. The rod has half-length
h, and "left" and "right" endpoints `Rl=Ro-h*Rx` and `Rr=Ro+h*Rx` at which
it can contact the halfspace whose surface is at Wy=0.

This system can be simulated using one of three models:
- a compliant contact model (the rod is rigid, but contact between
  the rod and the half-space is modeled as compliant) simulated using
  ordinary differential equations (ODEs),
- a fully rigid model simulated with piecewise differential algebraic
  equations (DAEs), and
- a fully rigid model simulated as a discrete system using a first-order
  time stepping approach.

The rod state is initialized to the configuration that corresponds to the
Painlevé Paradox problem, described in [Stewart 2000]. The paradox consists
of a rod contacting a planar surface *without impact* and subject to sliding
Coulomb friction. The problem is well known to correspond to an
*inconsistent rigid contact configuration*, where impulsive forces are
necessary to resolve the problem.

This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
this class, please refer to http://drake.mit.edu/cxx_inl.html.

@tparam T The vector element type, which must be a valid Eigen scalar.

Instantiated templates for the following scalar types @p T are provided:
- double

They are already available to link against in the containing library.

Inputs: planar force (two-dimensional) and torque (scalar), which are
        arbitrary "external" forces (expressed in the world frame) applied
        at the center-of-mass of the rod.

States: planar position (state indices 0 and 1) and orientation (state
        index 2), and planar linear velocity (state indices 3 and 4) and
        scalar angular velocity (state index 5) in units of m, radians,
        m/s, and rad/s, respectively. Orientation is measured counter-
        clockwise with respect to the x-axis. For simulations using the
        piecewise DAE formulation, one abstract state variable
        (of type Rod2D::Mode) is used to identify which dynamic mode
        the system is in (e.g., ballistic, contacting at one point and
        sliding, etc.) and one abstract state variable (of type int) is used
        to determine which endpoint(s) of the rod contact the halfspace
        (k=-1 indicates the left endpoint Rl, k=+1 indicates the right
        endpoint Rr, and k=0 indicates that both endpoints of the rod are
        contacting the halfspace).

Outputs: planar position (state indices 0 and 1) and orientation (state
         index 2), and planar linear velocity (state indices 3 and 4) and
         scalar angular velocity (state index 5) in units of m, radians,
         m/s, and rad/s, respectively.

- [Stewart, 2000]  D. Stewart, "Rigid-Body Dynamics with Friction and
                   Impact". SIAM Rev., 42(1), 3-39, 2000. **/
template <typename T>
class Rod2D : public systems::LeafSystem<T> {
 public:
  /// Simulation model and approach for the system.
  enum class SimulationType {
    /// For simulating the system using rigid contact, Coulomb friction, and
    /// piecewise differential algebraic equations.
    kPiecewiseDAE,

    /// For simulating the system using rigid contact, Coulomb friction, and
    /// a first-order time stepping approach.
    kTimeStepping,

    /// For simulating the system using compliant contact, Coulomb friction,
    /// and ordinary differential equations.
    kCompliant
  };

  /// Possible dynamic modes for the 2D rod.
  enum Mode {
    /// Mode is invalid.
    kInvalid,

    /// Rod is currently undergoing ballistic motion.
    kBallisticMotion,

    /// Rod is sliding while undergoing non-impacting contact at one contact
    /// point (a rod endpoint); the other rod endpoint is not in contact.
    kSlidingSingleContact,

    /// Rod is sticking while undergoing non-impacting contact at one contact
    /// point (a rod endpoint); the other rod endpoint is not in contact.
    kStickingSingleContact,

    /// Rod is sliding at two contact points without impact.
    kSlidingTwoContacts,

    /// Rod is sticking at two contact points without impact.
    kStickingTwoContacts
  };

  /// Constructor for the 2D rod system using the piecewise DAE (differential
  /// algebraic equation) based approach, the time stepping approach, or the
  /// compliant ordinary differential equation based approach.
  /// @param dt The integration step size. This step size cannot be reset
  ///           after construction.
  /// @throws std::logic_error if @p dt is not positive and simulation_type is
  ///         kTimeStepping or @p dt is not zero and simulation_type is
  ///         kPiecewiseDAE or kCompliant.
  explicit Rod2D(SimulationType simulation_type, double dt);

  /// Gets the constraint force mixing parameter (CFM, used for time stepping
  /// systems only).
  double get_cfm() const { return cfm_; }

  /// Sets the constraint force mixing parameter (CFM, used for time stepping
  /// systems only). The default CFM value is 1e-8.
  /// @param cfm a floating point value in the range [0, infinity].
  /// @throws std::logic_error if this is not a time stepping system or if
  ///         cfm is set to a negative value.
  void set_cfm(double cfm) {
    if (simulation_type_ != SimulationType::kTimeStepping)
      throw std::logic_error("Attempt to set CFM for non-time stepping "
                             "system.");
    if (cfm < 0)
      throw std::logic_error("Negative CFM value specified.");
    cfm_ = cfm;
  }

  /// Gets the error reduction parameter (ERP, used for time stepping systems
  /// only).
  double get_erp() const { return erp_; }

  /// Sets the error reduction parameter (ERP, used for time stepping systems
  /// only). The default ERP value is 0.8.
  /// @param erp a floating point value in the range [0, 1].
  /// @throws std::logic_error if this is not a time stepping system or if
  ///         erp is set to a negative value.
  void set_erp(double erp) {
    if (simulation_type_ != SimulationType::kTimeStepping)
      throw std::logic_error("Attempt to set ERP for non-time stepping "
                             "system.");
    if (erp < 0 || erp > 1)
      throw std::logic_error("Invalid ERP value specified.");
    erp_ = erp;
  }

  /// Models impact using an inelastic impact model with friction.
  /// @p new_state is set to the output of the impact model on return.
  void HandleImpact(const systems::Context<T>& context,
                    systems::State<T>* new_state) const;

  /// Gets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  double get_gravitational_acceleration() const { return g_; }

  /// Sets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  void set_gravitational_acceleration(double g) { g_ = g; }

  /// Gets the coefficient of dynamic (sliding) Coulomb friction.
  double get_mu_coulomb() const { return mu_; }

  /// Sets the coefficient of dynamic (sliding) Coulomb friction.
  void set_mu_coulomb(double mu) { mu_ = mu; }

  /// Gets the mass of the rod.
  double get_rod_mass() const { return mass_; }

  /// Sets the mass of the rod.
  void set_rod_mass(double mass) { mass_ = mass; }

  /// Gets the half-length h of the rod.
  double get_rod_half_length() const { return half_length_; }

  /// Sets the half-length h of the rod.
  void set_rod_half_length(double half_length) { half_length_ = half_length; }

  /// Gets the rod moment of inertia.
  double get_rod_moment_of_inertia() const { return J_; }

  /// Sets the rod moment of inertia.
  void set_rod_moment_of_inertia(double J) { J_ = J; }

  /// Get compliant contact normal stiffness in N/m.
  double get_stiffness() const { return stiffness_; }

  /// Set compliant contact normal stiffness in N/m (>= 0). This has no effect
  /// if the rod model is not compliant.
  void set_stiffness(double stiffness) {
    DRAKE_DEMAND(stiffness >= 0);
    stiffness_ = stiffness;
  }

  /// Get compliant contact normal dissipation in 1/velocity (s/m).
  double get_dissipation() const { return dissipation_; }

  /// Set compliant contact normal dissipation in 1/velocity (s/m, >= 0). This
  /// has no effect if the rod model is not compliant.
  void set_dissipation(double dissipation) {
    DRAKE_DEMAND(dissipation >= 0);
    dissipation_ = dissipation;
  }

  /// Get compliant contact static friction (stiction) coefficient `μ_s`.
  double get_mu_static() const { return mu_s_; }

  /// Set compliant contact stiction coefficient (>= mu_coulomb). This has no
  /// effect if the rod model is not compliant.
  void set_mu_static(double mu_static) {
    DRAKE_DEMAND(mu_static >= mu_);
    mu_s_ = mu_static;
  }

  /// Get the stiction speed tolerance (m/s).
  double get_stiction_speed_tolerance() const {return v_stick_tol_;}

  /// Set the stiction speed tolerance (m/s). This is the maximum slip
  /// speed that we are willing to consider as sticking. For a given normal
  /// force N this is the speed at which the friction force will be largest,
  /// at `μ_s*N` where `μ_s` is the static coefficient of friction. This has no
  /// effect if the rod model is not compliant.
  void set_stiction_speed_tolerance(double v_stick_tol) {
    DRAKE_DEMAND(v_stick_tol > 0);
    v_stick_tol_ = v_stick_tol;
  }

  /// Checks whether the system is in an impacting state, meaning that the
  /// relative velocity along the contact normal between the rod and the
  /// halfspace is such that the rod will begin interpenetrating the halfspace
  /// at any time Δt in the future (i.e., Δt > 0). If the context does not
  /// correspond to a configuration where the rod and halfspace are contacting,
  /// this method returns `false`.
  bool IsImpacting(const systems::Context<T>& context) const;

  /// Gets the integration step size for the time stepping system.
  /// @returns 0 if this is a DAE-based system.
  double get_integration_step_size() const { return dt_; }

  /// Gets the model and simulation type for this system.
  SimulationType get_simulation_type() const { return simulation_type_; }

  /// Return net contact forces as a spatial force F_Ro_W=(fx,fy,τ) where
  /// translational force f_Ro_W=(fx,fy) is applied at the rod origin Ro,
  /// and torque t_R=τ is the moment due to the contact forces actually being
  /// applied elsewhere. The returned spatial force may be the resultant of
  /// multiple active contact points. Only valid for simulation type kCompliant.
  Vector3<T> CalcCompliantContactForces(
      const systems::Context<T>& context) const;

 protected:
  int get_k(const systems::Context<T>& context) const;
  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;
  void DoCalcTimeDerivatives(const systems::Context<T>& context,
                             systems::ContinuousState<T>* derivatives)
                               const override;
  void DoCalcDiscreteVariableUpdates(const systems::Context<T>& context,
                                     systems::DiscreteState<T>* discrete_state)
      const override;
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 private:
  Vector2<T> CalcStickingImpactImpulse(const systems::Context<T>& context)
    const;
  Vector2<T> CalcFConeImpactImpulse(const systems::Context<T>& context) const;
  void CalcAccelerationsCompliantContactAndBallistic(
                                  const systems::Context<T>& context,
                                  systems::ContinuousState<T>* derivatives)
                                    const;
  void CalcAccelerationsBallistic(const systems::Context<T>& context,
                                  systems::ContinuousState<T>* derivatives)
                                    const;
  void CalcAccelerationsTwoContact(const systems::Context<T>& context,
                                   systems::ContinuousState<T>* derivatives)
                                     const;
  void CalcAccelerationsOneContactNoSliding(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const;
  void CalcAccelerationsOneContactSliding(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const;
  void SetAccelerations(const systems::Context<T>& context,
                        systems::VectorBase<T>* const f,
                        const T& fN, const T& fF,
                        const T& xc, const T& yc) const;
  Vector2<T> CalcStickingContactForces(
      const systems::Context<T>& context) const;


  // Utility method for determining the World frame location of one of three
  // points on the rod whose origin is Ro. Let r be the half-length of the rod.
  // Define point P = Ro+k*r where k = { -1, 0, 1 }. This returns p_WP.
  static Vector2<T> CalcRodEndpoint(const T& x, const T& y, const int k,
                                    const T& ctheta, const T& stheta,
                                    const double half_rod_len);

  // Given a location p_WC of a point C in the World frame, define the point Rc
  // of the rod that is coincident with C, and report Rc's World frame velocity
  // v_WRc. We're given p_WRo=(x,y) and V_WRo=(v_WRo,w_WR)=(xdot,ydot,thetadot).
  static Vector2<T> CalcCoincidentRodPointVelocity(
      const Vector2<T>& p_WRo, const Vector2<T>& v_WRo,
      const T& w_WR,  // aka thetadot
      const Vector2<T>& p_WC);

  // 2D cross product returns a scalar. This is the z component of the 3D
  // cross product [ax ay 0] × [bx by 0]; the x,y components are zero.
  static T cross2(const Vector2<T>& a, const Vector2<T>& b) {
    return a[0]*b[1] - b[0]*a[1];
  }

  // Calculates the cross product [0 0 ω] × [rx ry 0] and returns the first
  // two elements of the result (the z component is zero).
  static Vector2<T> w_cross_r(const T& w, const Vector2<T>& r) {
    return w * Vector2<T>(-r[1], r[0]);
  }

  // Quintic step function approximation used by Stribeck friction model.
  static T step5(const T& x);

  // Friction model used in compliant contact.
  static T CalcMuStribeck(const T& us, const T& ud, const T& v);

  // Solves linear complementarity problems for time stepping.
  solvers::MobyLCPSolver<T> lcp_;

  // The simulation type, unable to be changed after object construction.
  const SimulationType simulation_type_;

  // TODO(edrumwri,sherm1) Document these defaults once they stabilize.

  double dt_{0.};           // Integration step-size for time stepping approach.
  double mass_{1.};         // The mass of the rod (kg).
  double half_length_{1.};  // The length of the rod (m).
  double mu_{1000.};        // The (dynamic) coefficient of friction.
  double g_{-9.81};         // The acceleration due to gravity (in y direction).
  double J_{1.};            // The moment of the inertia of the rod.
  double erp_{0.8};         // ERP for time stepping systems.
  double cfm_{1e-8};        // CFM for time stepping systems.

  // Compliant contact parameters.
  double stiffness_{10000};   // Normal stiffness of the ground plane (N/m).
  double dissipation_{1};     // Dissipation factor in 1/velocity (s/m).
  double mu_s_{mu_};          // Static coefficient of friction (>= mu).
  double v_stick_tol_{1e-3};  // Slip speed below which the compliant model
                              //   considers the rod to be in stiction.
};

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
