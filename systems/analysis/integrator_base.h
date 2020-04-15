#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/** @addtogroup simulation
 @{
 @defgroup integrators Integrators

 Apart from solving initial value problems, for which the integrator is a
 key component of a simulator, integrators can also be used to solve
 boundary value problems (via numerical methods like the Multiple Shooting
 Method) and trajectory optimization problems (via numerical methods like
 direct transcription). IntegratorBase and its derivatives were developed
 primarily toward the former application (through
 IntegratorBase::IntegrateNoFurtherThanTime() and the Simulator class).
 However, the IntegratorBase architecture was developed to support these
 ancillary applications as well using the
 IntegratorBase::IntegrateWithMultipleStepsToTime() and
 IntegratorBase::IntegrateWithSingleFixedStepToTime() methods; the latter
 permits the caller to advance time using fixed steps in applications where
 variable stepping would be deleterious (e.g., direct transcription).

 @section integrator-selection Integrator selection
 A natural question for a user to ask of an integrator is: Which scheme
 (method) should be applied to a particular problem? The answer is whichever
 one most quickly computes the solution to the desired accuracy! Selecting
 an integration scheme for a particular problem is presently an artform. As
 examples of some selection criteria: multistep methods (none of which are
 currently implemented in Drake) generally work poorly when events (that
 require state reinitializations) are common, symplectic methods generally
 work well at maintaining stability for large integration steps, and stiff
 integrators are often best for computationally stiff systems. If ignorant
 as to the characteristics of a particular problem, it is often best to start
 with an explicit, Runge-Kutta type method. Statistics collected by the
 integrator can help diagnose performance issues and possibly point to the
 use of a different integration scheme.

 Some systems are known to exhibit "computational stiffness", by which it is
 meant that (excessively) small integration steps are necessary for purposes
 of stability: in other words, steps must be taken smaller than that
 required to achieve a desired accuracy *over a particular interval*.
 Thus, the nature of computationally stiff problems is that the solution to
 the ODE is *smooth* in the interval of stiffness (in contrast, some problems
 possess such high frequency dynamics that very small steps are simply
 necessary to capture the solution accurately). Implicit integrators are the
 go-to approach for solving computationally stiff problems, but careful
 consideration is warranted. Implicit integrators typically require much more
 computation than non-implicit (explicit) integrators, stiffness might be an
 issue on only a very small time interval, and some problems might be only
 "moderately stiff". Put another way, applying an implicit integrator to a
 potentially stiff problem might not yield faster computation. The first
 chapter of [Hairer, 1996] illustrates the issues broached in this paragraph
 using various examples.

 @section settings Integrator settings
 IntegratorBase provides numerous settings and flags that can leverage
 problem-specific information to speed integration and/or improve integration
 accuracy. As an example, IntegratorBase::set_maximum_step_size() allows the
 user to prevent overly large integration steps (that integration error
 control alone might be insufficient to detect). As noted previously,
 IntegratorBase also collects a plethora of statistics that can be used to
 diagnose poor integration performance. For example, a large number of
 shrinkages due to @ref error-estimation-and-control "error control" could
 indicate that a system is computationally stiff. **Note that you might need
 to alter the default settings to obtain desired performance even though we
 have attempted to select reasonable defaults for many problems.**

 See settings for @ref integrator-accuracy,
 @ref integrator-maxstep "maximum step size",
 @ref integrator-minstep "minimum step size", and
 @ref weighting-state-errors "weighting state errors" for
 in-depth information about the various performance settings shared across
 integrators.

 @section dense-sampling Dense sampling (interpolation)
 For applications that require a more dense sampling of the system
 continuous state than what would be available through either fixed or
 error-controlled step integration (for a given accuracy), dense output
 support is available (through IntegratorBase::StartDenseIntegration() and
 IntegratorBase::StopDenseIntegration() methods). The accuracy and performance
 of these outputs may vary with each integration scheme implementation.

 @section references References
  - [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
                     Equations II (Stiff and Differential-Algebraic Problems).
                     Springer, 1996.
 @}
 */

/**
 An abstract class for an integrator for ODEs and DAEs as represented by a
 Drake System. Integrators solve initial value problems of the form:<pre>
 ẋ(t) = f(t, x(t)) with f : ℝ × ℝⁿ → ℝⁿ
 </pre>
 (i.e., `f()` is an ordinary differential equation) given initial conditions
 (t₀, x₀). Thus, integrators advance the continuous state of a dynamical
 system forward in time.

 Drake's subclasses of IntegratorBase<T> should follow the naming pattern
 `FooIntegrator<T>` by convention.

 @tparam_default_scalar
 @ingroup integrators
 */
template <class T>
class IntegratorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntegratorBase)

  /**
   Status returned by StepOnceAtMost().
   When a step is successful, it will return an indication of what caused it
   to stop where it did. When unsuccessful it will throw an exception so you
   won't see any return value. When return of control is due ONLY to reaching
   a publish time, (status is kReachedPublishTime) the context may return an
   interpolated value at an earlier time.

   @note the simulation step must always end at an update time but can end
   after a publish time.
   */
  // TODO(edrumwri): incorporate kReachedZeroCrossing into the simulator.
  enum StepResult {
    /// Indicates a publish time has been reached but not an update time.
    kReachedPublishTime = 1,
    /// Localized an event; this is the *before* state (interpolated).
    kReachedZeroCrossing = 2,
    /// Indicates that integration terminated at an update time.
    kReachedUpdateTime = 3,
    /// User requested control whenever an internal step is successful.
    kTimeHasAdvanced = 4,
    /// Reached the desired integration time without reaching an update time.
    kReachedBoundaryTime = 5,
    /// Took maximum number of steps without finishing integrating over the
    /// interval.
    kReachedStepLimit = 6,
  };

  /**
   Maintains references to the system being integrated and the context used
   to specify the initial conditions for that system (if any).
   @param system A reference to the system to be integrated; the integrator
                 will maintain a reference to the system in perpetuity, so
                 the integrator must not outlive the system.
   @param context A pointer to a writeable context (nullptr is ok, but a
                  non-null pointer must be set before Initialize() is
                  called). The integrator will advance the system state using
                  the pointer to this context. The pointer to the context will
                  be maintained internally. The integrator must not outlive
                  the context.
   */
  explicit IntegratorBase(const System<T>& system,
                          Context<T>* context = nullptr)
      : system_(system), context_(context) {
    initialization_done_ = false;
  }

  virtual ~IntegratorBase() = default;

  /**
   @anchor integrator-accuracy
   @name Methods for getting and setting integrator accuracy
   The precise meaning of *accuracy* is a complicated discussion, but it
   translates roughly to the number of significant digits you want in the
   results. By convention it is supplied as `10^-digits`, meaning that an
   accuracy of 1e-3 provides about three significant digits. For continued
   discussion of accuracy, see [Sherman 2011].

   Integrators vary in the range of accuracy (loosest to tightest) that they
   can support, and each integrator will choose a default accuracy to be used
   that lies somewhere within this range and attempts to balance computation
   and accuracy. If you request accuracy outside the supported range for the
   chosen integrator it will be quietly adjusted to be in range. You can find
   out the accuracy setting actually being used using `get_accuracy_in_use()`.

   Implicit integrators additionally use the accuracy setting for determining
   when the underlying Newton-Raphson root finding process has converged. For
   those integrators, the accuracy setting also limits the allowable iteration
   error in the Newton-Raphson process. Looser accuracy in that process
   certainly implies greater error in the ODE solution and might impact the
   stability of the solution negatively as well.

   - [Sherman, 2011]  M. Sherman, et al. Procedia IUTAM 2:241-261 (2011),
                      Section 3.3.
                      http://dx.doi.org/10.1016/j.piutam.2011.04.023

   @{
   */
  /**
   Request that the integrator attempt to achieve a particular accuracy for
   the continuous portions of the simulation. Otherwise a default accuracy is
   chosen for you. This may be ignored for fixed-step integration since
   accuracy control requires variable step sizes. You should call
   supports_error_estimation() to ensure that the
   integrator supports this capability before calling this function; if
   the integrator does not support it, this method will throw an exception.

   @throws std::logic_error if integrator does not support error
           estimation.
   */
  // TODO(edrumwri): complain if integrator with error estimation wants to drop
  //                 below the minimum step size
  void set_target_accuracy(double accuracy) {
    if (!supports_error_estimation())
      throw std::logic_error(
          "Integrator does not support accuracy estimation "
          "and user has requested error control");
    target_accuracy_ = accuracy;
    accuracy_in_use_ = accuracy;
  }

  /**
   Gets the target accuracy.
   @sa get_accuracy_in_use()
   */
  double get_target_accuracy() const { return target_accuracy_; }

  /**
   Gets the accuracy in use by the integrator. This number may differ from
   the target accuracy if, for example, the user has requested an accuracy
   not attainable or not recommended for the particular integrator.
   */
  double get_accuracy_in_use() const { return accuracy_in_use_; }
  // @}

  /**
   @anchor error-estimation-and-control
   @name Methods related to error estimation and control
   Established methods for integrating ordinary differential equations
   invariably make provisions for estimating the "local error" (i.e., the
   error over a small time interval) of a solution. Although the relationship
   between local error and global error (i.e., the accumulated error over
   multiple time steps) can be tenuous, such error estimates can allow
   integrators to work adaptively, subdividing time intervals as necessary
   (if, e.g., the system is particularly dynamic or stationary in an interval).
   Even for applications that do not recommend such adaptive integration- like
   direct transcription methods for trajectory optimization- error estimation
   allows the user to assess the accuracy of the solution.
   @{
   */
  /**
   Derived classes must override this function to indicate whether the
   integrator supports error estimation. Without error estimation, the target
   accuracy setting (see @ref integrator-accuracy "accuracy settings") will be
   unused.
   */
  virtual bool supports_error_estimation() const = 0;

  /**
   Derived classes must override this function to return the order of the
   asymptotic term in the integrator's error estimate. An error estimator
   approximates the truncation error in an integrator's solution. That
   truncation error e(.) is approximated by a Taylor Series expansion in the
   neighborhood around t:
   @verbatim
   e(t+h) ≈ e(t) + he(t) + he'(t) + ½h²e''(t) + ...
          ≈ e(t) + he(t) + he'(t) + ½h²e''(t) + O(h³)
   @endverbatim
   where we have replaced the "..." with the asymptotic error of all terms
   truncated from the series.

   Implementions should return the order of the asymptotic term in the Taylor
   Series expansion around the expression for the error. For an integrator
   that propagates a second-order solution and provides an estimate of the
   error using an embedded first-order method, this method should return "2",
   as can be seen in the derivation below, using y* as the true solution:
   @verbatim
   y̅ = y* + O(h³)   [second order solution]
   ŷ = y* + O(h²)   [embedded first-order method]
   e = (y̅ - ŷ) = O(h²)
   @endverbatim

   If the integrator does not provide an error estimate, the derived class
   implementation should return 0.
   */
  virtual int get_error_estimate_order() const = 0;

  /**
   Gets the error estimate (used only for integrators that support error
   estimation). If the integrator does not support error estimation, nullptr
   is returned.
   */
  const ContinuousState<T>* get_error_estimate() const {
    return err_est_.get();
  }

  /**
   Return the step size the integrator would like to take next, based
   primarily on the integrator's accuracy prediction. This value will not
   be computed for integrators that do not support error estimation and
   NaN will be returned.
   */
  const T& get_ideal_next_step_size() const { return ideal_next_step_size_; }

  /**
   Sets an integrator with error control to fixed step mode. If the integrator
   runs in fixed step mode, it will always take the maximum step size
   directed (which may be that determined by get_maximum_step_size(), or may
   be smaller, as directed by, e.g., Simulator for event handling purposes).
   @warning The error estimation process will still be active (so
            get_error_estimate() will still return a correct result), meaning
            that the additional (typically, but not necessarily small)
            computation required for error estimation will still be performed.
   @throws std::logic_error if integrator does not support error
           estimation and @p flag is set to `false`.
   */
  void set_fixed_step_mode(bool flag) {
    if (!flag && !supports_error_estimation())
      throw std::logic_error("Integrator does not support accuracy estimation");
    fixed_step_mode_ = flag;
  }

  /**
   Gets whether an integrator is running in fixed step mode. If the integrator
   does not support error estimation, this function will always return `true`.
   @sa set_fixed_step_mode()
   */
  bool get_fixed_step_mode() const {
    return (!supports_error_estimation() || fixed_step_mode_);
  }
  // @}

  /**
   @name Methods for weighting state variable errors (in the context of error control)
   @anchor weighting-state-errors
   @{
   This group of methods describes how errors for state variables with
   heterogeneous units are weighted in the context of error-controlled
   integration. This is an advanced topic and most users can simply specify
   desired accuracy and accept the default state variable weights.

   A collection of state variables is generally defined in heterogeneous units
   (e.g. length, angles, velocities, energy). Some of the state
   variables cannot even be expressed in meaningful units, like
   quaternions. Certain integrators provide an estimate of the absolute error
   made in each state variable during an integration step. These errors must
   be properly weighted to obtain an "accuracy" _with respect to each
   particular variable_. These per-variable accuracy determinations can be
   compared against the user's requirements and used to select an appropriate
   size for the next step [Sherman 2011]. The weights are
   normally determined automatically using the system's characteristic
   dimensions, so *most users can stop reading now!* Custom weighting is
   primarily useful for performance improvement; an optimal weighting would
   allow an error-controlled integrator to provide the desired level of
   accuracy across all state variables without wasting computation
   achieving superfluous accuracy for some of those variables.

   Users interested in more precise control over state variable weighting may
   use the methods in this group to access and modify weighting factors for
   individual state variables. Changes to these weights can only be made prior
   to integrator initialization or as a result of an event being triggered
   and then followed by re-initialization.

   _Relative versus absolute accuracy_:

   %State variable integration error, as estimated by an integrator, is an
   absolute quantity with the same
   units as the variable. At each time step we therefore need to determine
   an absolute error that would be deemed "good enough", i.e. satisfies
   the user's accuracy requirement. If a variable is maintained to a
   _relative_ accuracy then that "good enough" value is defined to be the
   required accuracy `a` (a fraction like 0.001) times the current value of
   the variable, as long as that value
   is far from zero. For variables maintained to an *absolute* accuracy, or
   relative variables that are at or near zero (where relative accuracy would
   be undefined or too strict, respectively), we need a different way to
   determine the "good enough" absolute error. The methods in this section
   control how that absolute error value is calculated.

   _How to choose weights_:

   The weight `wᵢ` for a state variable `xᵢ` should be
   chosen so that the product `wᵢ * dxᵢ` is unitless, and in particular is 1
   when `dxᵢ` represents a "unit effect" of state variable `xᵢ`; that is, the
   change in `xᵢ` that produces a unit change in some quantity of interest in
   the system being simulated. Why unity (1)? Aside from normalizing the
   values, unity "grounds" the weighted error to the user-specified accuracy.
   A weighting can be applied individually to each state variable, but
   typically it is done approximately by combining the known type of the
   variable (e.g. length, angle) with a "characteristic scale" for that
   quantity. For example, if a "characteristic length" for the system being
   simulated is 0.1 meters, and `x₀` is a length variable measured in meters,
   then `w₀` should be 10 so that `w₀*dx₀=1` when `dx₀=0.1`. For angles
   representing pointing accuracy (say a camera direction) we typically assume
   a "characteristic angle" is one radian (about 60 degrees), so if x₁ is a
   pointing direction then w₁=1 is an appropriate weight. We can now scale an
   error vector `e=[dx₀ dx₁]` to a unitless fractional error vector
   `f=[w₀*dx₀ w₁*dx₁]`. Now to achieve a given accuracy `a`, say `a=.0001`,
   we need only check that `|fᵢ|<=a` for each element `i` of `f`. Further,
   this gives us a quantitative measure of "worst accuracy" that we can use
   to increase or reduce size of the next attempted step, so that we will just
   achieve the required accuracy but not much more. We'll be more precise
   about this below.

   @anchor quasi_coordinates
   _Some subtleties for second-order dynamic systems_:

   Systems governed by 2nd-order differential equations are typically split
   into second order (configuration) variables q, and rate (velocity)
   variables v, where the time derivatives qdot of q are linearly related to
   v by the kinematic differential equation `qdot = dq/dt = N(q)*v`.
   Velocity variables are
   chosen to be physically significant, but configuration variables
   may be chosen for convenience and do not necessarily have direct physical
   interpretation. For examples, quaternions are chosen as a numerically
   stable orientation representation. This is problematic for choosing weights
   which must be done by physical reasoning
   as sketched above. We resolve this by introducing
   the notion of "quasi-coordinates" ꝗ (pronounced "qbar") which are defined
   by the equation `ꝗdot = dꝗ/dt = v`. Other than time scaling,
   quasi-coordinates have the same units as their corresponding velocity
   variables. That is, for weighting we need to think
   of the configuration coordinates in the same physical space as the velocity
   variables; weight those by their physical significance; and then map back
   to an instantaneous weighting
   on the actual configuration variables q. This mapping is performed
   automatically; you need only to be concerned about physical weightings.

   Note that generalized quasi-coordinates `ꝗ` can only be defined locally for
   a particular configuration `q`. There is in general no meaningful set of
   `n` generalized
   coordinates which can be differentiated with respect to time to yield `v`.
   For example, the Hairy Ball Theorem implies that it is not possible for
   three orientation variables to represent all 3D rotations without
   singularities, yet three velocity variables can represent angular velocity
   in 3D without singularities.

   To summarize, separate weights can be provided for each of
   - `n` generalized quasi-coordinates `ꝗ`  (configuration variables in the
     velocity variable space), and
   - `nz` miscellaneous continuous state variables `z`.

   Weights on the generalized velocity variables `v (= dꝗ/dt)` are derived
   directly from the weights on `ꝗ`, weighted by a characteristic time.
   Weights on the actual `nq` generalized coordinates can
   be calculated efficiently from weights on the quasi-coordinates (details
   below).

   _How the weights are used_:

   The errors in the `ꝗ` and `z` variables are weighted by the diagonal
   elements
   of diagonal weighting matrices Wꝗ and Wz, respectively. (The block-diagonal
   weighting matrix `Wq` on the original generalized coordinates `q` is
   calculated from `N` and `Wꝗ`; see below.) In the absence of
   other information, the default for all weighting values is one, so `Wꝗ` and
   `Wz` are `n × n` and `nz × nz` identity matrices. The weighting matrix `Wv`
   for the velocity variables is just `Wv = τ*Wꝗ` where `τ` is a
   "characteristic time" for the system, that is, a quantity in time units
   that represents a significant evolution of the trajectory. This serves to
   control the accuracy with which velocity is determined relative to
   configuration. Note that larger values of `τ` are more conservative since
   they increase the velocity weights. Typically we use `τ=1.0` or `0.1`
   seconds for human-scale mechanical systems.
   <!-- TODO(sherm1): provide more guidance for velocity weighting. -->

   The weighting matrices `Wq`, `Wv`, and `Wz` are used to compute a weighted
   infinity norm as follows. Although `Wv` and `Wz` are constant, the actual
   weightings may be state dependent for relative-error calculations.
   Define block diagonal error weighting matrix `E=diag(Eq,Ev,Ez)` as follows:
   <pre>
     Eq = Wq
     Ev: Ev(i,i) = { min(Wv(i,i), 1/|vᵢ|)     if vᵢ is relative
                   { Wv(i,i)                  if vᵢ is absolute
     Ez: Ez(i,i) = { min(Wz(i,i), 1/|zᵢ|)     if zᵢ is relative
                   { Wz(i,i)                  if zᵢ is absolute
   </pre>
   (`Ev` and `Ez` are diagonal.) A `v` or `z` will be maintained to relative
   accuracy unless (a) it is "close" to zero (less than 1), or (b) the
   variable has been defined as requiring absolute accuracy. Position
   variables `q` are always maintained to absolute accuracy (see
   [Sherman 2011] for rationale).

   Now given an error estimate vector `e=[eq ev ez]`, the vector `f=E*e`
   can be considered to provide a unitless fractional error for each of the
   state variables. To achieve a given user-specified accuracy `a`, we require
   that norm_inf(`f`) <= `a`. That is, no element of `f` can have absolute
   value larger than `a`. We also use `f` to determine an ideal next step
   size using an appropriate integrator-specific computation.

   _Determining weights for q_:

   The kinematic differential equations `qdot=N(q)*v` employ an `nq × n`
   matrix `N`. By construction, this relationship is invertible using `N`'s
   left pseudo-inverse `N⁺` so that `v=N⁺ qdot` and `N⁺ N = I` (the identity
   matrix); however, `N N⁺ != I`, as `N` has more rows than columns generally.
   [Nikravesh 1988] shows how such a matrix `N` can be determined and provides
   more information. Given this relationship between `N` and `N⁺`, we can
   relate weighted errors in configuration coordinates `q` to weighted errors
   in generalized quasi-coordinates `ꝗ`, as the following derivation shows:
   <pre>
              v = N⁺ qdot         Inverse kinematic differential equation
          dꝗ/dt = N⁺ dq/dt        Use synonyms for v and qdot
             dꝗ = N⁺ dq           Change time derivatives to differentials
          Wꝗ dꝗ = Wꝗ N⁺ dq        Pre-multiply both sides by Wꝗ
        N Wꝗ dꝗ = N Wꝗ N⁺ dq      Pre-multiply both sides by N
        N Wꝗ dꝗ = Wq dq           Define Wq := N Wꝗ N⁺
         N Wꝗ v = Wq qdot         Back to time derivatives.
   </pre>
   The last two equations show that `Wq` as defined above provides the
   expected relationship between the weighted `ꝗ` or `v` variables in velocity
   space and the weighted `q` or `qdot` (resp.) variables in configuration
   space.

   Finally, note that a diagonal entry of one of the weighting matrices can
   be set to zero to disable error estimation for that state variable
   (i.e., auxiliary variable or configuration/velocity variable pair), but
   that setting an entry to a negative value will cause an exception to be
   thrown when the integrator is initialized.

   - [Nikravesh 1988] P. Nikravesh. Computer-Aided  Analysis of Mechanical
                      Systems. Prentice Hall, 1988. Sec. 6.3.
   - [Sherman 2011]   M. Sherman, et al. Procedia IUTAM 2:241-261 (2011),
                      Section 3.3.
                      http://dx.doi.org/10.1016/j.piutam.2011.04.023

   @sa CalcStateChangeNorm()
   */
  /**
   Gets the weighting vector (equivalent to a diagonal matrix) applied to
   weighting both generalized coordinate and velocity state variable errors,
   as described in the group documentation. Only used for integrators that
   support error estimation.
   */
  const Eigen::VectorXd& get_generalized_state_weight_vector() const {
    return qbar_weight_;
  }

  /**
   Gets a mutable weighting vector (equivalent to a diagonal matrix) applied
   to weighting both generalized coordinate and velocity state variable
   errors, as described in the group documentation. Only used for
   integrators that support error estimation. Returns a VectorBlock
   to make the values mutable without permitting changing the size of
   the vector. Requires re-initializing the integrator after calling
   this method; if Initialize() is not called afterward, an exception will be
   thrown when attempting to call StepOnceAtMost(). If the caller sets
   one of the entries to a negative value, an exception will be thrown
   when the integrator is initialized.
   */
  Eigen::VectorBlock<Eigen::VectorXd>
  get_mutable_generalized_state_weight_vector() {
    initialization_done_ = false;
    return qbar_weight_.head(qbar_weight_.rows());
  }

  /**
   Gets the weighting vector (equivalent to a diagonal matrix) for
   weighting errors in miscellaneous continuous state variables `z`. Only used
   for integrators that support error estimation.
   */
  const Eigen::VectorXd& get_misc_state_weight_vector() const {
    return z_weight_;
  }

  /**
   Gets a mutable weighting vector (equivalent to a diagonal matrix) for
   weighting errors in miscellaneous continuous state variables `z`. Only used
   for integrators that support error estimation. Returns a VectorBlock
   to make the values mutable without permitting changing the size of
   the vector. Requires re-initializing the integrator after calling this
   method. If Initialize() is not called afterward, an exception will be
   thrown when attempting to call StepOnceAtMost(). If the caller sets
   one of the entries to a negative value, an exception will be thrown
   when the integrator is initialized.
   */
  Eigen::VectorBlock<Eigen::VectorXd> get_mutable_misc_state_weight_vector() {
    initialization_done_ = false;
    return z_weight_.head(z_weight_.rows());
  }
  // @}

  /**
   @anchor integrator-initial-step-size
   @name Methods related to initial step size
   From [Watts 1983], "One of the more critical issues in solving ordinary
   differential equations by a step-by-step process occurs in the starting
   phase. Somehow the procedure must be supplied with an initial step size that
   is on scale for the problem at hand. It must be small enough to yield a
   reliable solution by the process, but it should not be so small as to
   significantly affect the efficiency of solution. The more important of these
   two possibilities is obviously the reliability question. The first step taken
   by the code must reflect how fast the solution changes near the initial
   point. For general purpose computing, an automatic step size adjustment
   procedure for choosing subsequent steps is essential to produce an accurate
   solution efficiently. This step size control is usually based on estimates of
   the local errors incurred by the numerical method. Because most codes also
   employ algorithmic devices which restrict the step size control to be
   moderately varying (for reliability), subsequent steps usually tend to stay
   on scale of the problem. This is not always so, as sometimes happens when
   working with crude tolerances on problems having rapidly varying components.
   Nevertheless, most step size adjustment procedures deal reasonably well with
   all but the most abrupt changes, leaving the most serious danger confined to
   the starting step size."

   Users may not have a good idea of an initial step size to take, so
   integration codes usually attempt to automatically select an initial step
   size. Sophisticated algorithms for initial step size selection are described
   in [Hindmarsh 1980], [Watts 1983], [Gladwell 1987], and [Hairer 2008]. These
   algorithms can fail to produce a good initial step size as well (see
   discussion in [Watts 1983]). Drake's integrators use a fraction (generally
   1/10th of the maximum step size) to set the initial step size. If you have a
   problem that operates at wildly varying time scales, e.g., Robertson's
   canonical stiff system problem (that requires a large maximum step size to
   be efficient), consider setting both the initial and maximum step sizes
   (i.e., not using the defaults) to keep from missing phenomena that occur over
   small time scales near the beginning of the time interval being integrated.

   - [Gladwell 1987]  I. Gladwell, L. F. Shampine, and R. W. Brankin. Automatic
                      selection of the initial step size for an ODE solver. J.
                      Comp. Appl. Math., Vol. 18, pp. 175-192, 1987.
   - [Hairer 2008]    E. Hairer, S. P. Norsett, and G. Wanner. Solving Ordinary
                      Differential Equations I (Nonstiff Problems), 2nd ed.
                      Springer, 2008.
   - [Hindmarsh 1980] A. C. Hindmarsh. LSODE and LSODI, two new initial value
                      ordinary differential equation solvers. ACM Signum
                      Newletter 15, 4, 1980.
   - [Robertson 1966] H.H. Robertson. The solution of a set of reaction rate
                      equations, pp. 178–182. Academic Press, 1966.
   - [Watts 1983]     H. A. Watts. Starting stepsize for an ODE solver. J. Comp.
                      Appl. Math., Vol. 8, pp. 177-191, 1983.
   @{
   */

  /**
   Request that the first attempted integration step have a particular size.
   If no request is made, the integrator will estimate a suitable size
   for the initial step attempt. *If the integrator does not support error
   control*, this method will throw a std::logic_error (call
   supports_error_estimation() to verify before calling this method). For
   variable-step integration, the initial target will be treated as a maximum
   step size subject to accuracy requirements and event occurrences. You can
   find out what size *actually* worked with
   `get_actual_initial_step_size_taken()`.
   @throws std::logic_error If the integrator does not support error
       estimation.
   */
  void request_initial_step_size_target(const T& step_size) {
    using std::isnan;
    if (!supports_error_estimation())
      throw std::logic_error(
          "Integrator does not support error estimation and "
          "user has initial step size target");
    req_initial_step_size_ = step_size;
  }

  /**
   Gets the target size of the first integration step. You can find out what
   step size was *actually* used for the first integration step with
   `get_actual_initial_step_size_taken()`.
   @see request_initial_step_size_target()
   */
  const T& get_initial_step_size_target() const {
    return req_initial_step_size_;
  }
  // @}

  /**
   @anchor integrator-maxstep
   @name Methods related to maximum integration step size

   Sets the _nominal_ maximum step size- the actual maximum step size taken
   may be slightly larger (see set_maximum_step_size() and
   get_stretch_factor())- that an integrator will take. Each integrator has a
   default maximum step size, which might be infinite.
   @{
   */
  /**
   Sets the maximum step size that may be taken by this integrator. This setting
   should be used if you know the maximum time scale of your problem. The
   integrator may stretch the maximum step size by as much as 1% to reach a
   discrete event. For fixed step integrators, all steps will be taken at the
   maximum step size *unless* an event would be missed.
   @warning See @ref integrator-initial-step-size "Initial step size selection"
   */
  // TODO(edrumwri): Update this comment when stretch size is configurable.
  void set_maximum_step_size(const T& max_step_size) {
    DRAKE_ASSERT(max_step_size >= 0.0);
    max_step_size_ = max_step_size;
  }

  /**
   Gets the maximum step size that may be taken by this integrator. This is
   a soft maximum: the integrator may stretch it by as much as 1% to hit a
   discrete event.
   @sa set_requested_minimum_step_size()
   */
  // TODO(edrumwri): Update this comment when stretch size is configurable.
  const T& get_maximum_step_size() const { return max_step_size_; }

  /**
   Gets the stretch factor (> 1), which is multiplied by the maximum
   (typically user-designated) integration step size to obtain the amount
   that the integrator is able to stretch the maximum time step toward
   hitting an upcoming publish or update event in
   IntegrateNoFurtherThanTime().
   @sa IntegrateNoFurtherThanTime()
   */
  double get_stretch_factor() const { return 1.01; }
  // @}

  /**
   @anchor integrator-minstep
   @name Methods related to minimum integration step size selection and behavior

   Variable step integrators reduce their step sizes as needed to achieve
   requirements such as specified accuracy or step convergence. However, it is
   not possible to take an arbitrarily small step. Normally integrators choose
   an appropriate minimum step and throw an exception if the requirements can't
   be achieved without going below that. Methods in this section allow you to
   influence two aspects of this procedure:

   - you can increase the minimum step size, and
   - you can control whether an exception is thrown if a smaller step would have
     been needed to achieve the aforementioned integrator requirements.

   By default, integrators allow a very small minimum step which can result in
   long run times. Setting a larger minimum can be helpful as a diagnostic to
   figure out what aspect of your simulation is requiring small steps. You can
   set the minimum to what should be a "reasonable" minimum based on what you
   know about the physical system. You will then get an std::runtime_error
   exception thrown at any point in time where your model behaves unexpectedly
   (due to, e.g., a discontinuity in the derivative evaluation function).

   If you disable the exception (via
   `set_throw_on_minimum_step_size_violation(false)`), the integrator will
   simply proceed with a step of the minimum size: accuracy is guaranteed only
   when the minimum step size is not violated. Beware that there can be no
   guarantee about the magnitude of any errors introduced by violating the
   accuracy "requirements" in this manner, so disabling the exception should be
   done warily.

   #### Details
   Because time is maintained to finite precision, the integrator uses a scalar
   `h_floor` to constrain time step h ≥ `h_floor` such that `current_time + h >
   current_time` will be strictly satisfied. The integrator will never
   automatically decrease its step below `h_floor`. We calculate `h_floor=max(ε,
   ε⋅abs(t))`, where t is the current time and ε is a small multiple of machine
   precision, typically a number like 1e-14. Note that `h_floor` necessarily
   grows with time; if that is a concern you should limit how long your
   simulations are allowed to run without resetting time.

   You may request a larger minimum step size `h_min`. Then at every time t, the
   integrator determines a "working" minimum `h_work=max(h_min, h_floor)`. If
   the step size selection algorithm determines that a step smaller than
   `h_work` is needed to meet accuracy or other needs, then a std::runtime_error
   exception will be thrown and the simulation halted. On the other hand, if you
   have suppressed the exception (again, via
   `set_throw_on_minimum_step_size_violation(false)`), the integration will
   continue, taking a step of size `h_work`.

   Under some circumstances the integrator may legitimately take a step of size
   `h` smaller than your specified `h_min`, although never smaller than
   `h_floor`. For example, occasionally the integrator may reach an event or
   time limit that occurs a very short time after the end of a previous step,
   necessitating that a tiny "sliver" of a step be taken to complete the
   interval. That does not indicate an error, and required accuracy and
   convergence goals are achieved. Larger steps can resume immediately
   afterwards. Another circumstance is when one of the integrator's stepping
   methods is called directly requesting a very small step, for example
   `IntegrateWithMultipleStepsToTime(h)`. No exception will be thrown in either
   of these cases.
   */

  //@{
  /**
   Sets the requested minimum step size `h_min` that may be taken by this
   integrator. No step smaller than this will be taken except under
   circumstances as described @ref integrator-minstep "above". This setting will
   be ignored if it is smaller than the absolute minimum `h_floor` also
   described above. Default value is zero.
   @param min_step_size a non-negative value. Setting this value to zero
                        will cause the integrator to use a reasonable value
                        instead (see get_working_minimum_step_size()).
   @sa get_requested_minimum_step_size()
   @sa get_working_minimum_step_size()
   */
  void set_requested_minimum_step_size(const T& min_step_size) {
    DRAKE_ASSERT(min_step_size >= 0.0);
    req_min_step_size_ = min_step_size;
  }

  /**
   Gets the requested minimum step size `h_min` for this integrator.
   @sa set_requested_minimum_step_size()
   @sa get_working_minimum_step_size(T)
   */
  const T& get_requested_minimum_step_size() const {
    return req_min_step_size_; }

  /**
   Sets whether the integrator should throw a std::runtime_error exception
   when the integrator's step size selection algorithm determines that it
   must take a step smaller than the minimum step size (for, e.g., purposes
   of error control). Default is `true`. If `false`, the integrator will
   advance time and state using the minimum specified step size in such
   situations. See @ref integrator-minstep "this section" for more detail.
   */
  void set_throw_on_minimum_step_size_violation(bool throws) {
    min_step_exceeded_throws_ = throws;
  }

  /**
   Reports the current setting of the throw_on_minimum_step_size_violation
   flag.
   @sa set_throw_on_minimum_step_size_violation().
   */
  bool get_throw_on_minimum_step_size_violation() const {
    return min_step_exceeded_throws_;
  }

  /**
   Gets the current value of the working minimum step size `h_work(t)` for
   this integrator, which may vary with the current time t as stored in the
   integrator's context.
   See @ref integrator-minstep "this section" for more detail.
   */
  T get_working_minimum_step_size() const {
    using std::max;
    using std::abs;
    // Tolerance is just a number close to machine epsilon.
    const double tol = 1e-14;

    const T smart_minimum = max(tol, abs(get_context().get_time()) * tol);
    return max(smart_minimum, req_min_step_size_);
  }
  // @}

  /**
   Resets the integrator to initial values, i.e., default construction
   values.
   */
  void Reset() {
    // Kill the error estimate and weighting matrices.
    err_est_.reset();
    qbar_weight_.setZero(0);
    z_weight_.setZero(0);
    pinvN_dq_change_.reset();
    unweighted_substate_change_.setZero(0);
    weighted_q_change_.reset();

    // Drops dense output, if any.
    dense_output_.reset();

    // Integrator no longer operates in fixed step mode.
    fixed_step_mode_ = false;

    // Statistics no longer valid.
    ResetStatistics();

    // Wipe out settings.
    req_min_step_size_ = 0;
    max_step_size_ = nan();
    accuracy_in_use_ = nan();

    // Indicate values used for error controlled integration no longer valid.
    prev_step_size_ = nan();
    ideal_next_step_size_ = nan();

    // Call the derived integrator reset routine.
    DoReset();

    // Indicate that initialization is necessary.
    initialization_done_ = false;
  }

  /**
   An integrator must be initialized before being used. The pointer to the
   context must be set before Initialize() is called (or an std::logic_error
   will be thrown). If Initialize() is not called, an exception will be
   thrown when attempting to call StepOnceAtMost(). To reinitialize the
   integrator, Reset() should be called followed by Initialize().
   @throws std::logic_error If the context has not been set or a user-set
           parameter has been set illogically (i.e., one of the
           weighting matrix coefficients is set to a negative value- this
           check is only performed for integrators that support error
           estimation; the maximum step size is smaller than the minimum
           step size; the requested initial step size is outside of the
           interval [minimum step size, maximum step size]).
   @sa Reset()
   */
  void Initialize() {
    if (!context_) throw std::logic_error("Context has not been set.");

    // Verify that user settings are reasonable.
    if constexpr (scalar_predicate<T>::is_bool) {
      if (max_step_size_ < req_min_step_size_) {
        throw std::logic_error("Integrator maximum step size is less than the "
                               "minimum step size");
      }
      if (req_initial_step_size_ > max_step_size_) {
        throw std::logic_error("Requested integrator initial step size is "
                               "larger than the maximum step size.");
      }
      if (req_initial_step_size_ < req_min_step_size_) {
        throw std::logic_error("Requested integrator initial step size is "
                               "smaller than the minimum step size.");
      }
    }

    // TODO(edrumwri): Compute qbar_weight_, z_weight_ automatically.
    // Set error weighting vectors if not already done.
    if (supports_error_estimation()) {
      // Allocate space for the error estimate.
      err_est_ = system_.AllocateTimeDerivatives();

      const auto& xc = context_->get_state().get_continuous_state();
      const int gv_size = xc.get_generalized_velocity().size();
      const int misc_size = xc.get_misc_continuous_state().size();
      if (qbar_weight_.size() != gv_size) qbar_weight_.setOnes(gv_size);
      if (z_weight_.size() != misc_size) z_weight_.setOnes(misc_size);

      // Verify that minimum values of the weighting matrices are non-negative.
      if ((qbar_weight_.size() && qbar_weight_.minCoeff() < 0) ||
          (z_weight_.size() && z_weight_.minCoeff() < 0))
        throw std::logic_error("Scaling coefficient is less than zero.");
    }

    // Statistics no longer valid.
    ResetStatistics();

    // Call the derived integrator initialization routine (if any)
    DoInitialize();

    initialization_done_ = true;
  }

  /**
   (Internal use only) Integrates the system forward in time by a single step
   with step size subject to integration error tolerances (assuming that the
   integrator supports error estimation). The integrator must already have
   been initialized or an exception will be thrown. The context will be
   integrated to a time that will never exceed the minimum of
   `publish_time`, `update_time`, and the current time plus
   `1.01 * get_maximum_step_size()`.
   @param publish_time The present or future time (exception will be thrown
          if this is not the case) at which the next publish will occur.
   @param update_time The present or future time (exception will be thrown
          if this is not the case) at which the next update will occur.
   @param boundary_time The present or future time (exception will be thrown
          if this is not the case) marking the end of the user-designated
          simulated interval.
   @throws std::logic_error If the integrator has not been initialized or one
                            of publish_time, update_time, or boundary_time is
                            in the past.
   @return The reason for the integration step ending.
   @post The time in the context will be no greater than
         `min(publish_time, update_time, boundary_time)`.
   @warning Users should generally not call this function directly; within
            simulation circumstances, users will typically call
            `Simulator::AdvanceTo()`. In other circumstances, users will
            typically call
            `IntegratorBase::IntegrateWithMultipleStepsToTime()`.

   This method at a glance:
   - For integrating ODEs/DAEs via Simulator
   - Supports fixed step and variable step integration schemes
   - Takes only a single step forward.
   */
  // TODO(edrumwri): Make the stretch size configurable.
  StepResult IntegrateNoFurtherThanTime(
    const T& publish_time, const T& update_time, const T& boundary_time);

  /**
   Stepping function for integrators operating outside of Simulator that
   advances the continuous state exactly to `t_final`. This method is
   designed for integrator users that do not wish to consider publishing or
   discontinuous, mid-interval updates. This method will step the integrator
   multiple times, as necessary, to attain requested error tolerances and
   to ensure the integrator converges.
   @warning Users should simulate systems using `Simulator::AdvanceTo()` in
            place of this function (which was created for off-simulation
            purposes), generally.
   @param t_final The current or future time to integrate to.
   @throws std::logic_error If the integrator has not been initialized or
                            t_final is in the past.
   @sa IntegrateNoFurtherThanTime(), which is designed to be operated by
       Simulator and accounts for publishing and state reinitialization.
   @sa IntegrateWithSingleStepToTime(), which is also designed to be operated
       *outside of* Simulator, but throws an exception if the integrator
       cannot advance time to `t_final` in a single step.

   This method at a glance:
   - For integrating ODEs/DAEs not using Simulator
   - Supports fixed step and variable step integration schemes
   - Takes as many steps as necessary until time has advanced to `t_final`
   */
  void IntegrateWithMultipleStepsToTime(const T& t_final) {
    using std::max;
    using std::min;

    const Context<T>& context = get_context();
    const T inf = std::numeric_limits<double>::infinity();

    do {
      IntegrateNoFurtherThanTime(inf, inf,
          min(t_final, context.get_time() + get_maximum_step_size()));
    } while (context.get_time() < t_final);
  }

  /**
   Stepping function for integrators operating outside of Simulator that
   advances the continuous state *using a single step* to `t_target`.
   This method is designed for integrator users that do not wish to
   consider publishing or discontinuous, mid-interval updates. One such
   example application is that of direct transcription for trajectory
   optimization, for which the integration process should be _consistent_: it
   should execute the same sequence of arithmetic operations for all values
   of the nonlinear programming variables. In keeping with the naming
   semantics of this function, error controlled integration is not supported
   (though error estimates will be computed for integrators that support that
   feature), which is a minimal requirement for "consistency".
   @warning Users should simulate systems using `Simulator::AdvanceTo()` in
            place of this function (which was created for off-simulation
            purposes), generally.
   @param t_target The current or future time to integrate to.
   @throws std::logic_error If the integrator has not been initialized or
                            `t_target` is in the past or the integrator
                            is not operating in fixed step mode.
   @sa IntegrateNoFurtherThanTime(), which is designed to be operated by
       Simulator and accounts for publishing and state reinitialization.
   @sa IntegrateWithMultipleStepsToTime(), which is also designed to be
       operated *outside of* Simulator, but will take as many integration
       steps as necessary until time has been stepped forward to `t_target`.
   @returns `true` if the integrator was able to take a single fixed step to
            `t_target`.

   This method at a glance:

   - For integrating ODEs/DAEs not using Simulator
   - Fixed step integration (no step size reductions for error control or
     integrator convergence)
   - Takes only a single step forward.
   */
  [[nodiscard]] bool IntegrateWithSingleFixedStepToTime(const T& t_target) {
    using std::max;
    using std::abs;

    const T h = t_target - context_->get_time();
    if (scalar_predicate<T>::is_bool && h < 0) {
      throw std::logic_error("IntegrateWithSingleFixedStepToTime() called with "
                             "a negative step size.");
    }
    if (!this->get_fixed_step_mode())
      throw std::logic_error("IntegrateWithSingleFixedStepToTime() requires "
                             "fixed stepping.");

    if (!Step(h))
      return false;

    UpdateStepStatistics(h);

    if constexpr (scalar_predicate<T>::is_bool) {
      // Correct any round-off error that has occurred. Formula below requires
      // that time be non-negative.
      DRAKE_DEMAND(context_->get_time() >= 0);
      const double tol = 10 * std::numeric_limits<double>::epsilon() *
          ExtractDoubleOrThrow(max(1.0, max(t_target, context_->get_time())));
      DRAKE_DEMAND(abs(context_->get_time() - t_target) < tol);
    }

    context_->SetTime(t_target);

    return true;
  }

  /**
   @name Integrator statistics methods
   @{
   These methods allow the caller to manipulate and query integrator
   statistics. Generally speaking, the larger the integration step taken, the
   faster a simulation will run. These methods allow querying (and resetting)
   the integrator statistics as one means of determining how to make
   a simulation run faster.
   */
  /**
   Forget accumulated statistics. These are reset to the values they have
   post construction or immediately after `Initialize()`.
   */
  void ResetStatistics() {
    actual_initial_step_size_taken_ = nan();
    smallest_adapted_step_size_taken_ = nan();
    largest_step_size_taken_ = nan();
    num_steps_taken_ = 0;
    num_ode_evals_ = 0;
    num_shrinkages_from_error_control_ = 0;
    num_shrinkages_from_substep_failures_ = 0;
    num_substep_failures_ = 0;
    DoResetStatistics();
  }

  /**
   Gets the number of failed sub-steps (implying one or more step size
   reductions was required to permit solving the necessary nonlinear system
   of equations).
   */
  int64_t get_num_substep_failures() const {
    return num_substep_failures_;
  }

  /**
   Gets the number of step size shrinkages due to sub-step failures (e.g.,
   integrator convergence failures) since the last call to ResetStatistics()
   or Initialize().
   */
  int64_t get_num_step_shrinkages_from_substep_failures() const {
    return num_shrinkages_from_substep_failures_;
  }

  /// Gets the number of step size shrinkages due to failure to meet targeted
  /// error tolerances, since the last call to ResetStatistics or Initialize().
  int64_t get_num_step_shrinkages_from_error_control() const {
    return num_shrinkages_from_error_control_;
  }

  /**
   Returns the number of ODE function evaluations (calls to
   CalcTimeDerivatives()) since the last call to ResetStatistics() or
   Initialize(). This count includes *all* such calls including (1)
   those necessary to compute Jacobian matrices; (2) those used in rejected
   integrated steps (for, e.g., purposes of error control); (3) those used
   strictly for integrator error estimation; and (4) calls that exhibit little
   cost (due to results being cached).
   */
  int64_t get_num_derivative_evaluations() const { return num_ode_evals_; }

  /**
   The actual size of the successful first step.
   */
  const T& get_actual_initial_step_size_taken() const {
    return actual_initial_step_size_taken_;
  }

  /**
   The size of the smallest step taken *as the result of a controlled
   integration step adjustment* since the last Initialize() or
   ResetStatistics() call. This value will be NaN for integrators without
   error estimation.
   */
  const T& get_smallest_adapted_step_size_taken() const {
    return smallest_adapted_step_size_taken_;
  }

  /**
   The size of the largest step taken since the last Initialize() or
   ResetStatistics() call.
   */
  const T& get_largest_step_size_taken() const {
    return largest_step_size_taken_;
  }

  /**
   The number of integration steps taken since the last Initialize()
   or ResetStatistics() call.
   */
  int64_t get_num_steps_taken() const { return num_steps_taken_; }

  /** Manually increments the statistic for the number of ODE evaluations.
   @warning Implementations should generally avoid calling this method;
            evaluating the ODEs using EvalTimeDerivatives() updates this
            statistic automatically and intelligently (by leveraging the
            caching system to avoid incrementing the count when cached
            evaluations are used).
   */
  void add_derivative_evaluations(double evals) { num_ode_evals_ += evals; }
  // @}

  /**
   Returns a const reference to the internally-maintained Context holding
   the most recent state in the trajectory. This is suitable for publishing or
   extracting information about this trajectory step.
   */
  const Context<T>& get_context() const { return *context_; }

  /**
   Returns a mutable pointer to the internally-maintained Context holding
   the most recent state in the trajectory.
   */
  Context<T>* get_mutable_context() { return context_; }

  /**
   Replace the pointer to the internally-maintained Context with a different
   one. This is useful for supplying a new set of initial conditions or
   wiping out the current context (by passing in a null pointer). You
   should invoke Initialize() after replacing the Context unless the
   context is null.
   @param context The pointer to the new context or nullptr to wipe out
                  the current context without replacing it with another.
   */
  void reset_context(Context<T>* context) {
    context_ = context;
    initialization_done_ = false;
  }


  /**
   @name               Methods for dense output computation
   @anchor dense_output_computation
   @{

   In general, dense output computations entail both CPU load and memory
   footprint increases during numerical integration. For some applications,
   the performance penalty may be prohibitive. As such, these computations
   are only carried out by explicit user request. The API to start and stop
   a _dense integration_ process (i.e. a numerical integration process that
   also computes dense output) is consistent with this design choice.

   Once dense integration is started, and until it is stopped, all subsequent
   integration steps taken will update the allocated dense output.
   */

  /**
   Starts dense integration, allocating a new dense output for this integrator
   to use.

   @pre The integrator has been initialized.
   @pre The system being integrated has continuous state.
   @pre No dense integration is in progress (no dense output is held by the
        integrator)
   @throws std::logic_error if any of the preconditions is not met.
   @warning Dense integration may incur significant overhead.
   */
  void StartDenseIntegration() {
    if (!is_initialized()) {
      throw std::logic_error("Integrator was not initialized.");
    }
    if (get_context().num_continuous_states() == 0) {
      throw std::logic_error("System has no continuous state,"
                             " no dense output can be built.");
    }
    if (get_dense_output()) {
      throw std::logic_error("Dense integration has been started already.");
    }
    dense_output_ = std::make_unique<trajectories::PiecewisePolynomial<T>>();
  }

  /**
   Returns a const pointer to the integrator's current PiecewisePolynomial
   instance, holding a representation of the continuous state trajectory since
   the last StartDenseIntegration() call. This is suitable to query the
   integrator's current dense output, if any (may be nullptr).
   */
  const trajectories::PiecewisePolynomial<T>* get_dense_output() const {
    return dense_output_.get();
  }

  /**
   Stops dense integration, yielding ownership of the current dense output
   to the caller.

   @remarks This process is irreversible.
   @returns A PiecewisePolynomial instance, i.e. a representation of the
            continuous state trajectory of the system being integrated
            that can be evaluated at any time within its extension. This
            representation is defined starting at the context time of the
            last StartDenseIntegration() call and finishing at the current
            context time.
   @pre Dense integration is in progress (a dense output is held by this
        integrator, after a call to StartDenseIntegration()).
   @post Previously held dense output is not updated nor referenced by
         the integrator anymore.
   @throws std::logic_error if any of the preconditions is not met.
   */
  std::unique_ptr<trajectories::PiecewisePolynomial<T>> StopDenseIntegration() {
    if (!dense_output_) {
      throw std::logic_error("No dense integration has been started.");
    }
    return std::move(dense_output_);
  }
  // @}

  /**
   Gets a constant reference to the system that is being integrated (and
   was provided to the constructor of the integrator).
   */
  const System<T>& get_system() const { return system_; }

  /// Indicates whether the integrator has been initialized.
  bool is_initialized() const { return initialization_done_; }

  /**
   Gets the size of the last (previous) integration step. If no integration
   steps have been taken, value will be NaN.
   */
  const T& get_previous_integration_step_size() const {
    return prev_step_size_;
  }

 protected:
  /**
   Resets any statistics particular to a specific integrator. The default
   implementation of this function does nothing. If your integrator
   collects its own statistics, you should re-implement this method and
   reset them there.
   */
  virtual void DoResetStatistics() {}

  /**
   Evaluates the derivative function and updates call statistics.
   Subclasses should call this function rather than calling
   system.EvalTimeDerivatives() directly.
   */
  const ContinuousState<T>& EvalTimeDerivatives(const Context<T>& context) {
    return EvalTimeDerivatives(get_system(), context);  // See below.
  }

  /**
   Evaluates the derivative function (and updates call statistics).
   Subclasses should call this function rather than calling
   system.EvalTimeDerivatives() directly. This version of this function
   exists to allow integrators to include AutoDiff'd systems in derivative
   function evaluations.
   */
  template <typename U>
  const ContinuousState<U>& EvalTimeDerivatives(const System<U>& system,
                                                const Context<U>& context) {
    const CacheEntry& entry = system.get_time_derivatives_cache_entry();
    const CacheEntryValue& value = entry.get_cache_entry_value(context);
    const int64_t serial_number_before = value.serial_number();
    const ContinuousState<U>& derivs =
        system.EvalTimeDerivatives(context);
    if (value.serial_number() != serial_number_before) {
      ++num_ode_evals_;  // Wasn't already cached.
    }
    return derivs;
  }

  /**
   Sets the working ("in use") accuracy for this integrator. The working
   accuracy may not be equivalent to the target accuracy when the latter is
   too loose or tight for an integrator's capabilities.
   @sa get_accuracy_in_use()
   @sa get_target_accuracy()
   */
  void set_accuracy_in_use(double accuracy) { accuracy_in_use_ = accuracy; }

  /**
   Generic code for validating (and resetting, if need be) the integrator
   working accuracy for error controlled integrators. This method is
   intended to be called from an integrator's DoInitialize() method.
   @param default_accuracy a reasonable default accuracy setting for this
          integrator.
   @param loosest_accuracy the loosest accuracy that this integrator should
          support.
   @param max_step_fraction a fraction of the maximum step size to use when
          setting the integrator accuracy and the user has not specified
          accuracy directly.
   @throws std::logic_error if neither the initial step size target nor
           the maximum step size has been set.
   */
  void InitializeAccuracy(double default_accuracy, double loosest_accuracy,
                          double max_step_fraction) {
    using std::isnan;

    // Set an artificial step size target, if not set already.
    if (isnan(this->get_initial_step_size_target())) {
      // Verify that maximum step size has been set.
      if (isnan(this->get_maximum_step_size()))
        throw std::logic_error("Neither initial step size target nor maximum "
                                   "step size has been set!");

      this->request_initial_step_size_target(
          this->get_maximum_step_size() * max_step_fraction);
    }

    // Sets the working accuracy to a good value.
    double working_accuracy = this->get_target_accuracy();

    // If the user asks for accuracy that is looser than the loosest this
    // integrator can provide, use the integrator's loosest accuracy setting
    // instead.
    if (isnan(working_accuracy)) {
      working_accuracy = default_accuracy;
    } else {
      if (working_accuracy > loosest_accuracy)
        working_accuracy = loosest_accuracy;
    }
    this->set_accuracy_in_use(working_accuracy);
  }

  /**
   Default code for advancing the continuous state of the system by a single
   step of @p h_max (or smaller, depending on error control). This particular
   function is designed to be called directly by an error estimating
   integrator's DoStep() method to effect error-controlled integration.
   The integrator can effect error controlled integration without calling this
   method, if the implementer so chooses, but this default method is expected
   to function well in most circumstances.
   @param[in] h_max The maximum step size to be taken. The integrator may
                 take a smaller step than specified to satisfy accuracy
                 requirements, to resolve integrator convergence problems, or
                 to respect the integrator's maximum step size.
   @throws std::logic_error if integrator does not support error
                            estimation.
   @note This function will shrink the integration step as necessary whenever
         the integrator's DoStep() fails to take the requested step
         e.g., due to integrator convergence failure.
   @returns `true` if the full step of size @p h_max is taken and `false`
            otherwise (i.e., a smaller step than @p h_max was taken).
   */
  bool StepOnceErrorControlledAtMost(const T& h_max);

  /**
   Computes the infinity norm of a change in continuous state. We use the
   infinity norm to capture the idea that, by providing accuracy requirements,
   the user can indirectly specify error tolerances that act to limit the
   largest error in any state vector component.
   @returns the norm (a non-negative value)
   */
  T CalcStateChangeNorm(const ContinuousState<T>& dx_state) const;

  /**
   Calculates adjusted integrator step sizes toward keeping state variables
   within error bounds on the next integration step. Note that it is not
   guaranteed that the (possibly) reduced step size will keep state variables
   within error bounds; however, the process of (1) taking a trial
   integration step, (2) calculating the error, and (3) adjusting the step
   size can be repeated until convergence.
   @param err
        The norm of the integrator error that was computed using
         @p attempted_step_size.
   @param attempted_step_size
        The step size that was attempted.
   @param[in,out] at_minimum_step_size
         If `true` on entry, the error control mechanism is not allowed to
         shrink the step because the integrator is stepping at the minimum
         step size (note that this condition will only occur if
         `get_throw_on_minimum_step_size_violation() == false`- an exception
         would be thrown otherwise). If `true` on entry and `false` on exit,
         the error control mechanism has managed to increase the step size
         above the working minimum; if `true` on entry and `true` on exit,
         error control would like to shrink the step size but cannot. If
         `false` on entry and `true` on exit, error control shrank the step
         to the working minimum step size.
   @returns a pair of types bool and T; the bool will be set to `true` if
        the integration step was to be considered successful and `false`
        otherwise. The value of the T type will be set to the recommended next
        step size.
   */
  std::pair<bool, T> CalcAdjustedStepSize(
      const T& err,
      const T& attempted_step_size,
      bool* at_minimum_step_size) const;

  /**
   Derived classes can override this method to perform special
   initialization. This method is called during the Initialize() method. This
   default method does nothing.
   */
  virtual void DoInitialize() {}

  /**
   Derived classes can override this method to perform routines when
   Reset() is called. This default method does nothing.
   */
  virtual void DoReset() {}

  /**
   Returns a mutable pointer to the internally-maintained PiecewisePolynomial
   instance, holding a representation of the continuous state trajectory since
   the last time StartDenseIntegration() was called. This is useful for
   derived classes to update the integrator's current dense output, if any
   (may be nullptr).
   */
  trajectories::PiecewisePolynomial<T>* get_mutable_dense_output() {
    return dense_output_.get();
  }

  /**
   Derived classes must implement this method to (1) integrate the continuous
   portion of this system forward by a single step of size @p h and
   (2) set the error estimate (via get_mutable_error_estimate()). This
   method is called during the integration process (via
   StepOnceErrorControlledAtMost(), IntegrateNoFurtherThanTime(), and
   IntegrateWithSingleFixedStepToTime()).
   @param h The integration step to take.
   @returns `true` if successful, `false` if the integrator was unable to take
             a single step of size @p h (due to, e.g., an integrator
             convergence failure).
   @post If the time on entry is denoted `t`, the time and state will be
         advanced to `t+h` if the method returns `true`; otherwise, the
         time and state should be reset to those at `t`.
   @warning It is expected that DoStep() will return `true` for some, albeit
            possibly very small, positive value of @p h. The derived
            integrator's stepping algorithm can make this guarantee, for
            example, by switching to an algorithm not subject to convergence
            failures (e.g., explicit Euler) for very small step sizes.
   */
  virtual bool DoStep(const T& h) = 0;

  // TODO(russt): Allow subclasses to override the interpolation scheme used, as
  // the 'optimal' dense output scheme is only known by the specific integration
  // scheme being implemented.
  /**
   Calls DoStep(h) while recording the resulting step in the dense output.  If
   the current dense output is already non-empty, then the time in the current
   context must match either the final segment time of the dense output, or the
   penultimate segment time (to support the case where the same integration step
   is attempted multiple times, which occurs e.g. in witness function
   isolation).
   @param h The integration step to take.
   @returns `true` if successful, `false` if either the integrator was unable to
           take a single step of size @p h or to advance its dense output an
           equal step.
   @sa DoStep()
   */
  bool DoDenseStep(const T& h) {
    const ContinuousState<T>& state = context_->get_continuous_state();

    // Note: It is tempting to avoid this initial call to EvalTimeDerivatives,
    // and just use AppendCubicHermiteSegment below.  But this version is robust
    // to e.g. UnrestrictedUpdates or any other changes that could occur between
    // calls to DoDenseStep().  And we hope that the caching in
    // EvalTimeDerivatives() avoids any cost for the easy case.
    const T start_time = context_->get_time();
    VectorX<T> start_state, start_derivatives;
    start_state = state.CopyToVector();
    start_derivatives = EvalTimeDerivatives(*context_).CopyToVector();

    // Performs the integration step.
    if (!DoStep(h)) return false;

    // Allow this update to *replace* the final segment if the start_time of
    // this step is earlier than the current end_time of the dense output and
    // matches the start_time of the the final segment of the dense output.
    // This happens, for instance, when the Simulator is doing WitnessFunction
    // isolation; it routinely back up the integration and try the same step
    // multiple times.  Note: we intentionally check for equality between
    // double values here.
    if (dense_output_->get_segment_times().size() > 1 &&
        start_time < dense_output_->end_time() &&
        start_time == dense_output_->get_segment_times().end()[-2]) {
      dense_output_->RemoveFinalSegment();
    }

    const ContinuousState<T>& derivatives = EvalTimeDerivatives(*context_);
    dense_output_->ConcatenateInTime(
        trajectories::PiecewisePolynomial<T>::CubicHermite(
            std::vector<T>({start_time, context_->get_time()}),
            {start_state, state.CopyToVector()},
            {start_derivatives, derivatives.CopyToVector()}));
    return true;
  }

  /**
   * Gets an error estimate of the state variables recorded by the last call
   * to StepOnceFixedSize(). If the integrator does not support error
   * estimation, this function will return nullptr.
   */
  ContinuousState<T>* get_mutable_error_estimate() { return err_est_.get(); }

  // Sets the actual initial step size taken.
  void set_actual_initial_step_size_taken(const T& h) {
    actual_initial_step_size_taken_ = h;
  }

  /**
   *  Sets the size of the smallest-step-taken statistic as the result of a
   *  controlled integration step adjustment.
   */
  void set_smallest_adapted_step_size_taken(const T& h) {
    smallest_adapted_step_size_taken_ = h;
  }

  // Sets the largest-step-size-taken statistic.
  void set_largest_step_size_taken(const T& h) {
    largest_step_size_taken_ = h;
  }

  // Sets the "ideal" next step size (typically done via error control).
  void set_ideal_next_step_size(const T& h) { ideal_next_step_size_ = h; }

 private:
  // Validates that a smaller step size does not fall below the working minimum
  // and throws an exception if desired.
  void ValidateSmallerStepSize(const T& current_step_size,
                               const T& new_step_size) const {
    if (new_step_size < get_working_minimum_step_size() &&
        new_step_size < current_step_size &&  // Verify step adjusted downward.
        min_step_exceeded_throws_) {
      DRAKE_LOGGER_DEBUG("Integrator wants to select too small step "
          "size of {}; working minimum is ", new_step_size,
          get_working_minimum_step_size());
      std::ostringstream str;
      str << "Error control wants to select step smaller than minimum" <<
           " allowed (" << get_working_minimum_step_size() << ")";
      throw std::runtime_error(str.str());
    }
  }

  // Updates the integrator statistics, accounting for a step just taken of
  // size h.
  void UpdateStepStatistics(const T& h) {
    // Handle first step specially.
    if (++num_steps_taken_ == 1) {
      set_actual_initial_step_size_taken(h);
      set_largest_step_size_taken(h);
    } else {
      if (h > get_largest_step_size_taken()) set_largest_step_size_taken(h);
    }

    // Update the previous step size.
    prev_step_size_ = h;
  }

  // Steps the system forward exactly by @p h, if possible, by calling DoStep
  // or DoDenseStep depending on whether dense integration was started or not.
  // Does necessary pre-initialization and post-cleanup. This method does not
  // update general integrator statistics (which are updated in the calling
  // methods), because error control might decide that it does not like the
  // result of the step and might "rewind" and take a smaller one.
  // @returns `true` if successful, `false` otherwise (due to, e.g., integrator
  //          convergence failure).
  // @note The working minimum step size does not apply here- see
  // @ref integrator-minstep "this section" for details.
  // @sa DoStep()
  // @sa DoDenseStep()
  bool Step(const T& h) {
    if (get_dense_output()) {
      return DoDenseStep(h);
    }
    return DoStep(h);
  }

  // Reference to the system being simulated.
  const System<T>& system_;

  // Pointer to the context.
  Context<T>* context_{nullptr};  // The trajectory Context.

  // Current dense output.
  std::unique_ptr<trajectories::PiecewisePolynomial<T>> dense_output_{nullptr};

  // Runtime variables.
  // For variable step integrators, this is set at the end of each step to guide
  // the next one.
  T ideal_next_step_size_{nan()};  // Indicates that the value is uninitialized.

  // The scaling factor to apply to an integration step size when an integrator
  // convergence failure occurs (to make convergence more likely on the next
  // attempt).
  // TODO(edrumwri): Allow subdivision factor to be user-tweakable.
  const double subdivision_factor_{0.5};

  // The accuracy being used.
  double accuracy_in_use_{nan()};

  // The maximum step size.
  T max_step_size_{nan()};

  // The minimum step size.
  T req_min_step_size_{0};

  // The last step taken by the integrator.
  T prev_step_size_{nan()};

  // Whether error-controlled integrator is running in fixed step mode. Value
  // is irrelevant for integrators without error estimation capabilities.
  bool fixed_step_mode_{false};

  // When the minimum step is exceeded, does the integrator throw an exception?
  bool min_step_exceeded_throws_{true};

  // Statistics.
  T actual_initial_step_size_taken_{nan()};
  T smallest_adapted_step_size_taken_{nan()};
  T largest_step_size_taken_{nan()};
  int64_t num_steps_taken_{0};
  int64_t num_ode_evals_{0};
  int64_t num_shrinkages_from_error_control_{0};
  int64_t num_shrinkages_from_substep_failures_{0};
  int64_t num_substep_failures_{0};

  // Applied as diagonal matrices to weight state change variables.
  Eigen::VectorXd qbar_weight_, z_weight_;

  // State copy for reversion during error-controlled integration.
  VectorX<T> xc0_save_;

  // The error estimate computed during integration with error control.
  std::unique_ptr<ContinuousState<T>> err_est_;

  // The pseudo-inverse of the matrix that converts time derivatives of
  // generalized coordinates to generalized velocities, multiplied by the
  // change in the generalized coordinates (used in state change norm
  // calculations).
  mutable std::unique_ptr<VectorBase<T>> pinvN_dq_change_;

  // Vectors used in state change norm calculations.
  mutable VectorX<T> unweighted_substate_change_;
  mutable std::unique_ptr<VectorBase<T>> weighted_q_change_;

  // Variable for indicating when an integrator has been initialized.
  bool initialization_done_{false};

  // This a workaround for an apparent bug in clang 3.8 in which
  // defining this as a static constexpr member kNaN failed to instantiate
  // properly for the AutoDiffXd instantiation (worked in gcc and MSVC).
  // Restore to sanity when some later clang is current.
  static constexpr double nan() {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double target_accuracy_{nan()};   // means "unspecified, use default"
  T req_initial_step_size_{nan()};  // means "unspecified, use default"
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::IntegratorBase)
