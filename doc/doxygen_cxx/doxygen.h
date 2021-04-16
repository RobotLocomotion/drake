
namespace drake {

/**
@mainpage
<h3>Overview</h3>

<p>Drake's C++ library is composed primarily of an interface for solving
  numerical optimization problems, interfaces for modeling dynamical system,
  and a collection of state-of-the-art algorithms for optimization on dynamical
  systems:</p>

 <ul>
   <li> @subpage solvers </li>
   <li> @subpage systems </li>
   <li> @subpage algorithms </li>
   <li> @ref drake::examples "Examples" </li>
   <li> @subpage technical_notes </li>
 </ul>

 <p>For more general information, you can also visit the <a
  href="https://drake.mit.edu">Drake documentation main page</a>.</p>
</p>

<h3>How do I document the code I am contributing?</h3>

<p>
If you want to make your own Doxygen locally, see
<a href="https://drake.mit.edu/documentation_instructions.html">Documentation
Generation Instructions</a></p>

<p><a href="https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html">Check
out the Doxygen C++ documentation</a></p>

<p>Drake's Doxygen documentation is
<a href="https://drake.mit.edu">hosted online</a> for the master branch, but is
only updated nightly.</p>
*/

// Define groups here so we can control the ordering.
/**
  @defgroup terminology_and_notation Drake Terminology and Notation
  @defgroup solvers Formulating and Solving Optimization Problems
  @defgroup systems Modeling Dynamical Systems
  @defgroup multibody Multibody Kinematics and Dynamics
  @defgroup algorithms Algorithms
  @defgroup geometry Geometric Representations
  @defgroup technical_notes Technical Notes
*/

/**
  @defgroup accuracy_and_tolerance Accuracy, Tolerance, and Precision
  @ingroup terminology_and_notation
*/

/** @addtogroup accuracy_and_tolerance
 @{
 Most numerical methods are approximate, and many can trade speed for quality
 under user control. We need standard, unambiguous terminology in Drake for
 requesting and reporting solution quality. The terms "tolerance", "accuracy",
 and "precision" are commonly used in this context but often imprecisely. Below
 we present how these terms should be used in Drake, following
 ref. [[1]](https://dx.doi.org/10.1016/j.piutam.2011.04.023). (Some Drake code
 and documentation may not follow these conventions for historical reasons. If
 you spot any violations, please add them to issue
 [#13271](https://github.com/RobotLocomotion/drake/issues/13271) so we can
 modernize.)

 ### accuracy

 `accuracy` α is a unitless quantity describing permissible relative error.
 (This is similar to the "relative tolerance" `rtol` parameter in many
 numerical packages, including some commonly used with Drake.) Ideally you can
 think of `accuracy` as specifying the number of correct significant digits in
 the desired result. For example, specifying `α = 10⁻³` should yield a result
 correct to three digits while `α = 10⁻⁶` should give six. This is a Platonic
 ideal that cannot always be achieved perfectly in practice, but Drake
 algorithms should attempt to respond to a user's `accuracy` request in
 approximately this way.

 We use the unambiguous terms "tight/tighter/tighten accuracy" and
 "loose/looser/loosen accuracy" in preference to "high/more/increase" or
 "low/less/decrease" which are prone to being misunderstood due to the fact
 that numerically smaller values correspond to more (tighter) accuracy.

 ### tolerance

 We use `tolerance` to mean a permissible absolute error ε, which is a quantity
 with units. (This is similar to the "absolute tolerance" `atol` parameter in
 many numerical packages, including some commonly used with Drake.) So
 "constraint tolerance" is the amount by which a user is willing to have a
 constraint violated, in units appropriate to that constraint. Thus a tolerance
 ε=10⁻⁴ for a position constraint means that an error of 0.1mm is acceptable.

 In code we often abbreviate `tolerance` as `tol`, but prefer the longer
 spelling in documentation.

 ### precision

 We do not use this term in Drake in the context of solution quality in
 numerical algorithms. Instead we reserve it for the Machine Learning
 term-of-art as in "precision and recall". "Precision" is also conventionally
 used in the context of floating point arithmetic, such as "double precision"
 and "machine precision".

 ### Given accuracy, we can determine tolerance

 In Drake numerical tests and error controlled numerical methods, we
 typically determine an absolute tolerance using the user-supplied `accuracy`
 value α, via `ε = α|q|` where q is the quantity of interest. This works well
 provided that `|q| ≫ 0`. When q is near zero the tolerance ε could be
 absurdly tight, and if q were exactly zero then we would have a tolerance
 of zero, meaning only a perfect solution would be acceptable. That is not
 achievable on a real computer and is why many packages require a user-specified
 "absolute tolerance" that is used only for quantities near zero. This is an
 awkward requirement for users however, since tolerance is inherently a quantity
 with units and most systems have variables of mixed units. So while `accuracy`
 applies equally to all quantities and is easy to determine (how many digits
 do you want in the answer?), `tolerance` needs to be worked out for each
 quantity. That is rarely done in practice.

 An alternative method, preferred for Drake algorithms, is to make use of known
 system scaling. For example, if we know a "unit length" ℓ (this is also called
 a "characteristic length") for a system we can translate accuracy into
 tolerance via `ε = αℓ`. Similarly we can define unit angle, time, and so on.
 This leads to the following computation of absolute tolerance ε, given only the
 user's requested accuracy α:

     (1)    ε = α max(μ, |q|)

 where μ is the "unit" or "characteristic" value in the same units as the
 quantity of interest q. Because of Drake's use of MKS units and typical
 use for human-scale robots, μ is almost always set to 1 or 0.1. Thus it is
 typically possible to make a good guess at μ, which we have found works about
 as well or better than trying to collect a bunch of absolute tolerances from
 users with more important things on their minds.

 You will see variants of the above Equation (1) used in many of Drake's
 algorithms and test cases. Please use a similar method and similar terminology
 in your own Drake contributions, unless you can make a strong argument that
 something else is necessary.

 - [1] M. Sherman, A. Seth, S. Delp. Procedia IUTAM 2:241-261 (2011),
   Section 3.3. https://dx.doi.org/10.1016/j.piutam.2011.04.023
 @}
*/

/**
  @defgroup multibody_notation Multibody Terminology and Notation
  @ingroup terminology_and_notation
*/

namespace solvers {
/**
  @{
    @defgroup solver_evaluators Costs and Constraints

    Most simple costs and constraints can be added directly to a
    MathematicalProgram through the MathematicalProgram::AddCost() and
    MathematicalProgram::AddConstraint() interfaces and their specializations.

    We also provide a number of classes for common and/or more complex costs
    and constraints, such as those built on the multibody::MultibodyPlant API.

    @ingroup solvers
  @}
*/
} // namespace solvers

/**
  @defgroup constraint_overview Multibody Dynamics Constraints
  @ingroup multibody
*/

// TODO(russt): Take a thorough pass through the algorithms group
// documentation, adding brief descriptions of each and tagging the relevant
// algorithms throughout the code.
/** @addtogroup algorithms
 @{
   @defgroup simulation Simulation
   @defgroup analysis Analysis
   @defgroup planning Planning
   @defgroup control Feedback Control Design
   @defgroup estimation State Estimation
   @defgroup identification System Identification
 @}
 */

/** @addtogroup planning
 @{
   A collection of algorithms for finding configurations and/or trajectories of
   dynamical systems.

   Many planning algorithms make heavy use of solvers::MathematicalProgram and
   the numerous @ref solver_evaluators.  There are also some useful classes in
   @ref manipulation_systems.
 @}
*/

/** @addtogroup control
 @{
   A collection of algorithms for synthesizing feedback control.

   A number of control design algorithms can also be found in @ref
   control_systems (if the design produces a systems::System).
 @}
*/

/** @addtogroup technical_notes

 @{

 @defgroup templates Template MetaProgramming

 <p>Drake's C++ libraries use a small amount of template metaprogramming to
  enable more advanced features (autodiff, symbolic computation, etc).  We
  have tried to avoid users having to be expert template programmers, but this
  is a good reference if you'd like to
  <a href="http://www.generic-programming.org/languages/cpp/techniques.php">
  learn more about generic programming</a>.</p>

 @}
*/

} // namespace drake