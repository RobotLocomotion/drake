
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
  @defgroup solvers Formulating and Solving Optimization Problems
  @defgroup systems Modeling Dynamical Systems
  @defgroup multibody Multibody Kinematics and Dynamics
  @defgroup algorithms Algorithms
  @defgroup technical_notes Technical Notes
*/

/**
  @defgroup multibody_notation Terminology and Notation
  @ingroup multibody
*/

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

