
/**
@mainpage
<h3>Overview</h3>

<p>Drake's C++ library is composed primarily of an interface for solving
  numerical optimization problems, interfaces for modeling dynamical system,
  and collection of state-of-the-art algorithms for optimization on dynamical
  systems:</p>

 <ul>
   <li> @subpage solvers </li>
   <li> @subpage systems </li>
   <li> @subpage algorithms </li>
   <li> @subpage python_bindings </li>
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
// Define groups here so we can contol the ordering.
// TODO(sherm1) Change Collision Concepts title here when #9467 is fixed.
/**
  @defgroup systems Modeling Dynamical Systems
  @defgroup solvers Formulating and Solving Optimization Problems
  @defgroup multibody_concepts Multibody Dynamics Concepts
  @defgroup collision_concepts Collision Concepts (RigidBodyPlant only)
*/

/* @defgroup algorithms Algorithms

 { @ingroup algorithms
   @defgroup simulation Simulation
   @defgroup analysis Analysis
   @defgroup planning Planning
   @defgroup control Feedback Control Design
   @defgroup estimation State Estimation
   @defgroup identification System Identification
 }
 */

/* @addtogroup technical_notes

 <p>Drake's C++ libraries use a small amount of template metaprogramming to
  enable more advanced features (autodiff, symbolic computation, etc).  We
  have tried to avoid users having to be expert template programmers, but this
  is a good reference if you'd like to
  <a href="http://www.generic-programming.org/languages/cpp/techniques.php">
  learn more about generic programming</a>.</p>
*/

