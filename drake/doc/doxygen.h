
/**
@mainpage
<h3>Overview</h3>

<p>Drake's C++ library is composed primarily of an interface for solving
  numerical optimization problems, interfaces for modeling dynamical system,
  and collection of state-of-the-art algorithms for optimization on dynamical
  systems:</p>
<ul>
  <li><a href="group__solvers.html">Formulating and Solving Optimization
    Problems</a></li>
  <li><a href="group__systems.html">Modeling Dynamical Systems</a></li>
  <ul>
    <li><a href="class_rigid_body_tree.html">Rigid-Body Kinematics and
  Dynamics</a></li>
  </ul>
  <li>Algorithms</li>
  <ul>
    <li><a href="group__simulation.html">Simulation</a></li>
    <li>Analysis</li>
    <li>Planning</li>
    <li>Feedback Control Design</li>
    <li>State Estimation</li>
    <li>System Identification</li>
  </ul>
</ul>
<p>For more general information, you can also visit the <a
  href="https://github.com/RobotLocomotion/drake/wiki/Documentation">Drake
  documentation main page</a></p>
</p>

<p>Drake's C++ libraries use a small amount of template metaprogramming to
  enable more advanced features (autodiff, symbolic computation, etc).  We
  have tried to avoid users having to be expert template programmers, but this
  is a good reference if you'd like to
  <a href="http://www.generic-programming.org/languages/cpp/techniques.php">
  learn more about generic programming</a>.</p>

<h3>How do I document the code I am contributing?</h3>

<p>
If you want to make your own Doxygen locally, run
<b><i>make documentation</i></b> in
the build/drake directory.  You may need to install Doxygen (e.g., with <i>sudo
apt-get install doxygen</i> on Ubuntu).</p>
<p>

<p><a href="https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html">Check
out the Doxygen C++ documentation</a></p>

<p>Drake's Doxygen documentation for the master branch is automatically updated
on every push to the <a href="https://github.com/RobotLocomotion/drake">github
repository</a>, so it should always be up to date.</p>

*/

