%> @mainpage
%>
%> 
%> <span style="font-weight:bold; font-size:larger">First time?  Check out the <a href="inherits.html">Class Hierarchy</a></span>.
%> <br />
%> <p>
%> If you want to make your own doxygen, run <b><i>make doxygen</i></b> in the root drake directory.  You may need to install doxygen (<i>sudo apt-get install doxygen</i>).</p>
%> <p>
%> Other useful pages:
%> <ul>
%> <li><a href="class_drake_system.html">DrakeSystem</a></li>
%> <li><a href="class_rigid_body_manipulator.html">RigidBodyManipulator</a></li>
%> <li><a href="https://github.com/RobotLocomotion/drake/wiki/Documentation">Drake documentation main page</a></li>
%> <li><a href="https://github.com/RobotLocomotion/drake/raw/master/doc/drake.pdf">Conceptual overview (pdf)</a></li>
%> <li><a href="http://rusty-drake.csail.mit.edu/docs/drakedocs/urdf/drakeURDF.html">URDF reference</a></li>
%> </ul>
%> </p>
%>
%>
%> <h3>How do I make my documentation look pretty?</h3>
%> <p>Comment your code in a MATLAB style (directly after the function definition) and the <a href="https://github.com/RobotLocomotion/drake/tree/master/doc/DoxygenMatlab">scripts</a> will deal with the rest.  There are a few commands that can help:</p>
%> <ul>
%> 	<li><b>\@param</b>: describe an input -- for example,
%>		<ul><li><i>\@param K the gain matrix</li><li>\@param x0 initial condition</i>
%>      <br /><br />
%>		@param K the gain matrix
%>		@param x0 initial condition
%>	</li></ul></li>
%> 	<p><li><b>\@retval</b>: describe a return value.
%>		<ul><li><i>\@retval traj simulated trajectory</li><li>\@retval utraj simulated input trajectory (type PolynomialTrajectory)</i><br /><br />
%>		@retval traj simulated trajectory
%>		@retval utraj simulated input trajectory (type PolynomialTrajectory)
%>	</li></ul></li>
%> 	<li><b>\@option</b>: describe an option parameter.</li>
%> 	<li><b>\@default</b>: list the default for an option.</li>
%> </ul>
%> 
%> <p>
%> Doxygen also supports most HTML tags including <b>\<pre\></b>, <b>\<b\></b>, <b>\<i\></b>, and more.  Take a look at <a href="http://www.stack.nl/~dimitri/doxygen/commands.html">the full command list</a>.
%>
%> Drake's doxygen documentation is automatically updated on every push to
%> the <a href="https://github.com/RobotLocomotion/drake">github repository</a>, so it
%> should always be up to date.
%>
%> <br /><br /><br /><br /><hr />
%> Brought to you by the MIT Robot Locomotion Group: http://groups.csail.mit.edu/locomotion/
