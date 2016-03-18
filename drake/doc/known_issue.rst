******************************
Build server known issue logic
******************************


We should absolutely aim for every unit/regression test to pass on every build.  But if we do find issues with the code, then it is better to add a (failing) test to the build server than to pretend the failure does not exist.  To help keep the signal-to-noise ratio high, we have implemented a simple "known issue" logic on the build server.

Any github issue attached to this main drake repository can add logic in it's description / initial comment field by adding a ``<matlab_test>`` xml field to the comment which contains a single matlab expression which, when evaluated using ``eval`` returns true iff the failing unit test was caused by the current issue.  The ``eval`` statement can make use of the following variables:

* ``test_name`` - the name of the test including the directory relative to the drake root folder
* ``error_id`` - the short string from the error command, e.g. ``'PathLCP:FailedToSolve'``
* ``error_message`` - the long string from the error command, e.g. something in matlab called ``error(error_id,error_msg)``
* ``is_segfault`` - true iff the server thinks that this unit test caused a segmentation fault
* ``test_output`` - the full matlab console output from this test (which can be searched for keywords as necessary)

The logic should be as discriminating as possible to make sure that we only catch exactly the tests which are really triggered by the current issue.

For example::

	<matlab_test>is_segfault && strcmp(test_name,'examples/Atlas/test/runAtlasWalkingTestMex')</matlab_test>

or ::

	<matlab_test> strcmp(error_id,'PathLCP:FailedToSolve') && ...
    any(strcmp(test_name,{'systems/plants/test/fallingBrickLCP', ...
          'systems/plants/test/fallingCapsulesTest',...
          'systems/plants/test/multiRobotTest', ...
          'systems/plants/test/momentumTest', ...
          'systems/plants/test/terrainTest', ...
          'systems/plants/test/testFloatingBaseDynamics', ...
          'examples/PR2/runPassive',...
          'examples/Quadrotor/Quadrotor.runOpenLoop', ...
          'examples/ZMP/CartTable.run'}))</matlab_test>

The build server will only check issues for *open* issues.  Once an issue has been closed it will no longer count as known.
