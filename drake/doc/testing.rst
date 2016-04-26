***************************************
Detailed Notes on How to Run Unit Tests
***************************************

Finding a Specific Unit Test
============================

::

   cd path/to/your/drake-distro/drake/pod-build
   ctest -N

This will list all the available tests. You can pipe the output of the `ctest -N` command to `grep` if you have a clue on how the test is named. 


Running a Specific Test
=======================

Once you know how the test is named you can run it by issuing a::

  ctest -VV -R name_of_the_test_as_ouput_from_ctest-N

where `name_of_the_test_as_ouput_from_ctest-N` is the name of the test exactly as printed out by `ctest -N` including, if any, the entire path to the test as printed out to the screen. 


Example: Find and run test `cascade_system`
=============================================

Find test::

  cd path/to/your/drake-distro/drake/pod-build
  ctest -N | grep -i cascade

Run the test::

  ctest -VV -R cascade_system_test

