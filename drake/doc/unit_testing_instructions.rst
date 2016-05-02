.. _unit-test-instructions:

***************************************
Detailed Notes on How to Run Unit Tests
***************************************

Finding a Specific Unit Test
============================

To find a specific unit test you need to execute `cmake -N` from within the build artifacts directory::

  cd [drake build artifacts directory]

For in source builds in Linux and Mac this directory is `[drake distro]/drake/pod-build`.
For out of source builds this directory is `[drake build artifacts directory]/drake`.

To retrieve a list of all the available unit tests issue a::

  ctest -N

You can pipe the output of the `ctest -N` command to `grep` if you have a clue about the unit test's name, which you can determine by looking at the `CMakeLists.txt` that included the unit test.



Running a Specific Test
=======================

Once you know the unit tests' name, you can run it by issuing a::

  ctest -VV -R name_of_the_test_as_output_from_ctest-N

where `name_of_the_test_as_output_from_ctest-N` is the name of the test exactly as printed out by `ctest -N` including, if any, the entire path to the test as printed out to the screen. 


Example: Find and run unit test named `cascade_system`
======================================================

Find test::

  cd [drake build artifacts directory]
  ctest -N | grep -i cascade

Run the test::

  ctest -VV -R cascade_system_test

