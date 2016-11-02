*****************************************
CLion IDE setup (experimental)
*****************************************

This guide describes how to set up Drake in the new JetBrains CLion IDE and has
been tested with CLion 1.1. It is assumed that ``drake-distro`` is
:ref:`already installed <installation_and_quick_start>`. CLion support is
currently experimental. I wrote this guide based on my OSX setup. I tried it on
Ubuntu, and CLion got stuck loading the CMake Project forever on my
machine. This happened because a call to Matlab wouldn't return and required
setting _matlab_root manually in mex.cmake.

Installing CLion
================

1. Go to https://www.jetbrains.com/clion/download/. Download the latest version
   of CLion. For OSX, choose the version with bundled custom JDK.
2. Install CLion. Exact steps depend on your platform, but it's
   straightforward. Just using defaults for everything is fine. You now have a
   30-day trial version of CLion. Either try it out as is, or get a free
   academic license `here <https://www.jetbrains.com/shop/eform/students>`_.

Setting up Drake in CLion
=========================

We'll set up CLion to build Drake only. The dependencies will just be built
from the command line. It is assumed Drake was cloned into a ``drake-distro``
directory as described in the :ref:`installation instructions <getting_drake>`.

1. :ref:`Build Drake <build_the_collection>` from the command line.
2. In the Welcome to CLion screen, click ``Import Project from Sources``. Or
   from the menu ``File > Import Project...``
3. Browse and select ``drake-distro/drake`` and click OK.
4. When asked whether to overwrite the existing ``CMakeLists.txt``, just click
   ``Open Project``.
5. Go to CLion Preferences (on OSX, ``CLion-EAP > Preferences``, on Ubuntu,
   ``File > Settings...``). Note that this preferences window comprises both
   global settings and project-specific settings (as noted by the text 'for
   current project' on some of the preference pages).
6. Browse to ``Build, Execution, Deployment > CMake``.
7. Under ``CMake Options``, fill in
   ``-DCMAKE_INSTALL_PREFIX=/absolute_path_to_your/drake-distro/build/install``.
8. [This step only for Ubuntu 14.04 - Trusty]. Under ``CMake Options`` 
   expand the tab ``Pass system 
   environment``. Add the following environment variables.  (You can copy these
   from this documentation one at a time and click on the ``paste`` button at
   the right of the environment variables dialog.)

   * ``CC=gcc-4.9``
   * ``CXX=g++-4.9``
   * ``FC=gfortran-4.9``

9. Click OK. CLion will take about a minute to reload the CMake Project. If
   everything is in order, there should be no errors or warnings. For fun,
   check out the `cache pane
   <https://www.jetbrains.com/help/clion/2016.1/cmake-cache.html>`_ in the
   `CMake tool window
   <https://www.jetbrains.com/help/clion/2016.1/cmake.html>`_ (the CMake icon
   in the lower left corner of the workspace); it's pretty handy.
10. If CLion asks you about unregistered VCS roots, you can just add them.

Building
========

To build Drake (and only Drake, this assumes that dependencies were already
built!):

1. Go to Run > Edit Configurations.
2. Select 'Build All' and click OK.
3. Click Run > Build (or use the keyboard shortcut).

Note that it's possible to select any CMake target you want from the Edit
Configurations menu. This can come in very handy when you want to quickly
iterate on a e.g. a specific executable without building all of Drake all the
time.

Code formatter settings
=======================

1. Make sure you have installed ``clang-format`` (see :doc:`code_style_tools`)
2. Go to File > Settings > Tools > External Tools
3. Add an entry for clang-format with

   * Program: ``clang-format``
   * Parameters (whole file): ``-i $FileName$``
   * Parameters (current selection only): 
     ``-lines $SelectionStartLine$:$SelectionEndLine$ -i $FileName$``
   * Working directory : ``$FileDir$``

Choose one or the other of the parameter settings. Now you can run this
(manually) on any file using Tools > External Tools in the drop down menu. You
can also add a keyboard shortcut.

You can also set the coding style through the following steps

1. Go to File > Settings > Editor > Code Style
2. On the right panel, Go to Default Options > Right margin (columns): Set it to 80
3. Go to File > Settings > Editor > Code Style > C/C++
4. On the right panel, choose Set from > Predefined Style > Google


Running a C++ executable
========================
1. Go to Run > Run...
2. Click an executable, or start typing to find your favorite executable and hit enter.

Debugging .mex functions in OSX
===============================

1. Go to Run > Edit Configurations
2. Click the + in the top left corner to create a new Run/Debug Configuration.
3. Name it Matlab
4. Use All targets as the Target
5. For Executable, click on the drop-down menu, scroll all the way down and
   click Select Other...
6. Browse to your Matlab executable. For OSX you can just use
   Applications/MATLAB_R2014a.app or something similar.
7. As the working directory, use /Users/twan/code/drake-distro/drake (adapted
   to your system)
8. Under Environment Variables, add a variable GRB_LICENSE_FILE and set it to
   the absolute path of your Gurobi license file. If you don't do this, Gurobi
   will not be able to find the license file since Gurobi relies on either the
   GRB_LICENSE_FILE or the HOME environment variable (if the license file is in
   the default location) to find it.
9. Leave everything else as is. Click OK to save the Run/Debug Configuration.
10. Click Run > Debug Matlab.
11. Once CLion is done building and you're in the Debug pane, click the
    Debugger tab and then the LLDB subtab.
12. Enter the following: ``process handle -p true -n false -s false SIGSEGV
    SIGBUS`` (taken from
    http://www.mathworks.com/help/matlab/matlab_external/debugging-on-mac-platforms.html)
    and hit enter.
13. Click Resume Program (play button) twice. Matlab should start up. Once it's
    started, you can run whatever Matlab code you like. You can set breakpoints
    in the C++ code in CLion, and if that code is called from Matlab and the
    breakpoint is hit, you'll be able to step through in CLion and inspect
    variables.

Note: if Matlab asks for activation, you'll need to copy the license (.lic)
file from ~/.matlab/R2014b_licenses (or whatever version of Matlab you have) to
the licenses subfolder of your Matlab installation
(e.g. /Applications/MATLAB_R2014b.app/licenses). If the licenses subfolder does
not exist, create it.
