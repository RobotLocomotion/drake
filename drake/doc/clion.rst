*****************************************
CLion IDE setup (experimental)
*****************************************

This guide describes how to set up Drake in the new JetBrains CLion IDE and has been tested with CLion 1.1. It is assumed that drake-distro is [already installed.](Installation-and-QuickStart) CLion support is currently experimental. I wrote this guide based on my OSX setup. I tried it on Ubuntu, and CLion got stuck loading the CMake Project forever on my machine. This happened because a call to Matlab wouldn't return and required setting _matlab_root manually in mex.cmake.

Installing CLion
================
1. Go to https://www.jetbrains.com/clion/download/. Download the latest version of CLion. For OSX, choose the version with bundled custom JDK.
2. Install CLion. Exact steps depend on your platform, but it's straightforward. Just using defaults for everything is fine. You now have a 30-day trial version of CLion. Either try it out as is, or get a free academic license [here](https://www.jetbrains.com/shop/eform/students).

Setting up Drake in CLion
=========================
We'll set up CLion to build Drake only. The dependencies will just be built from the command line.

1. Build drake-distro from the command line.
2. In the Welcome to CLion screen, click 'Import Project from Sources'.
3. Browse to the drake subfolder of your drake-distro install and click OK.
4. When asked whether to overwrite the existing CMakeLists.txt, just click Open Project.
5. Go to CLion Preferences (on OSX, CLion-EAP > Preferences, on Ubuntu, File > Settings). Note that this preferences window comprises both global settings and project-specific settings (as noted by the text 'for current project' on some of the preference pages).
6. Browse to Build, Execution, Deployment > CMake
7. Under CMake Options, fill in `-D CMAKE_INSTALL_PREFIX=/Users/twan/code/drake-distro/build` or whatever the path to your drake-distro build folder may be.
8. Under Build Output Path, fill in `pod-build`
9. Click OK. CLion will take about a minute to reload the CMake Project. If everything is in order, there should be no errors or warnings. For fun, check out the CMake Cache pane too by the way; it's pretty handy.
10. If CLion asks you about unregistered VCS roots, you can just add them.

Building
========
To build Drake (and only Drake, this assumes that dependencies were already built!):

1. Go to Run > Edit Configurations.
2. Select 'Build All' and click OK.
3. Click Run > Build (or use the keyboard shortcut).

Note that it's possible to select any CMake target you want from the Edit Configurations menu. This can come in very handy when you want to quickly iterate on a e.g. a specific executable without building all of Drake all the time.

Code formatter settings
=======================

1. Make sure you have installed `clang-format` (see :code_style_tools:)
2. Go to File > Preferences > Tools > External Tools
3. Add an entry for clang-format with

  * Program: `clang-format`
  * Parameters: `-i $FilePath$`
  * Working directory : `$ProjectFileDir$/..`

Now you can run this (manually) on any file using Tools > External Tools in the drop down menu.

NB: There is probably a better way to do this (see Editor > Code Style > C/C++).


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
5. For Executable, click on the drop-down menu, scroll all the way down and click Select Other...
6. Browse to your Matlab executable. For OSX you can just use Applications/MATLAB_R2014a.app or something similar.
7. As the working directory, use /Users/twan/code/drake-distro/drake (adapted to your system)
8. Under Environment Variables, add a variable GRB_LICENSE_FILE and set it to the absolute path of your Gurobi license file. If you don't do this, Gurobi will not be able to find the license file since Gurobi relies on either the GRB_LICENSE_FILE or the HOME environment variable (if the license file is in the default location) to find it.
9. Leave everything else as is. Click OK to save the Run/Debug Configuration.
10. Click Run > Debug Matlab.
11. Once CLion is done building and you're in the Debug pane, click the Debugger tab and then the LLDB subtab.
12. Enter the following: `process handle -p true -n false -s false SIGSEGV SIGBUS` (taken from http://www.mathworks.com/help/matlab/matlab_external/debugging-on-mac-platforms.html) and hit enter.
13. Click Resume Program (play button) twice. Matlab should start up. Once it's started, you can run whatever Matlab code you like. You can set breakpoints in the C++ code in CLion, and if that code is called from Matlab and the breakpoint is hit, you'll be able to step through in CLion and inspect variables.

Note: if Matlab asks for activation, you'll need to copy the license (.lic) file from ~/.matlab/R2014b_licenses (or whatever version of Matlab you have) to the licenses subfolder of your Matlab installation (e.g. /Applications/MATLAB_R2014b.app/licenses). If the licenses subfolder does not exist, create it.
