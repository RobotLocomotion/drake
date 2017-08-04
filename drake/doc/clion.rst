***************
CLion IDE setup
***************

This guide describes how to set up Drake in the JetBrains CLion IDE.
It is assumed that ``drake-distro`` is
:ref:`already installed <installation_and_quick_start>`.

Using CLion with CMake
======================

(We recommend using Bazel rather than CMake; see instructions below.
**CAUTION**: the latest Bazel plugin (version 2017.07.05.0.2) does not
necessarily work with the latest CLion. We have tested it up to CLion 2017.1.3
and it does not currently work with CLion 2017.2)

Installing CLion
----------------

1. Go to https://www.jetbrains.com/clion/download/. Download the latest version
   of CLion. For OSX, choose the version with bundled custom JDK.
2. Install CLion. Exact steps depend on your platform, but it's
   straightforward. Just using defaults for everything is fine. You now have a
   30-day trial version of CLion. Either try it out as is, or get a free
   academic license `here <https://www.jetbrains.com/shop/eform/students>`_.

Upgrading CLion
---------------

Users upgrading from a previous version of CLion should do the following:

1. To have your Unity launcher CLion icon point to the correct version,
   run ``locate jetbrains-clion.desktop`` and edit the located file. If more
   than one file is located, you may want to consolidate to a single launch file
   in your user directory, typically ``~/.local/share/applications``.
2. Uninstall the previous version of the Bazel plugin and update to the latest
   version. See `Installing the Bazel Plugin`_.
3. CLion 2017.1.3 users, confirm that you are using Bazel plugin 2017.05.02
   and Bazel version 0.5.2.

**Note**: It is not necessary to import your project into a *new* CLion project.
Overwriting the old project is appropriate (i.e., the directory likely located
in ``~/ClionProjects/project-name``).

Setting up Drake in CLion
-------------------------

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
--------

To build Drake (and only Drake, this assumes that dependencies were already
built!):

1. Go to Run > Edit Configurations.
2. Select 'Build All' and click OK.
3. Click Run > Build (or use the keyboard shortcut).

Note that it's possible to select any CMake target you want from the Edit
Configurations menu. This can come in very handy when you want to quickly
iterate on a e.g. a specific executable without building all of Drake all the
time.

Running a C++ executable
------------------------
1. Go to Run > Run...
2. Click an executable, or start typing to find your favorite executable and hit
   enter.

Debugging .mex functions in OSX
-------------------------------

1. Go to Run > Edit Configurations
2. Click the + in the top left corner to create a new Run/Debug Configuration.
3. Name it Matlab
4. Use All targets as the Target
5. For Executable, click on the drop-down menu, scroll all the way down and
   click Select Other...
6. Browse to your Matlab executable. For OSX you can just use
   Applications/MATLAB_R2014a.app or something similar.
7. As the working directory, use ``drake-distro/drake``.
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

Using CLion with Bazel
======================

(See note above about CLion versions compatible with Bazel.)

First, install Bazel and build Drake with Bazel, following
:ref:`the Drake Bazel instructions <bazel>`.

A Note About Environment Variables
----------------------------------
CLion forwards environment variables to the processes it launches, including
the Bazel client and server. We have a number of Bazel repository rules that
consult environment variables, especially ``PATH``, to locate external
dependencies. Therefore, some care is necessary to make sure CLion is launched
with the environment you actually want!

**Ubuntu users** will generally get good behavior by default, because ``apt``
installs binaries in reasonable, standard paths, and because most CLion launch
mechanisms will have already sourced the ``.bashrc``. Do be careful that
``ccache`` is not on your ``PATH``, though.  If you launch CLion with ``ccache``
on your ``PATH``, and then CLion launches a Bazel server, you'll need to quit
CLion, kill the Bazel server, and run ``bazel clean`` to recover.

**OS X users** will get broken behavior by default.  When you run an OS X app
graphically, the parent process is `launchd` (PID 1), which provides its own
standard environment variables to the child process.  In particular, it provides
a minimal ``PATH`` that does not include ``/usr/local/bin``, where most Homebrew
executables are installed.  Consequently, the Bazel build will fail to find
Homebrew dependencies like ``glib``, ``pkg-config``, and ``gfortran``.

The simplest solution is not to launch CLion graphically. Instead, configure
your shell environment properly in ``.bashrc``, and launch CLion from the
command line::

  /Applications/CLion.app/Contents/MacOS/clion

If you strongly prefer clicking on buttons, you might be able to configure the
``launchd`` environment using ``launchctl``, but this process is finicky. We
have no reliable recipe for it yet.

Installing the Bazel Plugin
---------------------------

To use Bazel in CLion, you must install a plugin supplied by Google. The plugin
requires CLion 2016.3 or later.  To install the plugin, open
``File > Settings``, select ``Plugins``, and press the ``Browse repositories``
button.  Locate and install the ``CLion with Bazel`` plugin. You will be
prompted to restart CLion.

Setting up Drake in CLion
-------------------------
CLion will invoke Bazel to build Drake, including the external dependencies
specified in the WORKSPACE file.

1. ``File > Import Bazel Project``
2. Select Workspace: Use an existing Bazel workspace, and provide the path to
   your ``drake-distro`` directory.
3. (Sometimes) Select Bazel Executable: If prompted, specify the path to your
   Bazel executable. The default is probably correct.
4. Select Project View: choose "Import from workspace", and
   select the file ``drake-distro/.bazelproject``
5. Project View: Pick a ``project data directory`` of your choice for the
   CLion project files. It must not be a subdirectory of ``drake-distro``.
6. (Advanced) Project View: If you only wish to develop a subset of Drake,
   you can specify only those files and targets in the project view file.
   Most users should leave it as-is.
7. Click "Finish".  CLion will begin ingesting the Drake source, building
   symbols, and compiling Drake. This will take several minutes.

Building and Running Targets
----------------------------

To build all of Drake with default Bazel options, select
``Bazel > Build > Compile Project``.

To build or run a specific target go to ``Run > Edit Configurations``. Click
``+`` to create a new Bazel command.  Specify the configuration name and Bazel
options. The ``Target expression`` specifies the actual code (library, binary,
and/or test) that you want to run. To learn more about target expressions, see
`the Bazel manual
<https://bazel.build/versions/master/docs/bazel-user-manual.html#target-patterns>`
_. Once you've created a configuration, you can launch it from the ``Run`` menu.

To run a specific target in the debugger, create a configuration as above,
using the ``bazel run`` command. Then launch it from ``Run > Debug``.

Keeping CLion Up-to-Date with the Bazel Build
---------------------------------------------

Changes to BUILD files can add or remove source files from the Bazel build.
To propagate those changes into the CLion project structure, select
``Bazel > Sync Project With BUILD Files``.

Git Integration
---------------

CLion provides a user interface for Git, which you can enable in the ``VCS``
menu.  It automatically detects all Git roots within the workspace. This will
include ``bazel-drake-distro``, which is a Bazel-internal detail. Bazel edits
the contents of that directory for its own purposes, and those changes will
spuriously appear in the CLion UI as changes that need to be committed. To make
CLion ignore ``bazel-drake-distro``, enable Git integration under the ``VCS``
tab, then go to ``File > Settings``. Select the ``Version Control`` menu item
directly (not one of the subtopics displayed when that item is expanded). You
will see a list of all the Git root directories. Look for ``bazel-drake-distro``
on that list and select it. On the right hand side are ``+`` and ``-`` buttons;
click ``-`` to remove the spurious root directory. After that you should be
able to go to ``VCS > Commit Changes`` and there should be no changes seen.

Integrating External Tools with CLion
=====================================

.. role:: raw-html(raw)
   :format: html

CLion provides a mechanism for invoking external binaries/scripts/etc. with
parameters derived from the CLion GUI. Below, we outline a number of common
tools to aid with compliance with the Drake style guide. The work to create
a new external tool is the same in all cases; only the specific tool settings
differ from tool to tool. We'll outline the general work here and provide
per-tool details below.

1. Open the Settings dialog (``File`` > ``Settings``) or ``Alt+Ctrl+S``.
2. Navigate to ``Tools`` > ``External Tools``.
3. Click the :raw-html:`<font size="5" color="green">+</font>` sign to add a new
   tool.
4. Set the appropriate fields in the ``Edit Tool``. See the following tools for
   details.
5. Click ``Ok``.

There are several ways to use an *External Tool*. One is to right-click on a
file and select ``External Tools`` > ``Tool Name``. Another is to select
``Tools`` > ``External Tools`` > ``Tool Name``. For tools that operate on a
selected file, make sure that file is "active" by clicking on it. The
``Tool Name`` will be the value set in the ``Name`` field outlined below.

.. _integrating_format_tools_with_clion:

Formatting files
----------------

You can use clang format to modify the formatting of your file in the GUI. We'll
introduce three variants:

- Apply clang-format to a whole file.
- Apply clang-format to selected lines.
- Apply clang-format to correct ``#include`` ordering.

These tools modify the selected file. There is a synchronization issue with
CLion such that the modification may not be immediately apparent. When in doubt,
select away from the target file and back; this will cause the file to refresh
and you can confirm that the file has been modified as expected.

First, make sure you have installed ``clang-format``
(see :doc:`code_style_tools`).

Clang format selected file
^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Clang Format Full File``
  :Description: ``Apply clang-format to the active file``
  :Program: ``clang-format``
  :Parameters: ``-i $FileName$``
  :Working directory: ``$FileDir$``

Leave the checkbox options in their default state.

Clang format selected lines
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Clang Format Selected Lines``
  :Description: ``Apply clang-format to the selected lines``
  :Program: ``clang-format``
  :Parameters: ``-lines $SelectionStartLine$:$SelectionEndLine$ -i $FileName$``
  :Working directory: ``$FileDir$``

Leave the checkbox options in their default state.

Correct #include ordering
^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Clang Format Include Ordering``
  :Description: ``Runs the clang format for correcting includes on the current
                  file``
  :Program: ``bazel``
  :Parameters: ``run //drake/tools:clang-format-includes -- $FilePath$``
  :Working directory: ``$Projectpath$``

Leave the checkbox options in their default state.

.. _integrating_lint_tools_with_clion:

"Linting" files
---------------

"Linting" refers to using tools to find aspects of code which don't conform
to specified coding practices. You can apply Drake's linting tools in CLion to
find such issues. We'll define two tools:

- General linting (via cpplint) which captures most of the Drake style guide.
- Drake extended linting which captures aspects of the Drake style guide _not_
  captured by the general linting tool. This includes detecting out-of-order
  ``#include`` directives.

These tools produce reports. In some cases, the reports can be automatically
converted into clickable links so that you can click on a messsage and be taken
to the file and line indicated in the message. The configuration instructions 
include the details of how to configure these clickable links.

You can also set the general coding style for CLion through the following steps

1. Go to ``File`` > ``Settings`` > ``Editor`` > ``Code Style``
2. On the right panel, Go to ``Default Options`` > ``Right margin (columns)``: 
   Set it to 80
3. Go to ``File`` > ``Settings`` > ``Editor`` > ``Code Style`` > ``C/C++``
4. On the right panel, choose ``Set from`` > ``Predefined Style`` > ``Google``

Lint selected file for google style guide
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Cpplint File``
  :Description: ``Apply cpplint to the current file``
  :Program: ``bazel``
  :Parameters: ``run @google_styleguide//:cpplint -- --output=eclipse
                 $FilePath$``
  :Working directory: ``$Projectpath$``

To configure the clickable links:

1. Click the ``Output Filters...`` button.
2. Click the :raw-html:`<font size="5" color="green">+</font>` sign to add a
   filter.
3. Add the following values in the following fields (and click "OK):

  :Name: ``Extract Links``
  :Description: ``Convert file/line references into clickable links.``
  :Regular expression to match output: ``$FILE_PATH$:$LINE$``

4. Click ``OK`` on the ``Edit filter`` dialog.
5. Click ``OK`` on the ``Output Filters`` dialog.

Lint selected file for Drake style addenda
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This tool is a supplement to the google style cpplint. It tests for additional
style requirements which are otherwise missed by the general tool. The primary
reason to run this is to confirm that the order of the ``#include`` statements
is correct.

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Drake Lint File``
  :Description: ``Apply drake lint to the current file``
  :Program: ``bazel``
  :Parameters: ``run //tools:drakelint -- $FilePath$``
  :Working directory: ``$Projectpath$``

This tool does not *currently* produce output from which clickable links can be
made. In the event that it reports a problem with the includes, simply execute
the ``Clang Format Include Ordering`` external tool on the file.
