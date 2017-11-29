***************
CLion IDE setup
***************

This guide describes how to set up Drake in the JetBrains CLion IDE.

Using CLion with Bazel
======================

(See note below about CLion versions compatible with Bazel.)

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
mechanisms will have already sourced the ``.bashrc``.

**macOS users** will get broken behavior by default.  When you run an macOS app
graphically, the parent process is `launchd` (PID 1), which provides its own
standard environment variables to the child process.  In particular, it provides
a minimal ``PATH`` that does not include ``/usr/local/bin``, where most Homebrew
executables are installed.  Consequently, the Bazel build will fail to find
Homebrew dependencies like ``glib`` and ``pkg-config``.

The simplest solution is not to launch CLion graphically. Instead, configure
your shell environment properly in ``.bashrc``, and launch CLion from the
command line::

  /Applications/CLion.app/Contents/MacOS/clion

If you strongly prefer clicking on buttons, you might be able to configure the
``launchd`` environment using ``launchctl``, but this process is finicky. We
have no reliable recipe for it yet.

Installing CLion
----------------

1. Go to https://www.jetbrains.com/clion/download/. Download the latest version
   of CLion. For macOS, choose the version with bundled custom JDK.
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

First, make sure you have installed ``clang-format-4.0``
(see :doc:`code_style_tools`).

Clang format selected file
^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Clang Format Full File``
  :Description: ``Apply clang-format to the active file``
  :Program: ``clang-format-4.0``
  :Parameters: ``-i $FileName$``
  :Working directory: ``$FileDir$``

Leave the checkbox options in their default state.

Clang format selected lines
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Clang Format Selected Lines``
  :Description: ``Apply clang-format to the selected lines``
  :Program: ``clang-format-4.0``
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
  :Parameters: ``run //tools/lint:clang-format-includes -- $FilePath$``
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
  :Parameters: ``run @styleguide//:cpplint -- --output=eclipse
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
  :Parameters: ``run //tools/lint:drakelint -- $FilePath$``
  :Working directory: ``$Projectpath$``

In the event of finding a lint problem (e.g., out-of-order include files), the
CLion output will contain a *single* clickable link. This link is only the
*first* error encountered in the include section; there may be more. The link
merely provides a hint to the developer to see the problem area. Rather than
fixing by hand, we strongly recommend executing the ``Clang Format Include
Ordering`` external tool on the file.

Alternative linting configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The linting tools have been configured to use the bazel system. The advantage in
doing so is that it guarantees that the tools are built prior to being used.
However, bazel only allows one instance of bazel to run at a time. For example,
if building Drake in a command-line window, it would be impossible to lint files
at the same time.

The work around is to change the configurations to execute the binaries
directly. This approach generally works but will fail if the corresponding bazel
targets have not been built. The tools would need to be built prior to
execution.

With this warning in place, you can make the following modifications to the
linting tools to be able to lint and compile simultaneously.

Google style guide linting
""""""""""""""""""""""""""

Change the following fields in the instructions given above:

  :Program: ``bazel-bin/external/styleguide/cpplint_binary``
  :Parameters: ``--output=eclipse $FilePath$``

Building the google styleguide lint tool:

``bazel build @styleguide//:cpplint``

Drake style addenda
"""""""""""""""""""

Change the following fields in the instructions given above:

  :Program: ``/bazel-bin/tools/lint/drakelint``
  :Parameters: ``$FilePath$``

Building the drake addenda lint tool:

``bazel build //tools/lint:drakelint``
