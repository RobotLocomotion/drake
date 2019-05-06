***************
CLion IDE setup
***************

.. note::

  EVERY SETUP STEP IN THIS DOCUMENT IS CRITICAL TO GET CLION WORKING
  PROPERLY.  Read carefully, and do not skip anything.

.. note::

  The Bazel plugin for CLion does not support macOS, per
  `bazelbuild/intellij#109 <https://github.com/bazelbuild/intellij/issues/109>`_.

This guide describes how to set up Drake in the JetBrains CLion IDE on Ubuntu.

Using CLion with Bazel
======================

First, you **must** install Bazel and build Drake with Bazel, following
:ref:`the Drake Bazel instructions <bazel>`.

To use Drake with CLion, your Drake checkout **must** be named ``drake``.

Installing CLion
----------------

1. Go to https://www.jetbrains.com/clion/download/. Download the latest version
   of CLion.
2. Install CLion. Exact steps depend on your platform, but it's
   straightforward. Just using defaults for everything is fine. You now have a
   30-day trial version of CLion. Either try it out as is, or get a free
   academic license `here <https://www.jetbrains.com/shop/eform/students>`_.

The most recent versions that we have tested for compatibility are:
  - Ubuntu 16.04
  - Bazel 0.23.1
  - CLion 2018.3.4 (January 31, 2019) with:

    - Bazel plugin 2019.01.14.0.5.

Many versions of the above (Bazel / CLion / Bazel plugin) are *not* compatible
with each other.  We strongly suggest using only the versions shown above, when
working with Drake.

At the time of this writing, CLion 2018.3.4 will pick Bazel plugin
2019.03.05.0.1, which has a problem with ``Run > Debug...``
(``Run > Run...`` is fine). We have to downgrade Bazel plugin to
2019.01.14.0.5. See `Downgrading the Bazel Plugin`_.

Upgrading CLion
---------------

Users upgrading from a previous version of CLion should do the following:

1. To have your Unity launcher CLion icon point to the correct version,
   run ``locate jetbrains-clion.desktop`` and edit the located file. If more
   than one file is located, you may want to consolidate to a single launch file
   in your user directory, typically ``~/.local/share/applications``.
2. Uninstall the previous version of the Bazel plugin and update to the latest
   version. See `Installing the Bazel Plugin`_.

**Note**: It is not necessary to import your project into a *new* CLion project.
Overwriting the old project is appropriate.

Installing the Bazel Plugin
---------------------------

To use Bazel in CLion, you **must** install a plugin supplied by Google.  To
install the plugin, open ``Settings`` (either ``Welcome > Configure >
Settings`` or ``File > Settings``), select ``Plugins``, and press the ``Browse
repositories`` button.  Locate and install the ``Bazel`` plugin. You will be
prompted to restart CLion.

To use Drake in CLion you **must** use Drake's bazel wrapper.
Open ``Settings > Bazel Settings``.  For ``Bazel binary location`` select the
path to ``drake/tools/clion/bazel_wrapper`` from any recent Drake source tree
(it doesn't have to match the current project open in CLion).

Downgrading the Bazel Plugin
----------------------------
These instructions were tested with CLion 2018.3.4.

1. Goto https://plugins.jetbrains.com/plugin/9554-bazel to download an older
   version of Bazel Plugin to get the file ``clwb_bazel.zip``.

2. In CLion, go to ``Install Plugin from Disk...`` by:

    a. Select ``File > Settings``.
    b. Select ``Plugins``.
    c. Click on the "gear" ⛭ icon to get a pop-up menu.
    d. Select ``Install Plugin from Disk...`` and then choose the file
       ``clwb_bazel.zip``.

Setting up Drake in CLion
-------------------------
CLion will invoke Bazel to build Drake, including the external dependencies
specified in the WORKSPACE file.

1. ``File > Import Bazel Project``
2. Select Workspace: Use an existing Bazel workspace, and provide the path to
   your ``drake`` directory.
3. Select Project View: choose "Import project view file", and
   select the file ``drake/.bazelproject``
4. Project View: Pick a ``project data directory`` of your choice for the
   CLion project files. It must not be a subdirectory of ``drake``.
5. (Advanced) Project View: If you only wish to develop a subset of Drake,
   you can specify only those files and targets in the project view file.
   Most users should leave it as-is.
6. Click "Finish".  CLion will begin ingesting the Drake source, building
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
<https://docs.bazel.build/versions/master/user-manual.html#target-patterns>`
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
include ``bazel-drake``, which is a Bazel-internal detail. Bazel edits
the contents of that directory for its own purposes, and those changes will
spuriously appear in the CLion UI as changes that need to be committed. To make
CLion ignore ``bazel-drake``, enable Git integration under the ``VCS``
tab, then go to ``File > Settings``. Select the ``Version Control`` menu item
directly (not one of the subtopics displayed when that item is expanded). You
will see a list of all the Git root directories. Look for ``bazel-drake``
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
per-tool details below. The GUI description applies to version 2018.1.6 and
may be slightly different in previous versions.

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

First, make sure you have installed ``clang-format-6.0``
(see :doc:`code_style_tools`).

Clang format selected file
^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Clang Format Full File``
  :Description: ``Apply clang-format to the active file``
  :Program: ``clang-format-6.0``
  :Arguments: ``-i $FileName$``
  :Working directory: ``$FileDir$``
  :Advanced Options: Uncheck ``Open console for tool output``

Leave the checkbox options in their default state.

Clang format selected lines
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Clang Format Selected Lines``
  :Description: ``Apply clang-format to the selected lines``
  :Program: ``clang-format-6.0``
  :Arguments: ``-lines $SelectionStartLine$:$SelectionEndLine$ -i $FileName$``
  :Working directory: ``$FileDir$``
  :Advanced Options: Uncheck ``Open console for tool output``

Leave the checkbox options in their default state.

Correct #include ordering
^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

  :Name: ``Clang Format Include Ordering``
  :Description: ``Runs the clang format for correcting includes on the current
                  file``
  :Program: ``bazel``
  :Arguments: ``run //tools/lint:clang-format-includes -- $FilePath$``
  :Working directory: ``$Projectpath$``
  :Advanced Options: Uncheck ``Open console for tool output``

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
converted into clickable links so that you can click on a message and be taken
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
  :Arguments: ``run @styleguide//:cpplint -- --output=eclipse
                 $FilePath$``
  :Working directory: ``$Projectpath$``
  :Advanced Options: Confirm ``Open console for tool output`` is checked

To configure the clickable links, enter the following string in the ``Advanced
Options`` > ``Output filters`` window:

    ``$FILE_PATH$:$LINE$``

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
  :Arguments: ``run //tools/lint:drakelint -- $FilePath$``
  :Working directory: ``$Projectpath$``
  :Advanced Options: Confirm ``Open console for tool output`` is checked

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
  :Arguments: ``--output=eclipse $FilePath$``

Building the google styleguide lint tool:

``bazel build @styleguide//:cpplint``

Drake style addenda
"""""""""""""""""""

Change the following fields in the instructions given above:

  :Program: ``bazel-bin/tools/lint/drakelint``
  :Arguments: ``$FilePath$``

Building the drake addenda lint tool:

``bazel build //tools/lint:drakelint``
