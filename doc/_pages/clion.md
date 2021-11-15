---
title: CLion IDE setup
---

<div class="note" markdown="1">
EVERY SETUP STEP IN THIS DOCUMENT IS CRITICAL TO GET CLION WORKING PROPERLY.
Read carefully, and do not skip anything.
</div>

<div class="note" markdown="1">
The Bazel plugin for CLion does not support macOS, per
<https://github.com/bazelbuild/intellij/issues/109>
</div>

This guide describes how to set up Drake in the JetBrains CLion IDE on Ubuntu.

# Using CLion with Bazel

First, you **must** install Bazel and build Drake with Bazel, following
[the Drake Bazel instructions](/bazel.html).

To use Drake with CLion, your Drake checkout **must** be named ``drake``.

## Installing CLion


1. Go to [https://www.jetbrains.com/clion/download/](https://www.jetbrains.com/clion/download/). Look for "Other versions"
   and download the appropriate version of CLion (see below).
2. Install CLion. Exact steps depend on your platform, but it's
   straightforward. Just using defaults for everything is fine. You now have a
   30-day trial version of CLion. Either try it out as is, or get a free
   academic license [here](https://www.jetbrains.com/shop/eform/students).

The most recent versions that we have tested for compatibility are:
* Ubuntu 20.04 (Focal)
* Bazel 4.2.1 (2021-08-30)
* CLion 2021.2.3 (2021-10-14)
    * Bazel plugin 2021.11.03.1.1 (2021-11-08)

Many versions of the above (Bazel / CLion / Bazel plugin) are *not* compatible
with each other.  We strongly suggest using only the versions shown above, when
working with Drake.

## Upgrading CLion

Users upgrading from a previous version of CLion should do the following:

1. To have your Unity launcher CLion icon point to the correct version,
   run ``locate jetbrains-clion.desktop`` and edit the located file. If more
   than one file is located, you may want to consolidate to a single launch file
   in your user directory, typically ``~/.local/share/applications``.
2. Uninstall the previous version of the Bazel plugin and update to the version
   above (or the most recent version compatible with the CLion version you
   are upgrading to).
   See [Installing the Bazel Plugin](#installing-the-bazel-plugin).
3. In CLion, select
   ``Bazel->Sync->Non-incrementally Sync Project with BUILD Files``
4. You may need to delete cached data in ``~/.cache/bazel`` and
   ``~/.cache/JetBrains/CLion2021.2/caches`` if you get error messages
   complaining about old files in the cache (substitute the right version
   number).

**Note**: It is not necessary to import your project into a *new* CLion project.
Overwriting the old project is appropriate.

## Installing the Bazel Plugin

To use Bazel in CLion, you **must** install a plugin supplied by Google.  To
install the plugin, open ``Settings`` (either ``Welcome > Configure >
Settings`` or ``File > Settings``), select ``Plugins``, then choose the
``Marketplace`` tab.  Locate and install the ``Bazel`` plugin. You will be
prompted to restart CLion.

To use Drake in CLion you **must** use Drake's bazel wrapper.
Open ``Settings > Bazel Settings``.  For ``Bazel binary location`` select the
path to ``drake/tools/clion/bazel_wrapper`` from any recent Drake source tree
(it doesn't have to match the current project open in CLion).

## Setting up Drake in CLion

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

## Building and Running Targets

To build all of Drake with default Bazel options, select
``Bazel > Build > Compile Project``.

To build or run a specific target go to ``Run > Edit Configurations``. Click
``+`` to create a new Bazel command.  Specify the configuration name and Bazel
options. The ``Target expression`` specifies the actual code (library, binary,
and/or test) that you want to run. To learn more about target expressions, see
[the Bazel manual](https://docs.bazel.build/versions/master/user-manual.html#target-patterns).
Once you've created a configuration, you can launch it from the ``Run`` menu.

To run a specific target in the debugger, create a configuration as above,
using the ``bazel run`` command. Then launch it from ``Run > Debug``.

## Keeping CLion Up-to-Date with the Bazel Build

Changes to BUILD files can add or remove source files from the Bazel build.
To propagate those changes into the CLion project structure, select
``Bazel > Sync Project With BUILD Files``.

## Git Integration

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

# Integrating External Tools with CLion

CLion provides a mechanism for invoking external binaries/scripts/etc. with
parameters derived from the CLion GUI. Below, we outline a number of common
tools to aid with compliance with the Drake style guide. The work to create
a new external tool is the same in all cases; only the specific tool settings
differ from tool to tool. We'll outline the general work here and provide
per-tool details below. The GUI description applies to version 2018.1.6 and
may be slightly different in previous versions.

1. Open the Settings dialog (``File`` > ``Settings``) or ``Alt+Ctrl+S``.
2. Navigate to ``Tools`` > ``External Tools``.
3. Click the {::nomarkdown}<font color="green">+</font>{:/} sign to add a new
   tool.
4. Set the appropriate fields in the ``Edit Tool``. See the following tools for
   details.
5. Click ``Ok``.

There are several ways to use an *External Tool*. One is to right-click on a
file and select ``External Tools`` > ``Tool Name``. Another is to select
``Tools`` > ``External Tools`` > ``Tool Name``. For tools that operate on a
selected file, make sure that file is "active" by clicking on it. The
``Tool Name`` will be the value set in the ``Name`` field outlined below.

## Formatting files

You can use clang format to modify the formatting of your file in the GUI. We'll
introduce three variants:

* Apply clang-format to a whole file.
* Apply clang-format to selected lines.
* Apply clang-format to correct ``#include`` ordering.

These tools modify the selected file. There is a synchronization issue with
CLion such that the modification may not be immediately apparent. When in doubt,
select away from the target file and back; this will cause the file to refresh
and you can confirm that the file has been modified as expected.

First, make sure you have installed ``clang-format-9``
(see [Tools for Code Style Compliance](/code_style_tools.html)).

### Clang format selected file

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

* **Name:** ``Clang Format Full File``
* **Description:** ``Apply clang-format to the active file``
* **Program:** ``clang-format-9``
* **Arguments:** ``-i $FileName$``
* **Working directory:** ``$FileDir$``
* **Advanced Options:** Uncheck ``Open console for tool output``

Leave the checkbox options in their default state.

### Clang format selected lines

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

* **Name:** ``Clang Format Selected Lines``
* **Description:** ``Apply clang-format to the selected lines``
* **Program:** ``clang-format-9``
* **Arguments:** ``-lines $SelectionStartLine$:$SelectionEndLine$ -i $FileName$``
* **Working directory:** ``$FileDir$``
* **Advanced Options:** Uncheck ``Open console for tool output``

Leave the checkbox options in their default state.

### Correct #include ordering

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

* **Name:** ``Clang Format Include Ordering``
* **Description:** ``Runs the clang format for correcting includes on the current
                  file``
* **Program:** ``bazel``
* **Arguments:** ``run //tools/lint:clang-format-includes -- $FilePath$``
* **Working directory:** ``$Projectpath$``
* **Advanced Options:** Uncheck ``Open console for tool output``

Leave the checkbox options in their default state.

## "Linting" files


"Linting" refers to using tools to find aspects of code which don't conform
to specified coding practices. You can apply Drake's linting tools in CLion to
find such issues. We'll define two tools:

* General linting (via cpplint) which captures most of the Drake style guide.
* Drake extended linting which captures aspects of the Drake style guide _not_
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

### Lint selected file for google style guide

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

* **Name:** ``Cpplint File``
* **Description:** ``Apply cpplint to the current file``
* **Program:** ``bazel``
* **Arguments:** ``run @styleguide//:cpplint -- --output=eclipse
                 $FilePath$``
* **Working directory:** ``$Projectpath$``
* **Advanced Options:** Confirm ``Open console for tool output`` is checked

To configure the clickable links, enter the following string in the ``Advanced
Options`` > ``Output filters`` window:

&nbsp;&nbsp;&nbsp;
``$FILE_PATH$:$LINE$``

### Lint selected file for Drake style addenda

This tool is a supplement to the google style cpplint. It tests for additional
style requirements which are otherwise missed by the general tool. The primary
reason to run this is to confirm that the order of the ``#include`` statements
is correct.

Open the ``Edit Tool`` for external tools as outlined above and enter the
following values for the fields:

* **Name:** ``Drake Lint File``
* **Description:** ``Apply drake lint to the current file``
* **Program:** ``bazel``
* **Arguments:** ``run //tools/lint:drakelint -- $FilePath$``
* **Working directory:** ``$Projectpath$``
* **Advanced Options:** Confirm ``Open console for tool output`` is checked

In the event of finding a lint problem (e.g., out-of-order include files), the
CLion output will contain a *single* clickable link. This link is only the
*first* error encountered in the include section; there may be more. The link
merely provides a hint to the developer to see the problem area. Rather than
fixing by hand, we strongly recommend executing the ``Clang Format Include
Ordering`` external tool on the file.

### Alternative linting configuration

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

**Google style guide linting**

Change the following fields in the instructions given above:

* **Program:** ``bazel-bin/external/styleguide/cpplint_binary``
* **Arguments:** ``--output=eclipse $FilePath$``

Building the google styleguide lint tool:

``bazel build @styleguide//:cpplint``

**Drake style addenda**

Change the following fields in the instructions given above:

* **Program:** ``bazel-bin/tools/lint/drakelint``
* **Arguments:** ``$FilePath$``

Building the drake addenda lint tool:

``bazel build //tools/lint:drakelint``

# Debugging an arbitrary program in Drake with CLion

Apparently CLion (or perhaps the Bazel plugin) has a certain amount of
auto-configuration of run/debug targets. It appears to hinge on the presence of
the gtest.h header in source files. This is convenient, but only further
mystifies the process of debugging a non-gtest program. This section explains
how to configure debugging support for arbitrary programs in a Drake/CLion
project.

This section assumes all of the Drake-recommended installation and
configuration is done.

## Get the bazel target string


Find the source file of the program in the file tree view. Right-click on the
file, and select "Copy BUILD Target String". This will put the Bazel target
name into the clipboard.

## Start a run configuration


From the top menu, select "Run/Edit Configurations...". Select the "+" at the
upper left of the dialog to add a new configuration. From the list, select
"Bazel Command".

## Fill in the configuration


Now it's time to fill in the new blank configuration. Give it a name, then
select the "+" at the right side to add the target expression. Once the edit
box appears, paste the contents of the clipboard there. Hit "Enter" or "Tab" to
confirm the setting; a port number value should appear in the "Port number"
field below. In "Bazel command", select either "run" (for an arbitrary
program), or "test" (for a Bazel test target). Everything else can be left at
default values. Click OK to finish.

## Launch the debugger

At this point, the top menu "Run" should have entries to run or debug the new
configuration. Select the debug entry there, or use the controls at the upper
right to launch the debugger.
