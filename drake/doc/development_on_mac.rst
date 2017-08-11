************************
Drake Development on Mac
************************

This page contains information that may be useful to people developing on macOS / OS X.

*Note that the applications, tools, and libraries listed below are simply those that some have found useful. They should* **not** *be interpreted as mandatory.*

.. contents:: `Table of contents`
   :depth: 2
   :local:


Git Tools
=========

gitk
----

The version of `Git <https://git-scm.com>`_ that comes with Xcode may not include `gitk <https://git-scm.com/docs/gitk>`_, a GUI-based Git repository browser. A workaround is to get a newer version using Homebrew:

    brew update
    brew install git

If gitk does not start with an ``unknown color name "lime"`` error, upgrade your version of Tcl/Tk:

    brew cask install tcl

References:

1. http://stackoverflow.com/questions/34637896/gitk-will-not-start-on-mac-unknown-color-name-lime
2. http://stackoverflow.com/questions/17582685/install-gitk-on-mac

SourceTree
-----------

`SourceTree <https://www.sourcetreeapp.com>`_ is a free Git and Mercurial client for Windows and Mac made by Atlassian.

(*Notes coming soon*)


Profiling
=========

A few of us have used the Xcode tool "Instruments" (which should already be installed on your system) with considerable success, but we do not have much experience with it yet.
