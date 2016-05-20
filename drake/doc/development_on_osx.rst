*****************************************
Drake Development on OS X
*****************************************

This page contains information that may be useful to people developing in OS X.

*Note that the applications, tools, and libraries listed below are simply those that some have found useful. They should* **not** *be interpreted as mandatory.*

.. contents:: `Table of contents`
   :depth: 2
   :local:


pygame
======

`pygame <http://pygame.org>`_ is used by ``drake/examples/Cars/SteeringCommandDriver.py``.

Install Mercurial:

    brew install mercurial

Install pygame:

    pip install hg+http://bitbucket.org/pygame/pygame

Ensure your ``PYTHONPATH`` environment variable includes Drake's site-package and dist-packages directories:

    export PYTHONPATH=$PYTHONPATH:[DRAKE_ROOT]/build/lib/python2.7/site-packages:[DRAKE_ROOT]/build/lib/python2.7/dist-packages



git Tools
=========

gitk
----

The version of `git <https://git-scm.com>`_ that comes with OS X may not include `gitk <https://git-scm.com/docs/gitk>`_, a GUI-based git repository browser. A workaround is to get a newer version using brew:

    brew update
    brew install git

If gitk does not start with an ``unknown color name "lime"`` error, upgrade your version of Tcl/Tk:

    brew cask install tcl

References:

1. http://stackoverflow.com/questions/34637896/gitk-will-not-start-on-mac-unknown-color-name-lime
2. http://stackoverflow.com/questions/17582685/install-gitk-on-mac

SourceTree
-----------

`SourceTree <https://www.sourcetreeapp.com>`_ is a free git and mercurial client for Windows and Mac made by Atlassian.

(*Notes comming soon*)


Profiling
=========

A few of us have used the xcode tool "Instruments" (which should already be installed on your system) with considerable success, but we do not have much experience with it yet.

