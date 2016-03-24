*****************************************
Drake Development on OS X
*****************************************

This page contains software development notes that are specific to OS X.

Running gitk
============

The version of `git <https://git-scm.com>`_ that comes with OS X may not include `gitk <https://git-scm.com/docs/gitk>`_, a GUI-based git repository browser. We recommend getting a newer version using brew:

    brew update
    brew install git

If gitk does not start with an ``unknown color name "lime"`` error, upgrade your vesrion of Tcl/Tk:

    brew cask install tcl

References:

1. http://stackoverflow.com/questions/34637896/gitk-will-not-start-on-mac-unknown-color-name-lime
2. http://stackoverflow.com/questions/17582685/install-gitk-on-mac