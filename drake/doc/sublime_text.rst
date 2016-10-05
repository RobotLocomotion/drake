.. _sublime_text_notes:

*****************************************
Sublime Text Notes
*****************************************

.. contents:: `Table of contents`
   :depth: 3
   :local:

.. _sublime-intro:

Introduction
============

This page contains notes on how to use
`Sublime Text <https://www.sublimetext.com>`_ with Drake.

Packages
========

This subsection lists various `packages <https://packagecontrol.io/>`_ that we
recommend you install.

CMake
-----

https://packagecontrol.io/packages/CMake

cpplint
-------

https://packagecontrol.io/packages/SublimeLinter-cpplint

Trailing Spaces
---------------

https://packagecontrol.io/packages/TrailingSpaces

User Preferences
================

To change your custom user preferences, go to "Sublime Text", "Preferences",
"Settings - User." Then add the following:

Full File Path Display
----------------------
Note that this only needs to be done on OS X since the full path appears by
default on Linux machines.

To display the current file's full path in the title bar on OSX::

    "show_full_path": true,

Show \*.sdf Files in Side Bar
-----------------------------

To show ``*.sdf`` files in the side bar::

    "file_exclude_patterns":
    [
        "*.pyc",
        "*.pyo",
        "*.exe",
        "*.dll",
        "*.obj",
        "*.o",
        "*.a",
        "*.lib",
        "*.so",
        "*.dylib",
        "*.ncb",
        "*.suo",
        "*.pdb",
        "*.idb",
        ".DS_Store",
        "*.class",
        "*.psd",
        "*.db",
        "*.sublime-workspace"
    ],

Note that the above list does not include ``*.sdf``. This is expected since by
default ``*.sdf`` does show up in the list.

Automatically Show an 80-character Ruler
----------------------------------------

To view an 80 character-wide ruler::

    "rulers": [80],

Always displaying a ruler is useful to conform to
:ref:`Drake's coding style <code-style-guide>`.
