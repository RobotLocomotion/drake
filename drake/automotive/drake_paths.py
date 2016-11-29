#!/usr/bin/env python

"""Find and export useful Drake paths.
"""

import os
import sys

_THIS_FILE = os.path.abspath(__file__)
_THIS_DIR = os.path.dirname(_THIS_FILE)
DRAKE_DIR = os.path.dirname(_THIS_DIR)
DRAKE_DIST_DIR = os.path.dirname(DRAKE_DIR)
DRAKE_DIST_BUILD_DIR = os.getenv(
    'DRAKE_DIST_BUILD',
    os.path.join(DRAKE_DIST_DIR, 'build'))
DRAKE_INSTALL_BIN_DIR = os.path.join(DRAKE_DIST_BUILD_DIR, "install", "bin")

if not os.path.exists(DRAKE_INSTALL_BIN_DIR):
    raise RuntimeError(
        "cannot find DRAKE_DIST_BUILD_DIR at " + DRAKE_DIST_BUILD_DIR)

DRAKE_DRAKE_BUILD_DIR = os.path.join(DRAKE_DIST_BUILD_DIR, "drake")

DRAKE_LCMTYPES_DIR = os.path.join(
    DRAKE_DIST_BUILD_DIR, 'drake', 'lcmtypes')
DRAKE_DIST_PYTHON_INSTALL_DIR = os.path.join(
    DRAKE_DIST_BUILD_DIR, 'install', 'lib', 'python2.7')


def _add_path(apath):
    if apath not in sys.path:
        sys.path.append(apath)


def add_module_search_paths():
    _add_path(DRAKE_LCMTYPES_DIR)  # First, to pick up local edits to messages.
    _add_path(os.path.join(DRAKE_DIST_PYTHON_INSTALL_DIR, 'dist-packages'))
    _add_path(os.path.join(DRAKE_DIST_PYTHON_INSTALL_DIR, 'site-packages'))
