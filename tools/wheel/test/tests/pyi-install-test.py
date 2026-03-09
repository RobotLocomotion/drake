#!/usr/bin/env python

import os

import pydrake


def assert_exists(path):
    assert os.path.exists(path), f'{path!r} does not exist!'
    print(path)

# Check that type information files are present.
pydrake_dir = pydrake.__path__[0]
assert_exists(os.path.join(pydrake_dir, 'py.typed'))
assert_exists(os.path.join(pydrake_dir, 'lcm.pyi'))
