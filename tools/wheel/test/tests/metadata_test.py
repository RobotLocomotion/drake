#!/usr/bin/env python

import importlib.metadata

version = importlib.metadata.version("drake")
assert isinstance(version, str), repr(version)
