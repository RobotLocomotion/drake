"""This (private) module is like pydrake.all, but with an even wider charter.

The pydrake.all module merges all of Drake's _stable_ modules (e.g., it does
not incorporate any of the `pydrake.examples.*`).

On the other hand, this module imports all of Drake's _public_ modules (which
are a superset of the stable modules). Importing all modules is a prerequisite
of generating our website docs, *.pyi type stub files, etc.

Anytime pydrake gains a new public module, that module needs to be added to
either pydrake.all or this file.

TODO(jwnimmer-tri) Nobody ever remembers to add their module to the lists.
We need some kind of automated cross-check to enforce the rule.
"""

import importlib

# Start with all of the stable modules.
importlib.__import__("pydrake.all")

# Add the remaining public modules.
importlib.__import__("pydrake.common.cpp_param")
importlib.__import__("pydrake.common.cpp_template")
importlib.__import__("pydrake.examples")
importlib.__import__("pydrake.examples.gym.play_cart_pole")
importlib.__import__("pydrake.examples.gym.train_cart_pole")
importlib.__import__("pydrake.gym")
importlib.__import__("pydrake.tutorials")
