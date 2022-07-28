"""When used during unit testing, this module disables scipy -- as if it wasn't
installed. This helps developers check what happens for users who don't have
scipy, even if the developer does have scipy installed system-wide.
"""

raise ModuleNotFoundError("No module named 'scipy'")
