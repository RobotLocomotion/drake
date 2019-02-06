# Only try to import attic symbols if they're available.
try:
    from pydrake.attic.systems.sensors import *
except ImportError:
    pass
