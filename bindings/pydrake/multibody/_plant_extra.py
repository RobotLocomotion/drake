# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import functools as _functools

from pydrake.autodiffutils import AutoDiffXd as _AD
from pydrake.common.deprecation import deprecated_callable as _deprecated


@_deprecated("Spell as ContactVisualizerParams instead", date="2022-05-01")
def ContactResultsToMeshcatParams(*args, **kwargs):
    from pydrake.multibody.meshcat import ContactVisualizerParams as real
    return real(*args, **kwargs)


@_deprecated("Spell as ContactVisualizer instead", date="2022-05-01")
def ContactResultsToMeshcat(T=float, *args, **kwargs):
    from pydrake.multibody.meshcat import ContactVisualizer_ as real
    return real[T](*args, **kwargs)


ContactResultsToMeshcat_ = dict((
    (float, ContactResultsToMeshcat),
    (_AD, _functools.partial(ContactResultsToMeshcat, T=_AD)),
))
