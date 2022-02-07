# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import functools as _functools

from pydrake.autodiffutils import AutoDiffXd as _AD
from pydrake.common.deprecation import deprecated_callable as _deprecated


@_deprecated(
    "pydrake.multibody.plant.ContactResultsToMeshcatParams has been renamed to"
    " pydrake.multibody.meshcat.ContactVisualizerParams."
    " The old name is deprecated.",
    date="2022-05-01")
def ContactResultsToMeshcatParams(*args, **kwargs):
    from pydrake.multibody.meshcat import ContactVisualizerParams as real
    return real(*args, **kwargs)


@_deprecated(
    "pydrake.multibody.plant.ContactResultsToMeshcat has been renamed to"
    " pydrake.multibody.meshcat.ContactVisualizer."
    " The old name is deprecated.",
    date="2022-05-01")
def ContactResultsToMeshcat(T=float, *args, **kwargs):
    from pydrake.multibody.meshcat import ContactVisualizer_ as real
    return real[T](*args, **kwargs)


ContactResultsToMeshcat_ = dict((
    (float, ContactResultsToMeshcat),
    (_AD, _functools.partial(ContactResultsToMeshcat, T=_AD)),
))


@_deprecated(
    "pydrake.multibody.plant.ContactResultsToMeshcat has been renamed to"
    " pydrake.multibody.meshcat.ContactVisualizer."
    " The old name is deprecated.",
    date="2022-05-01")
def _AddToBuilder(T=float, *args, **kwargs):
    from pydrake.multibody.meshcat import ContactVisualizer_ as real
    return real[T].AddToBuilder(*args, **kwargs)


ContactResultsToMeshcat.AddToBuilder = _AddToBuilder
ContactResultsToMeshcat_[float].AddToBuilder = _AddToBuilder
ContactResultsToMeshcat_[_AD].AddToBuilder = _functools.partial(
    _AddToBuilder, T=_AD)
