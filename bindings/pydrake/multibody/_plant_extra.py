import textwrap as _textwrap
import sys as _sys

import pydrake.autodiffutils as _ad
import pydrake.symbolic as _sym
from pydrake.common import cpp_param as _cpp_param
from pydrake.common import cpp_template as _cpp_template
from pydrake.common.deprecation import _warn_deprecated
from pydrake.common.value import Value as _Value

_PARAM_LIST = (
    (float,),
    (_ad.AutoDiffXd,),
    (_sym.Expression,),
)

_deprecation_msg = """
``VectorExternallyAppliedSpatialForced`` and
``Value[VectorExternallyAppliedSpatialForced]`` are deprecated. Please use
``list()`` and ``Value[List[ExternallyAppliedSpatialForce]]`` instead.
The deprecated code will be removed from Drake on or after 2020-09-01.
""".strip()


@_cpp_template.TemplateClass.define(
    "VectorExternallyAppliedSpatialForced_", _PARAM_LIST,
    # N.B. We must pass `scope` here for this code which is executed via a
    # Python C extension.
    scope=_sys.modules[__name__])
def VectorExternallyAppliedSpatialForced_(param):

    class Impl(list):
        def __new__(cls, *args, **kwargs):
            _warn_deprecated(_deprecation_msg)
            return list(*args, **kwargs)

    # N.B. For whatever reason, Sphinx / Python docstrings work best in Sphinx
    # when using a multiline string.
    Impl.__doc__ = f"""
    Warning:
        {_textwrap.indent(_deprecation_msg, ' '*8).lstrip()}
    """
    return Impl


def _deprecate_vector_instantiations():
    for param in _PARAM_LIST:
        # Deprecate old type.
        cls, _ = VectorExternallyAppliedSpatialForced_.deprecate_instantiation(
            param, _deprecation_msg)
        # Add deprecated alias to Value[] using the correct new class.
        correct_cls = _Value[
            _cpp_param.List[ExternallyAppliedSpatialForce_[param]]]
        _Value.add_instantiation(cls, correct_cls)
        _Value.deprecate_instantiation(cls, _deprecation_msg)


# Default instantiation.
# N.B. This will not trigger a warning if a user constructs an instance of this
# class.
VectorExternallyAppliedSpatialForced = (
    VectorExternallyAppliedSpatialForced_[None])

_deprecate_vector_instantiations()
