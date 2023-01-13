import pydrake.autodiffutils as _ad
from pydrake.common import _pretty_tparam
import pydrake.symbolic as _sym


def _indented_repr(o):
    """Returns repr(o), with any lines beyond the first one indented +2."""
    return repr(o).replace("\n", "\n  ")


def _spatial_vector_repr(rotation_name, translation_name):

    def repr_with_closure(self):
        rotation = self.rotational().tolist()
        translation = self.translational().tolist()
        return (
            f"{_pretty_tparam(type(self))}(\n"
            f"  {rotation_name}={_indented_repr(rotation)},\n"
            f"  {translation_name}={_indented_repr(translation)},\n"
            f")")

    return repr_with_closure


def _add_repr_functions():
    for T in [float, _ad.AutoDiffXd, _sym.Expression]:
        SpatialVelocity_[T].__repr__ = _spatial_vector_repr("w", "v")
        SpatialMomentum_[T].__repr__ = _spatial_vector_repr("h", "l")
        SpatialAcceleration_[T].__repr__ = _spatial_vector_repr("alpha", "a")
        SpatialForce_[T].__repr__ = _spatial_vector_repr("tau", "f")


_add_repr_functions()
