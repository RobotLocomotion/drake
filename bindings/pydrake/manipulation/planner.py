"""Deprecated aliases to planning resources."""

from pydrake.common.deprecation import _warn_deprecated

from pydrake.multibody.inverse_kinematics import (
    DifferentialInverseKinematicsIntegrator,
    DifferentialInverseKinematicsStatus,
    DifferentialInverseKinematicsParameters,
    DifferentialInverseKinematicsResult,
    DoDifferentialInverseKinematics,
)

_warn_deprecated(
    "Please import the differential ik API from "
    "pydrake.multibody.inverse_kinematics instead of the deprecated "
    f"{__name__} submodule.",
    date="2023-06-01", stacklevel=3)
