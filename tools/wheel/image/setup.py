import glob
import os

from setuptools import Distribution, find_packages, setup


class BinaryDistribution(Distribution):
    """Force the wheel builder to output a platform-specific wheel tag."""

    def has_ext_modules(self) -> bool:
        return True


DRAKE_VERSION = os.environ.get("DRAKE_VERSION", "0.0.0")

DRAKE_IS_ABI3_WHEEL = bool(int(os.environ["DRAKE_IS_ABI3_WHEEL"]))

# Required python packages that will be pip installed along with pydrake
python_required = [
    "matplotlib",
    "numpy",
    "pydot",
    "PyYAML",
    # MOSEK's published wheels declare an upper bound on their supported Python
    # version, which is currently Python < 3.15. When that changes to a larger
    # version number, we should bump this up to match, and also grep tools/wheel
    # for other mentions of MOSEK version bounds and fix those as well. Further,
    # if this version number is lower than the maximum version for which Drake
    # builds wheels (see tools/wheel/wheel_builder/macos.py and
    # tools/wheel/wheel_builder/linux.py), then that should be documented for
    # users accordingly.
    # Additionally, MOSEK is not supported on Linux aarch64. (Apple Silicon
    # is spelled 'arm64', so this doesn't apply there.)
    'Mosek==11.1.2 ; python_version < "3.15" and platform_machine != "aarch64"',
]


def _find_data_files(*patterns: str) -> list[str]:
    result = []
    for pattern in patterns:
        result += [
            f"../{f}"
            for f in glob.iglob(pattern, recursive=True, include_hidden=True)
        ]
    return result


def _actually_find_packages() -> list[str]:
    """Find additional `pydrake` modules intended to be packaged in the wheel
    which aren't found by `setuptools` due to a missing `__init__.py` file."""
    result = find_packages()
    result.extend(
        [
            "pydrake.autodiffutils",
            "pydrake.common",
            "pydrake.examples",
            "pydrake.geometry",
            "pydrake.manipulation",
            "pydrake.math",
            "pydrake.solvers",
            "pydrake.symbolic",
            "pydrake.visualization",
        ]
    )
    print(f"Using packages={result}")
    return result


setup(
    name="drake",
    version=DRAKE_VERSION,
    description="Model-based design and verification for robotics",
    long_description="""
Drake ("dragon" in Middle English) is a toolbox started by the Robot Locomotion
Group at the MIT Computer Science and Artificial Intelligence Lab (CSAIL).
The development team has now grown significantly, with core development led by
the Toyota Research Institute.
It is a collection of tools for analyzing the dynamics of our robots and
building control systems for them, with a heavy emphasis on optimization-based
design/analysis.

See https://drake.mit.edu/pip.html for installation instructions and caveats.
""".strip(),
    url="https://drake.mit.edu",
    author="Drake Development Team",
    author_email="drake-users@mit.edu",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Environment :: Console",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: BSD License",
        "License :: Other/Proprietary License",
        "Operating System :: MacOS",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3 :: Only",
        "Programming Language :: Python :: Implementation :: CPython",
        "Topic :: Scientific/Engineering",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    license="Various",
    platforms=["linux_x86_64", "macosx_arm64"],
    packages=_actually_find_packages(),
    # Add in any packaged data.
    include_package_data=True,
    package_data={
        "": _find_data_files(
            "pydrake/py.typed",
            "pydrake/**/*.pyi",
            "pydrake/**/*.so",
            "pydrake/lib/**",
            "pydrake/doc/**",
            "pydrake/share/**",
            "pydrake/INSTALLATION",
        )
    },
    python_requires=">=3.12",
    options=(
        {
            # This matches the Py_LIMITED_API pin at //tools/workspace/nanobind.
            "bdist_wheel": {"py_limited_api": "cp312"},
        }
        if DRAKE_IS_ABI3_WHEEL
        else {}
    ),
    install_requires=python_required,
    distclass=BinaryDistribution,
    zip_safe=False,
)
