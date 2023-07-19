import re
import sys
from pathlib import Path

from bazel_tools.tools.python.runfiles import runfiles

sys.path.insert(0, str(Path(__file__).parent.parent / "image"))
from vtk_cmake_configure_args import cxx_std  # noqa: E402


def _rlocation(relative_path: str) -> Path:
    """Return the real path to ``drake/{relative_path}``."""
    manifest = runfiles.Create()
    resource_path = f"drake/{relative_path}"
    resolved_path = manifest.Rlocation(resource_path)
    assert resolved_path, f"Missing {resource_path}"
    return Path(resolved_path).resolve()


def parse_cxx_std_from_bazelrc(bazelrc_path: Path):
    """Load the bazelrc path, parse and return the C++ standard drake uses."""
    with open(bazelrc_path) as f:
        for line in f:
            m = re.match(
                r"^build --cxxopt=-std=c\+\+(?P<cxx_std>[0-9]{2})",
                line,
            )
            if m is not None:
                return m.group("cxx_std")

    raise RuntimeError(f"Could not find --cxxopt=-std=c++XY in {bazelrc_path}")


def assert_cxx_std_matches(codename: str):
    """Assert that the drake and VTK C++ standard for ``codename`` match."""
    assert codename in {
        "mac",
        "focal",
        "jammy",
    }, f"Unexpected codename={codename}"

    if codename == "mac":
        bazelrc_path = _rlocation("tools/macos.bazelrc")
    else:
        bazelrc_path = _rlocation(f"tools/ubuntu-{codename}.bazelrc")
    drake_cxx_std = parse_cxx_std_from_bazelrc(bazelrc_path)
    vtk_cxx_std = cxx_std(codename)
    assert vtk_cxx_std == drake_cxx_std, (
        f"C++ standard mismatch for {codename}: vtk_cxx_std={vtk_cxx_std}, "
        f"drake_cxx_std={drake_cxx_std} parsed from {bazelrc_path}."
    )


def main():
    assert_cxx_std_matches("mac")
    assert_cxx_std_matches("focal")
    assert_cxx_std_matches("jammy")


if __name__ == "__main__":
    main()
