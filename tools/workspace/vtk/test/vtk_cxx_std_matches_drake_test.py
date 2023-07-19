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
    cxx_std_re = re.compile(r"^build --cxxopt=-std=c\+\+(?P<cxx_std>[0-9]{2})")
    with open(bazelrc_path) as f:
        for line in f:
            m = cxx_std_re.match(line)
            if m is not None:
                return m.group("cxx_std")

    raise RuntimeError(f"Could not find --cxxopt=-std=c++XY in {bazelrc_path}")


def main():
    # TODO(svenevs): paths returned are in the sandbox, which is invalid.
    # I could not figure out how to make these data = [] available either.
    macos_bazelrc_path = _rlocation("tools/macos.bazelrc")
    macos_cxx_std = parse_cxx_std_from_bazelrc(macos_bazelrc_path)
    assert macos_cxx_std == cxx_std("mac")

    focal_bazelrc_path = _rlocation("tools/ubuntu-focal.bazelrc")
    focal_cxx_std = parse_cxx_std_from_bazelrc(focal_bazelrc_path)
    assert focal_cxx_std == cxx_std("focal")

    jammy_bazelrc_path = _rlocation("tools/ubuntu-jammy.bazelrc")
    jammy_cxx_std = parse_cxx_std_from_bazelrc(jammy_bazelrc_path)
    assert jammy_cxx_std == cxx_std("jammy")


if __name__ == "__main__":
    main()
