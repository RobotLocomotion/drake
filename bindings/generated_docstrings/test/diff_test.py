from pathlib import Path
import sys

from python import runfiles


def _read_resource(resource_path: Path):
    manifest = runfiles.Create()
    path = Path(manifest.Rlocation(str(resource_path)))
    return path.read_text(encoding="utf-8")


def main():
    if sys.version_info[:2] == (3, 10):
        # The libclang on jammy gives different docstring output.
        print("WARNING: skipping test on Ubuntu 22.04 Jammy")
        return 0
    filenames = sys.argv[1:]
    assert len(filenames) > 0
    expected_dir = Path("drake/bindings/generated_docstrings/gen")
    actual_dir = Path("drake/bindings/generated_docstrings")
    for filename in filenames:
        expected_text = _read_resource(expected_dir / filename)
        actual_text = _read_resource(actual_dir / filename)
        if actual_text != expected_text:
            print("ERROR: generated docstrings need regeneration; run:")
            print("  bazel run //bindings/generated_docstrings:regenerate")
            return 1
    return 0


assert __name__ == "__main__"
sys.exit(main())
