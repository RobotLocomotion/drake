"""A helper tool for cc_hidden.bzl that copies macOS archives, demoting the
symbol visibility while doing so using `nmedit` to mark everything hidden.
"""

import argparse
from pathlib import Path
import subprocess
import tempfile


def _rewrite(*, input: Path, output: Path, temp: Path):
    """Given input and output paths to `*.a` library files (and a temporary
    directory scratch path), copies the input to output while demoting symbol
    visibility.
    """
    # Create two scratch directories.
    input_dir = temp / "input"
    input_dir.mkdir()
    output_dir = temp / "output"
    output_dir.mkdir()

    # Extract the input archive.
    subprocess.run(
        ["/usr/bin/ar", "-x", input],
        cwd=input_dir,
        check=True,
    )
    objs = [x.relative_to(input_dir) for x in input_dir.glob("**/*.o")]
    assert objs

    # Copy objects to the output archive, changing them to be hidden.
    for x in objs:
        subprocess.run(
            [
                "/usr/bin/nmedit",
                "-s",
                "/dev/null",
                "-p",
                input_dir / x,
                "-o",
                output_dir / x,
            ],
            check=True,
        )

    # Repack the output archive.
    subprocess.run(
        ["/usr/bin/ar", "-q", output] + objs,
        cwd=output_dir,
        check=True,
    )


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", metavar="FILE", type=Path, required=True)
    parser.add_argument("--output", metavar="FILE", type=Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        _rewrite(
            input=args.input.absolute(),
            output=args.output.absolute(),
            temp=Path(temp).absolute(),
        )


assert __name__ == "__main__"
_main()
