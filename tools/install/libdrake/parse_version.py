"""Parse the version stamp file and produce a CMake cache-style script file
which specifies the variable substitutions needed for drake-config.cmake."""

import argparse
import re

VERSION_TAG = "STABLE_VERSION"


# Check if a version string conforms to PEP 440.
def _check_version(version):
    return (
        re.match(
            r"^([1-9][0-9]*!)?(0|[1-9][0-9]*)"
            r"(\.(0|[1-9][0-9]*))*((a|b|rc)(0|[1-9][0-9]*))?"
            r"(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?"
            r"([+][a-z0-9]+([-_\.][a-z0-9]+)*)?$",
            version,
        )
        is not None
    )


# Validate a full version identifier and split it into four integer parts,
# padding to four. Any pre-release, 'dev', 'post', and/or local identifier (the
# portion following a '+') is discarded. Raises ValueError if the version is
# invalid or carries an (unsupported) epoch.
def _split_version(version_full):
    if not _check_version(version_full):
        raise ValueError(f"Version {version_full} is not valid")
    if re.match(r"^[1-9][0-9]*!", version_full):
        raise ValueError(
            f"Version {version_full} contains an epoch,"
            " which is not supported at this time"
        )

    m = re.match(r"^[0-9.]+", version_full)
    assert m

    # Note: user and continuous builds may have more than three parts.
    version_parts = m.group(0).split(".")
    if len(version_parts) < 4:
        if len(version_parts) == 3:
            version_parts.append(0)
        else:
            raise ValueError(
                f"Version {version_full} does not have enough parts"
            )

    return tuple(map(int, version_parts))


# Extract full version and version parts from a version stamp file. If a
# version is specified, the input file should contain a line starting with
# 'STABLE_VERSION', which should be three space-separated words; the tag, the
# full version, and the git SHA. If version information is not found, this
# returns (None, None).
def _parse_stamp(stamp_file):
    for line in stamp_file:
        if line.startswith(VERSION_TAG):
            tag, version_full, _git_sha = line.strip().split()
            assert tag == VERSION_TAG
            return version_full, _split_version(version_full)

    return None, None


# Extract full version and version parts from a source archive's
# PACKAGE_VERSION.TXT, which holds a single "<version> <sha>" line. Used only
# when the stamp is absent (an unstamped Bazel build from a tarball). Returns
# (None, None) if no path is given or the file is empty.
def _parse_package_version(package_version_path):
    if package_version_path is None:
        return None, None
    with open(package_version_path) as f:
        content = f.read().strip()
    if not content:
        return None, None
    version_full = content.split()[0]
    return version_full, _split_version(version_full)


# Write version information to CMake cache-style script.
def _write_version_info(out, version_full, version_parts):
    if version_full is None:
        # The full version is reported as "unknown", but the numeric components
        # are reported as 0 so that they remain usable as integers, e.g. in the
        # DRAKE_VERSION_* preprocessor macros in drake/version.h.
        out.write('set(DRAKE_VERSION "unknown")\n')
        out.write('set(DRAKE_VERSION_MAJOR "0")\n')
        out.write('set(DRAKE_VERSION_MINOR "0")\n')
        out.write('set(DRAKE_VERSION_PATCH "0")\n')
        out.write('set(DRAKE_VERSION_TWEAK "0")\n')
    else:
        out.write(f'set(DRAKE_VERSION "{version_full}")\n')
        out.write(f'set(DRAKE_VERSION_MAJOR "{version_parts[0]}")\n')
        out.write(f'set(DRAKE_VERSION_MINOR "{version_parts[1]}")\n')
        out.write(f'set(DRAKE_VERSION_PATCH "{version_parts[2]}")\n')
        out.write(f'set(DRAKE_VERSION_TWEAK "{version_parts[3]}")\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "input",
        type=argparse.FileType("r"),
        help="Path to file optionally containing stamp version.",
    )
    parser.add_argument(
        "output", type=argparse.FileType("w"), help="Path to output file."
    )
    parser.add_argument(
        "--package_version_path",
        default=None,
        help="Path to a source archive's PACKAGE_VERSION.TXT, used only when "
        "the stamp is absent (i.e. an unstamped Bazel build from a tarball).",
    )
    args = parser.parse_args()

    version_full, version_parts = _parse_stamp(args.input)
    if version_full is None:
        version_full, version_parts = _parse_package_version(
            args.package_version_path
        )
    _write_version_info(args.output, version_full, version_parts)

    return 0


if __name__ == "__main__":
    main()
