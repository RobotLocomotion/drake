# Define some common bash variables used throughout the VTK build process:
# - os_name: `uname -s` output ('Darwin' or 'Linux').
# - architecture: either 'arm64' or 'x86_64'.
# - codename: one of 'mac' or `lsb_release -cs` on Linux.
readonly os_name="$(uname -s)"
if [[ "${os_name}" == "Darwin" ]]; then
    # On macOS x86_64 (rather than reported i386) for consistency.
    if [[ "$(/usr/bin/arch)" == "arm64" ]]; then
        readonly architecture="arm64"
    else
        readonly architecture="x86_64"
    fi
    readonly codename="mac"  # We build on the oldest supported distribution.
else
    readonly codename="$(lsb_release -cs)"
    readonly architecture="$(/usr/bin/arch)"
fi
