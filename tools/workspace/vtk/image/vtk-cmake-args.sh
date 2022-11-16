# Enforce that the script is being sourced, the parent script will have a bash
# array `vtk_cmake_args` defined to use on the final CMake configure line.  To
# support developer oversight the arguments will be printed to the console.
#
# The four different build flavors (linux, linux_wheel, mac, mac_wheel) share
# many common arguments, but different (platform specific) arguments such as
# dependencies must change depending on the flavor.  This file serves as the
# single point of coordination between the build flavors.
if [[ "${0}" == "${BASH_SOURCE[0]}" ]]; then
    echo 'vtk-cmake-args.sh: this file must be sourced.' >&2
    exit 1
fi
if ! [[ "${drake_vtk_build_flavor}" =~ ^(linux|linux_wheel|mac|mac_wheel)$ ]]; then
    echo 'vtk-cmake-args.sh: `drake_vtk_build_flavor` must be one of ' >&2
    echo '`linux`, `mac`, `linux_wheel` or `mac_wheel`.' >&2
    exit 1
fi

# The path to the directory containing this file.  The linux (docker) and macOS
# builds have different mount points for where this file lives on disk
this_file_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
vtk_common_cmake_args_path="${this_file_dir}/vtk-common-cmake-args"
if ! [[ -f "${vtk_common_cmake_args_path}" ]]; then
    echo "vtk-cmake-args.sh: '${vtk_common_cmake_args_path}' is not a file." >&2
    exit 1
fi

# Map the image/vtk-common-cmake-args file to an array, omitting any comments.
# The `mapfile` builtin is not available on macOS's default bash.
if type -a mapfile >/dev/null 2>&1; then
    mapfile -t vtk_common_cmake_args < \
        <(sed -e '/^#/d' -e 's/^/-D/' < "${vtk_common_cmake_args_path}")
else
    index=0
    vtk_common_cmake_args=()
    while IFS='' read -r line || [[ -n "${line}" ]]; do
        arg="$(sed -e '/^#/d' -e 's/^/-D/' <<< "${line}")"
        # If the line was commented out, arg will be the empty string.
        if ! [[ -z "${arg}" ]]; then
            vtk_common_cmake_args[index++]="${arg}"
        fi
    done < "${vtk_common_cmake_args_path}"
fi

declare -a vtk_cmake_args

# On macOS use the Cocoa graphics framework.
if [[ "${drake_vtk_build_flavor}" =~ ^(mac|mac_wheel)$ ]]; then
    vtk_cmake_args+=("-DVTK_USE_COCOA:BOOL=ON")
fi

# Define the C++ standard.
if [[ "${drake_vtk_build_flavor}" == "linux" ]]; then
    # C++ standard on Linux is defined by `tools/linux-${codename}.bazelrc`.
    if [[ "${codename}" == "focal" ]]; then
        readonly cxx_std="17"
    elif [[ "${codename}" == "jammy" ]]; then
        readonly cxx_std="20"
    else
        echo "Unsupported distrbution: ${codename}." >&2
        exit 1
    fi
elif [[ "${drake_vtk_build_flavor}" == "linux_wheel" ]]; then
    # TODO(svenevs): is this the right C++ standard choice for linux wheel?
    cxx_std="17"
elif [[ "${drake_vtk_build_flavor}" =~ ^(mac|mac_wheel)$ ]]; then
    # C++ standard on macOS is defined by `tools/macos.bazelrc`.
    readonly cxx_std="20"
else
    echo 'Unknown build flavor, no known C++ standard.' >&2
    exit 1
fi
# Enforce an exact CMake C++ standard.
vtk_cmake_args+=(
    "-DCMAKE_CXX_STANDARD=${cxx_std}"
    "-DCMAKE_CXX_STANDARD_REQUIRED=ON"
    "-DCMAKE_CXX_EXTENSIONS=OFF"
)

# Add fortification / hardening flags for redistributables.
# TODO(svenevs): fix the linking flags on macOS.
# TODO(svenevs): finish going through hardening-check on Linux.
vtk_cmake_args+=(
    "-DCMAKE_C_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations'"
    "-DCMAKE_CXX_FLAGS:STRING='-D_FORTIFY_SOURCE=2 -fstack-protector-strong -Wno-deprecated-declarations'"
)
if [[ "${codename}" != "mac" ]]; then
    # Not available / applicable on clang.
    vtk_cmake_args+=(
        "-DCMAKE_EXE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro'"
        "-DCMAKE_MODULE_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro'"
        "-DCMAKE_SHARED_LINKER_FLAGS:STRING='-Wl,-Bsymbolic-functions -Wl,-z,now -Wl,-z,relro'"
    )
fi

# For wheel builds we build a handful of external dependencies (in
# `tools/wheel/image/dependencies/projects`), as well as request that VTK bundle
# additional third party libraries that on native builds drake has a dependency
# on in `tools/workspace/{dependency}`.
if [[ "${drake_vtk_build_flavor}" =~ ^(linux_wheel|mac_wheel)$ ]]; then
    vtk_bundle_external="ON"
    vtk_cmake_args+=(
        -DVTK_MODULE_ENABLE_VTK_RenderingFreeTypeFontConfig:STRING=DONT_WANT
    )
else
    vtk_bundle_external="OFF"
    vtk_cmake_args+=(
        -DVTK_MODULE_ENABLE_VTK_RenderingFreeTypeFontConfig:STRING=DEFAULT
    )
fi
vtk_cmake_args+=(
    "-DVTK_MODULE_USE_EXTERNAL_VTK_expat:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_freetype:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_glew:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_hdf5:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_jsoncpp:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_libxml2:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_lzma:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_netcdf:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_ogg:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_sqlite:BOOL=${vtk_bundle_external}"
    "-DVTK_MODULE_USE_EXTERNAL_VTK_theora:BOOL=${vtk_bundle_external}"
)
