load(
    "//tools/workspace:execute.bzl",
    "execute_or_fail",
)
load(
    "//tools/workspace:github.bzl",
    "setup_github_repository",
)

def _remove_comments_and_blank_lines(text):
    result = ""
    for line in text.splitlines():
        if "#" in line:
            line, _ = line.split("#", 1)
        if line.strip() == "":
            continue
        result += line
        result += "\n"
    return result

def parse_module(repo_ctx, subdir):
    """Parses and returns a vtk.module file as a dict.

    For an overview of VTK modules, see:
    https://github.com/Kitware/VTK/blob/v9.2.6/Documentation/Doxygen/ModuleSystem.md#modules

    An upstream `Foo/Bar/vtk.module` file is formatted like this:
    NAME
      VTK::FooBar
    THIRD_PARTY
    DEPENDS
      VTK::CommonCore
      VTK::CommonDataModel

    Our returned dict will look like this:
    {
      "subdir": "Foo/Bar",
      "NAME": "VTK::FooBar",
      "THIRD_PARTY": 1,
      "DEPENDS: [
        "VTK::CommonCore",
        "VTK::CommonDataModel"
      ],
    }

    Note that even if a list-valued field like "DEPENDS" only has one item in
    a given module file, we still parse it as a one-item list so that our code
    that consumes the information can iterate over it without any hassle.

    Internally, to make it easier to parse each dict item as a single line,
    we'll munge the file contents to have one line per dict key:

    NAME=VTK::FooBar
    THIRD_PARTY
    DEPENDS=VTK::CommonCore=VTK::CommonDataModel
    """

    result = dict(subdir = subdir)
    content = repo_ctx.read(subdir + "/vtk.module")
    clean_content = _remove_comments_and_blank_lines(content)
    lines = clean_content.replace("\n  ", "=").splitlines()
    for line in lines:
        tokens = line.split("=")
        key, values = tokens[0], tokens[1:]

        # To date, VTK upstream is consistent that key names ending in "S" are
        # list-valued, and key names not ending in "S" are not list-valued. If
        # they ever break that pattern, we'll need a lookup table here instead
        # of this heuristic.
        parse_as_list = key.endswith("S") or key in [
            "SPDX_COPYRIGHT_TEXT",
        ]

        if parse_as_list:
            result[key] = values
        elif len(values) == 1:
            result[key] = values[0]
        elif len(values) == 0:
            # This a a boolean-like item (either present, or not).
            # We'll encode that as an int to avoid JSON encoding snafus.
            result[key] = 1
        else:
            fail(("vtk/{subdir}/vtk.module: Got multiple values for {key} " +
                  "but we assumed (because its name ended with 'S') that it " +
                  "was not supposed to be a list").format(
                subdir = subdir,
                key = key,
            ))

    return result

def create_modules_bzl(repo_ctx):
    """Finds all vtk.module files, parses them, and writes their content into
    a loadable `modules.bzl` file in the root of the repository.

    This is necessary because BUILD files can't parse external metadata as part
    of their rules; the only thing they can do is load `*.bzl` files, so we
    must convert the module metadata to bzl.
    """

    # Find all vtk.module files.
    subdirs = []
    for line in execute_or_fail(
        repo_ctx,
        ["/usr/bin/find", "-L", ".", "-name", "vtk.module"],
    ).stdout.splitlines():
        # Remove the leading "./" and tailing "/vtk.module".
        subdir = line[2:-11]
        subdirs.append(subdir)

    # Parse all vtk.module files.
    modules = dict()
    for subdir in subdirs:
        content = parse_module(repo_ctx, subdir)
        modules[content["NAME"]] = content

    # Encode the output. Because we lean on json encoding, this will not work
    # correctly if anything in the data structure is None, True, or False.
    # We rely on parse_module() to avoid that situation.
    bzl_content = "MODULES = " + json.encode(modules) + "\n"

    # Pass along the os.name and os.arch for convenience.
    platform = dict(name = repo_ctx.os.name, arch = repo_ctx.os.arch)
    bzl_content += "PLATFORM = " + json.encode(platform) + "\n"

    # Write the output.
    repo_ctx.file("modules.bzl", content = bzl_content)

def _impl_local_override(repo_ctx):
    for item in repo_ctx.path(repo_ctx.attr.path).readdir():
        repo_ctx.symlink(item, item.basename)
    create_modules_bzl(repo_ctx)
    repo_ctx.symlink(repo_ctx.attr.build_file, "BUILD.bazel")
    repo_ctx.symlink(repo_ctx.attr.settings_bzl, "settings.bzl")

_vtk_internal_repository_impl_local_override = repository_rule(
    attrs = {
        "path": attr.string(),
        "build_file": attr.label(),
        "settings_bzl": attr.label(allow_single_file = True),
    },
    implementation = _impl_local_override,
)

def _impl(repo_ctx):
    error = setup_github_repository(repo_ctx).error
    if error != None:
        fail(error)
    create_modules_bzl(repo_ctx)
    repo_ctx.symlink(repo_ctx.attr.settings_bzl, "settings.bzl")

_vtk_internal_repository_impl = repository_rule(
    attrs = {
        # These are the attributes for setup_github_repository.
        "repository": attr.string(),
        "commit": attr.string(),
        "sha256": attr.string(),
        "build_file": attr.label(),
        "patches": attr.label_list(),
        "extra_strip_prefix": attr.string(),
        "mirrors": attr.string_list_dict(),
        # This attribute is specific to our rule, not setup_github_repository.
        "settings_bzl": attr.label(allow_single_file = True),
    },
    implementation = _impl,
)

def _resolve_drake_abbreviation(name, label_str):
    """De-abbreviates the given label_str as a Drake tools/workspace label.
    If the label_str is None, returns None. If the label_str is relative,
    interprets it relative to the "@drake//tools/workspace/{name}/" package
    and returns an absolute label. Otherwise, returns the label_str unchanged.
    """
    if label_str == None:
        return None
    if label_str.startswith(":"):
        return "@drake//tools/workspace/" + name + label_str
    return label_str

def vtk_internal_repository(
        name,
        local_repository_override = None,
        repository = "Kitware/VTK",
        commit = "a9bd41e68fa5ee434dbf6224c6c78af334383870",
        sha256 = "cfdb1e6201b6ba3b6922a95b532b8d78c18a9bd9a2e723e7910461886af2c030",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            # Drake's conventions for VTK patches are:
            # - All "patches/upstream/" come first; these are the changes that
            #   will be upstreamed into VTK itself, so they should be the first
            #   changes applied to reduce merge conflict churn.
            # - Patch file names should begin with the name of the module being
            #   edited (e.g., patching IO/Image is named io_image_{foo}.patch).
            # - Use alphabetical order within a directory when listing patches.
            ":patches/upstream/rendering_opengl2_replace_single_scattering_with_multi_scattering_in_pbr.patch",
            ":patches/upstream/utilities_x11_more_functions.patch",
            ":patches/common_core_fmt12.patch",
            ":patches/common_core_fmt9.patch",
            ":patches/common_core_nobacktrace.patch",
            ":patches/common_core_rm_cin_prompting.patch",
            ":patches/common_core_version.patch",
            ":patches/common_datamodel_no_pegtl.patch",
            ":patches/common_executionmodel_disable_static_destructors.patch",
            ":patches/io_image_formats.patch",
            ":patches/rendering_opengl2_nobacktrace.patch",
            ":patches/rendering_opengl2_no_factory.patch",
            ":patches/rendering_opengl2_no_global_display_counter.patch",
            ":patches/rendering_opengl2_no_global_x11_counter.patch",
            ":patches/rendering_opengl2_preserve_direct_light_specular_reflections.patch",  # noqa
            ":patches/utilities_x11_dlsym.patch",
            ":patches/vtkfast_float_hidden.patch",
            ":patches/vtkpugixml_hidden.patch",
            ":patches/vtkscn_hidden.patch",
            ":patches/vtksys_hidden.patch",
        ],
        settings_bzl = ":settings.bzl",
        **kwargs):
    """Declares VTK using a repository rule, typically from a github download
    but when local_repository_override is provided it will be used instead.

    The current local_repository_override support is slightly inelegant,
    because it does not automatically apply any of the `patches = [...]`.
    Instead, you will need to manually cherry-pick all of Drake's patches into
    your VTK checkout by hand.
    """
    build_file = _resolve_drake_abbreviation(name, build_file)
    patches = [
        _resolve_drake_abbreviation(name, one_patch)
        for one_patch in (patches or [])
    ]
    settings_bzl = _resolve_drake_abbreviation(name, settings_bzl)
    if local_repository_override:
        _vtk_internal_repository_impl_local_override(
            name = name,
            path = local_repository_override,
            build_file = build_file,
            settings_bzl = settings_bzl,
        )
    else:
        _vtk_internal_repository_impl(
            name = name,
            repository = repository,
            commit = commit,
            sha256 = sha256,
            build_file = build_file,
            patches = patches,
            settings_bzl = settings_bzl,
            **kwargs
        )
