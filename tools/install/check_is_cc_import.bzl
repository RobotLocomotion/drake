load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("//tools/skylark:cc.bzl", "CcInfo")
load(":install.bzl", "InstallInfo")

# The check_is_cc_import rule checks that none of its `deps = []` are compiled
# from source. If any are from source, it immediately fails during the analysis
# phase. If everything is fine, it returns an empty InstallInfo so that it can
# be used as a dependency of an install target. Big picture, this allows us to
# check that the arcane spelling of the required override_repository flag in
# our CMakeLists.txt had the correct effect.

def _impl(ctx):
    cc_infos = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    for input in cc_infos.linking_context.linker_inputs.to_list():
        for library in input.libraries:
            if len(library.pic_objects) > 0:
                fail("The CMake --override_repository flag was misspelled.")
    return [
        InstallInfo(
            install_actions = [],
            rename = {},
            installed_files = {},
        ),
    ]

check_is_cc_import = rule(
    implementation = _impl,
    attrs = dict(
        deps = attr.label_list(providers = [CcInfo]),
    ),
)
