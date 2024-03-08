load("//tools/workspace:github.bzl", "github_archive")

def openusd_internal_repository(
        name,
        mirrors = None):
    path = "/home/hongw/temp/usd-prefix"
    native.new_local_repository(
        name = name,
        path = path,
        build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(
    name = "openusd",
    hdrs = glob(["include/**"]),
    includes = ["include"],
    linkopts = [
        "-L{path}/lib",
        "-Wl,-rpath,{path}/lib",
        "-lusd_arch",
        "-lusd_gf",
        "-lusd_js",
        "-lusd_plug",
        "-lusd_tf",
        "-lusd_trace",
        "-lusd_vt",
        "-lusd_work",
        "-lusd_ar",
        "-lusd_kind",
        "-lusd_ndr",
        "-lusd_pcp",
        "-lusd_sdf",
        "-lusd_sdr",
        "-lusd_usd",
        "-lusd_usdGeom",
        "-lusd_usdShade",
        "-lusd_usdUtils",
    ],
)
""".format(path = path),
    )
