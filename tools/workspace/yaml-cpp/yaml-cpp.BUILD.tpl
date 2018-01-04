# Altered copy of the pkg_config_package build template.
# Adds a hard-coded dependency on "@boost" to work around a yaml-cpp
# misconfiguration.
# https://github.com/RobotLocomotion/drake/pull/7602#issuecomment-353093726

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "%{name}",
    srcs = ["%{srcs}"],
    hdrs = glob(["include/**"]),
    defines = ["%{defines}"],
    includes = ["%{includes}"],
    linkopts = ["%{linkopts}"],
    deps = [ "@boost" ]
)
