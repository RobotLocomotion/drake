load("//tools:install.bzl", "install")

def install_data(destination):
    native.filegroup(
        name = "install_models",
        srcs = native.glob(
        include=[
            "**/*.csv",
            "**/*.json",
            "**/*.obj",
            "**/*.sdf",
            "**/*.urdf",
            "**/*.xml",
        ],
        exclude=["**/test/*"],
        ),
        visibility = ["//visibility:public"],
    )

    install(
        name = "install_data",
        data = [":install_models"],
        data_dest = destination,
        visibility = ["//visibility:public"],
    )

