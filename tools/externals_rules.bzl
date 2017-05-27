load("@drake//tools:github.bzl", "github_archive")

def drake_external_rule_repositories(drake_relative_path):
    """
    Provide externals that define rules which are used to define other externals
    listed in @drake//tools:externals.bzl.

    The required drake_relative_path= is necessary to define the drake workspace
    root relative to the active workspace root.

    This is due to constraints with bazel, which does not permit using load()
    within 
    For more information, see https://github.com/bazelbuild/bazel/issues/2757.
    """

    # TODO(eric.cousineau): See if there is a way to get:
    # Label("@drake//:install").workspace_root to NOT always return "external/drake"
    # Then this annoying drake_relative_path argument can be removed.

    native.local_repository(
        name = "kythe",
        path = drake_relative_path + "/tools/third_party/kythe",
    )

    # Required for buildifier.
    github_archive(
        name = "io_bazel_rules_go",
        repository = "bazelbuild/rules_go",
        commit = "0.4.4",
        sha256 = "afec53d875013de6cebe0e51943345c587b41263fdff36df5ff651fbf03c1c08",
    )
