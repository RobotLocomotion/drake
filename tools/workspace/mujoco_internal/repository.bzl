load("//tools/workspace:github.bzl", "github_archive")

def mujoco_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco",
        # Note for upgraders: Drake compiles a small subset of MuJoCo's
        # engine internals (the native convex collision detection, i.e.
        # GJK/EPA with multi-point contact recovery) whose private headers
        # and struct layouts may change between MuJoCo releases. After an
        # upgrade, re-run the multipoint contact unit tests in
        # //geometry/proximity; they pin down the expected contact manifolds
        # numerically.
        upgrade_type = "release",
        commit = "3.12.0",
        sha256 = "9faa979982c3e924e8aaff3b16983bba3a1ab19c81f4f73178ae9c2ea25e467d",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
