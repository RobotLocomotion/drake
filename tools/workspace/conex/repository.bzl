# -*- python -*-
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def conex_repository(
        name,
        mirrors = None):
      git_repository(
          name = name,
          commit = "bc34a103c21f12b3bbfbb2db02b81772398cf6a3",
          shallow_since = "1618867192 -0400",
          remote = "git@github.com:frankpermenter/conex.git",
      )
