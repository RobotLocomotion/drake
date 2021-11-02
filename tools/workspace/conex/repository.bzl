# -*- python -*-
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def conex_repository(
        name,
        mirrors = None):
      git_repository(
          name = name,
          commit = "782e4d2ab1adfcebad234ddd48f5e32b0e41edde",
          shallow_since = "1625176963 -0400",
          remote = "git@github.com:frankpermenter/conex.git",
      )
