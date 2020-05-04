# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def sdformat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osrf/sdformat",
        # DNM: Drake CI test for PR https://github.com/osrf/sdformat/pull/268
        commit = "9f794faa76ff62715a26f139358e001e41ce653b",
        sha256 = "7b0253e3ea40904a456285b9ac8f6d75a3b15c6c7384b7c7e7fb5717675c3ec7",  # noqa
        build_file = "@drake//tools/workspace/sdformat:package.BUILD.bazel",
        mirrors = mirrors,
    )
