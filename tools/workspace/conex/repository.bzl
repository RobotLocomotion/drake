# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def conex_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ToyotaResearchInstitute/conex",
        commit ="c6a63d551408bb8353a4d983793a7566eb4579b1",
        sha256 = "cc33c3390d7582ffbb22ad9c2a1e369b86340f9f1357ed7b3aa4fc4ae0d182b0",
        mirrors = mirrors,
    )
