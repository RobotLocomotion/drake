# -*- python -*-
load("@drake//tools/workspace:github.bzl", "github_archive")

def conex_repository(
        name,
        mirrors = None):
      github_archive(
          name = name,
          repository = "ToyotaResearchInstitute/conex",
          commit = "782e4d2ab1adfcebad234ddd48f5e32b0e41edde",
          sha256 = "af6223d96ce6ebd3de64cd8b2547cc889f346b6cadf1742c21cf698a510f5136",
          mirrors = mirrors,
      )
