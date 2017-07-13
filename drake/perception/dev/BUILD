# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:python_lint.bzl", "python_lint")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
	name = "neural_network",
	srcs = [
					"neural_network.cc",
					],
	hdrs = [
					"neural_network.h",
					],
	deps = [
					"//drake/systems/framework",
				 ],
)

# === test/ ===
drake_cc_googletest(
	name = "neural_network_test",
	srcs = ["test/neural_network_test.cc"],
	deps = [
					":neural_network",
					"//drake/common:eigen_matrix_compare",
					],
)

