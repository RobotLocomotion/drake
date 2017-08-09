# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)
load("//tools:lint.bzl", "add_lint_tests")
load("//tools:install_data.bzl", "install_data")

package(default_visibility = ["//visibility:public"])

drake_cc_binary(
    name = "pr2_simulation",
    srcs = ["pr2_simulation.cc"],
    data = [":models"],
    deps = [
	"//drake/common:find_resource",
    	"//drake/lcm",
	"//drake/multibody/parsers",
	"//drake/multibody/rigid_body_plant:drake_visualizer",
	"//drake/multibody/rigid_body_plant:rigid_body_plant",
	"//drake/multibody:rigid_body_tree_construction",
	"//drake/systems/analysis:semi_explicit_euler_integrator",
	"//drake/systems/analysis:simulator",
	"//drake/systems/framework:diagram_builder",
	"//drake/systems/primitives:constant_vector_source",
    ],
)

install_data()

add_lint_tests()
