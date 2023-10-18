# -*- coding: utf-8 -*-

from pydrake.common.parsing import (
    PackageMap,
)

import copy
import os
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings


class TestParsing(unittest.TestCase):

    def test_package_map(self):
        # Simple coverage test for constructors.
        dut = PackageMap()
        self.assertEqual(dut.size(), 2)
        PackageMap(other=dut)
        copy.copy(dut)

        dut = PackageMap.MakeEmpty()
        dut2 = PackageMap.MakeEmpty()
        tmpdir = os.environ.get('TEST_TMPDIR')

        # Simple coverage test for Add, AddMap, Contains, size,
        # GetPackageNames, GetPath, AddPackageXml, Remove.
        dut.Add(package_name="root", package_path=tmpdir)
        dut2.Add(package_name="root", package_path=tmpdir)
        dut.AddMap(dut2)
        self.assertTrue(dut.Contains(package_name="root"))
        self.assertEqual(dut.size(), 1)
        self.assertEqual(dut.GetPackageNames(), ["root"])
        self.assertEqual(dut.GetPath(package_name="root"), tmpdir)
        dut.AddPackageXml(filename=FindResourceOrThrow(
            "drake/multibody/parsing/test/box_package/package.xml"))
        dut2.Remove(package_name="root")
        self.assertEqual(dut2.size(), 0)

        # Simple coverage test for folder and environment.
        dut.PopulateFromEnvironment(environment_variable='TEST_TMPDIR')
        dut.PopulateFromFolder(path=tmpdir)

    def test_package_map_remote_params(self):
        dut = PackageMap.RemoteParams(
            urls=["file:///tmp/missing.zip"],
            sha256="0" * 64,
            archive_type="zip",
            strip_prefix="prefix",)
        self.assertIn("missing.zip", dut.ToJson())
        copy.copy(dut)
        copy.deepcopy(dut)

    def test_package_map_add_remote(self):
        """Runs a full lifecycle of AddRemote + GetPath to check that Python
        bindings calling C++ code that shells out to Python all plays nice.
        """
        dut = PackageMap.MakeEmpty()
        zipfile = FindResourceOrThrow(
            "drake/multibody/parsing/test/package_map_test_packages/"
            "compressed.zip")
        dut.AddRemote(package_name="compressed",
                      params=PackageMap.RemoteParams(
                          urls=[f"file://{zipfile}"],
                          sha256=("b4bdbad313293ca61fe8f4ed1b5579da"
                                  "dadb3a5c08f0a6d06a8e39e5f97f1bd1"),
                          strip_prefix="compressed_prefix"))
        path = dut.GetPath("compressed")
        with open(f"{path}/README", encoding="utf-8") as f:
            self.assertEqual(f.read(), "This package is empty.\n")
