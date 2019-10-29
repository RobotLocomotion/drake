import unittest
import filecmp
from tools.pybind11_coverage import libclang_parser
print(__import__('glob').glob("**/*", recursive=True))


class TestLibclangParser(unittest.TestCase):
    def setUp(self):
        self.docstrings = ["pydrake_doc.drake.solvers." + x for x in [
            'OsqpSolver.doc',
            'OsqpSolver.ctor.doc',
            'OsqpSolverDetails.doc',
            'OsqpSolverDetails.iter.doc',
            'OsqpSolverDetails.status_val.doc',
            'OsqpSolverDetails.primal_res.doc',
            'OsqpSolverDetails.dual_res.doc',
            'OsqpSolverDetails.setup_time.doc',
            'OsqpSolverDetails.solve_time.doc',
            'OsqpSolverDetails.polish_time.doc',
            'OsqpSolverDetails.run_time.doc'
        ]]

    def test_parser(self):
        filenames = ["tools/pybind11_coverage/test/test_py.cc"]
        pybind_docstrings = \
            libclang_parser.get_docstring_for_bindings(filenames)
        assert(len(pybind_docstrings) == len(self.docstrings))
        assert(set(pybind_docstrings) == set(self.docstrings))
        assert(filecmp.cmp(
            "tools/pybind11_coverage/file_coverage_test.csv",
            "tools/pybind11_coverage/test/file_coverage.csv",
            shallow=False))

        assert(filecmp.cmp(
            "tools/pybind11_coverage/class_coverage_test.csv",
            "tools/pybind11_coverage/test/class_coverage.csv",
            shallow=False))

        assert(filecmp.cmp(
            "tools/pybind11_coverage/documentation_pybind_test.xml",
            "tools/pybind11_coverage/test/documentation_pybind.xml",
            shallow=False))
