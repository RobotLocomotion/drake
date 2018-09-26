import unittest


class TestDocumentationPybind(unittest.TestCase):
    def test_generation_simple(self):
        with open("bindings/pydrake/documentation_pybind.h") as f:
            text = f.read()
        snippets = [
            # Overarching structure
            "constexpr struct /* pydrake_doc */ {",
            # Anonymous struct comment
            "struct /* drake */ {",
            # Anonymous struct variable
            "} drake;",
            # Qualified symbol
            "// drake::math::RigidTransform::RigidTransform<T>",
            # File with line number
            "// drake/math/rigid_transform.h:",
            # First symbol documentation
            "const char* doc =",
            # Overload symbol documentation
            "const char* doc_3 =",
            # Constructor
            "} ctor;",
        ]
        # Do not use `assertIn`, as we don't want to dump out all the text in
        # the case of failure.
        for snippet in snippets:
            self.assertTrue(
                snippet in text, "Could not find snippet:\n{}".format(snippet))
