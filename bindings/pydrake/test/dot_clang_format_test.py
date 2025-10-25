import unittest


class TestDotfile(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_dotfile_consistency(self):
        # Slurp both files.
        with open(".clang-format", encoding="utf-8") as f:
            root_contents = f.readlines()
        with open("bindings/pydrake/.clang-format", encoding="utf-8") as f:
            bindings_contents = f.readlines()

        # The bindings file should be longer.
        self.assertGreater(len(bindings_contents), len(root_contents))

        # Some settings are explicitly set in both the root dotfile and the
        # bindings dotfile. To assist our diffing of the two files, we'll
        # remove them from our model of the root dotfile.
        set_in_both = [
            "AllowShortLambdasOnASingleLine",
        ]
        for setting in set_in_both:
            needle = f"{setting}: "
            matches = [
                i
                for i, one_line in enumerate(root_contents)
                if one_line.startswith(needle)
            ]
            self.assertEqual(len(matches), 1, msg=setting)
            setting_i = matches[0]
            # Remove the setting line and the blank line after.
            del root_contents[setting_i]
            self.assertEqual(root_contents[setting_i], "\n")
            del root_contents[setting_i]
            # Remove the preceding comment block.
            comment_i = setting_i - 1
            while root_contents[comment_i].startswith("#"):
                del root_contents[comment_i]
                comment_i -= 1

        # Sanity check that it was actually in the bindings dotfile.
        for setting in set_in_both:
            needle = f"{setting}: "
            matches = [
                i
                for i, one_line in enumerate(bindings_contents)
                if one_line.startswith(needle)
            ]
            self.assertEqual(len(matches), 1, msg=setting)

        # The bindings config only ever appends to the root settings.
        self.assertMultiLineEqual(
            "".join(root_contents),
            "".join(bindings_contents[: len(root_contents)]),
        )
