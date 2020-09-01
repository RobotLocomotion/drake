import os
from textwrap import dedent
import unittest

from pydrake.common.yaml import (
    yaml_dump,
    yaml_load,
    yaml_load_data,
    yaml_load_file,
)


class TestYaml(unittest.TestCase):

    def test_via_file(self):
        filename = os.path.join(os.environ["TEST_TMPDIR"], "foo.yaml")
        with open(filename, "w") as f:
            f.write("{foo: bar}\n")
        for func in yaml_load, yaml_load_file:
            actual = func(filename=filename)
            self.assertDictEqual(actual, {"foo": "bar"})

    def test_via_str(self):
        for func in yaml_load, yaml_load_data:
            actual = func(data="{foo: bar}")
            self.assertDictEqual(actual, {"foo": "bar"})

    def test_load_merge_keys(self):
        """Sanity check support for merge keys."""
        # See https://yaml.org/type/merge.html for details; pyyaml already
        # supports this feature natively, Anzu doesn't need anything special.
        data = """
        _template: &template
          foo: 1.0
        doc:
          value:
            << : *template
            bar: 2.0
        """
        parsed = yaml_load_data(data)
        self.assertDictEqual(parsed["doc"], {
            "value": {
                "foo": 1.0,
                "bar": 2.0,
            }})

    def test_stochastic(self):
        # Per the variant types in stochastic.h.
        data = """
        doc: [ !Deterministic { value: 5.0 },
               !Gaussian { mean: 2.0, std: 4.0 },
               !GaussianVector { mean: [1.1, 1.2, 1.3], std: [0.1, 0.2, 0.3] },
               !Uniform { min: 1.0, max: 5.0 },
               !UniformDiscrete { values: [1, 1.5, 2] },
               !UniformVector { min: [10, 20], max: [11, 22] },
        ]
        """
        doc = yaml_load(data=data)["doc"]
        self.assertDictEqual(doc[0], {
            "_tag": "!Deterministic",
            "value": 5.0,
        })
        self.assertDictEqual(doc[1], {
            "_tag": "!Gaussian",
            "mean": 2.0,
            "std": 4.0,
        })
        self.assertDictEqual(doc[2], {
            "_tag": "!GaussianVector",
            "mean": [1.1, 1.2, 1.3],
            "std": [0.1, 0.2, 0.3]
        })
        self.assertDictEqual(doc[3], {
            "_tag": "!Uniform",
            "min": 1.0,
            "max": 5.0,
        })
        self.assertDictEqual(doc[4], {
            "_tag": "!UniformDiscrete",
            "values": [1, 1.5, 2],
        })
        self.assertDictEqual(doc[5], {
            "_tag": "!UniformVector",
            "min": [10, 20],
            "max": [11, 22],
        })

        actual_str = yaml_dump({"doc": doc})
        expected_str = dedent(
            r"""
            doc:
            - !Deterministic {value: 5.0}
            - !Gaussian {mean: 2.0, std: 4.0}
            - !GaussianVector
              mean: [1.1, 1.2, 1.3]
              std: [0.1, 0.2, 0.3]
            - !Uniform {max: 5.0, min: 1.0}
            - !UniformDiscrete
              values: [1, 1.5, 2]
            - !UniformVector
              max: [11, 22]
              min: [10, 20]
            """
        ).lstrip()
        self.assertEqual(expected_str, actual_str)

    def test_transform_uniform(self):
        # Per the variant types in transform.h.
        data = """
        rotation: !Uniform {}
        """
        rotation = yaml_load(data=data)["rotation"]
        self.assertDictEqual(rotation, {
            "_tag": "!Uniform",
        })

    def test_transform_angle_axis(self):
        # Per the variant types in transform.h.
        data = """
        rotation: !AngleAxis
          angle_deg: !Uniform { min: -10, max: 10 }
          axis: [0, 0, 1]
        """
        rotation = yaml_load(data=data)["rotation"]
        self.assertDictEqual(rotation, {
            "_tag": "!AngleAxis",
            "angle_deg": {"_tag": "!Uniform", "max": 10, "min": -10},
            "axis": [0, 0, 1],
        })

    def test_transform_rpy(self):
        # Per the variant types in transform.h.
        data = """
        rotation: !Rpy
          deg: !UniformVector
            min: [80, -0.25, -1.]
            max: [100, 0.25,  1.]
        """
        rotation = yaml_load(data=data)["rotation"]
        self.assertDictEqual(rotation, {
            "_tag": "!Rpy",
            "deg": {
                "_tag": "!UniformVector",
                "min": [80, -0.25, -1.],
                "max": [100, 0.25,  1.],
            },
        })

    def test_unknown_tag(self):
        data = """
        foo: !Bar
          detail: 22
        """
        bar = yaml_load(data=data)["foo"]
        self.assertDictEqual(bar, {
            "_tag": "!Bar",
            "detail": 22,
        })

    def test_standard_tag(self):
        data = """
        foo: !!str bar
        """
        bar = yaml_load(data=data)["foo"]
        self.assertEqual(bar, "bar")

    def test_using_template(self):
        data = """
        _template: &template
          value:
            foo: 1.0
            bar: 2.0
        name1:
          <<: *template
          seed: 1
        name2:
          <<: *template
          seed: 2
        """
        expected = {
            "name1": {
                "seed": 1,
                "value": {
                    "foo": 1.0,
                    "bar": 2.0,
                }
            },
            "name2": {
                "seed": 2,
                "value": {
                    "foo": 1.0,
                    "bar": 2.0,
                }
            },
        }
        self.assertDictEqual(yaml_load_data(data), expected)

        # Test again, asking to keep the template around.
        expected["_template"] = {
            "value": {
                "foo": 1.0,
                "bar": 2.0,
            }
        }
        self.assertDictEqual(yaml_load_data(data, private=True), expected)
