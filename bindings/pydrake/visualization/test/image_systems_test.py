import math
import unittest

from pydrake.common.test_utilities import numpy_compare
from pydrake.geometry import (
    RenderLabel,
    Rgba,
)
from pydrake.systems.sensors import (
    ImageDepth16U,
    ImageDepth32F,
    ImageLabel16I,
    ImageRgba8U,
)
from pydrake.visualization import (
    ColorizeDepthImage,
    ColorizeLabelImage,
    ConcatenateImages,
)


class TestColorizeDepthImage(unittest.TestCase):

    def test_properties(self):
        dut = ColorizeDepthImage()
        self.assertIsInstance(dut.invalid_color, Rgba)
        rgba = Rgba(0.1, 0.2, 0.3, 0.4)
        dut.invalid_color = rgba
        self.assertEqual(dut.invalid_color, rgba)

    def test_eval_32f_and_calc_32f(self):
        dut = ColorizeDepthImage()
        context = dut.CreateDefaultContext()
        depth = ImageDepth32F(6, 2)
        dut.GetInputPort("depth_image_32f").FixValue(context, depth)
        color = dut.GetOutputPort("color_image").Eval(context)
        self.assertIsInstance(color, ImageRgba8U)
        dut.Calc(depth, color)

    def test_eval_16u_and_calc_16u(self):
        depth = ImageDepth16U(6, 2)
        dut = ColorizeDepthImage()
        context = dut.CreateDefaultContext()
        dut.GetInputPort("depth_image_16u").FixValue(context, depth)
        color = dut.GetOutputPort("color_image").Eval(context)
        self.assertIsInstance(color, ImageRgba8U)
        dut.Calc(depth, color)


class TestColorizeLabelImage(unittest.TestCase):

    def test_properties(self):
        dut = ColorizeLabelImage()
        self.assertIsInstance(dut.background_color, Rgba)
        rgba = Rgba(0.1, 0.2, 0.3, 0.4)
        dut.background_color = rgba
        self.assertEqual(dut.background_color, rgba)

    def test_eval_and_calc(self):
        dut = ColorizeLabelImage()
        context = dut.CreateDefaultContext()
        label = ImageLabel16I(6, 2)
        dut.GetInputPort("label_image").FixValue(context, label)
        color = dut.GetOutputPort("color_image").Eval(context)
        self.assertIsInstance(color, ImageRgba8U)
        dut.Calc(label, color)


class TestConcatenateImages(unittest.TestCase):

    def test_smoke(self):
        rows, cols = (2, 3)
        dut = ConcatenateImages(rows=rows, cols=cols)

        # Set the input port values.
        context = dut.CreateDefaultContext()
        small = ImageRgba8U(4, 2)
        for row in range(rows):
            for col in range(cols):
                dut.get_input_port(row=row, col=col).FixValue(context, small)

        # Check the output image.
        actual = dut.GetOutputPort("color_image").Eval(context)
        self.assertEqual(actual.width(), small.width() * cols)
        self.assertEqual(actual.height(), small.height() * rows)
