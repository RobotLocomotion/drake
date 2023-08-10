import unittest

from pydrake.systems.sensors import ImageDepth32F, ImageLabel16I, ImageRgba8U
from pydrake.visualization import LcmImageArrayViewer


class TestLcmImageArrayViwer(unittest.TestCase):

    def test_concatenate_images(self):
        test_images = [ImageRgba8U(10, 10, 4) for i in range(4)]
        result = LcmImageArrayViewer.concatenate_images(test_images, 1, 4)
        self.assertEqual(result.shape, (10, 40, 4))
