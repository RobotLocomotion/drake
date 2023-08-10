from io import BytesIO
import unittest

import numpy as np
from PIL import Image

from drake import lcmt_image, lcmt_image_array
from pydrake.lcm import DrakeLcm
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import LcmPublisherSystem
from pydrake.systems.sensors import ImageDepth32F, ImageLabel16I, ImageRgba8U
from pydrake.visualization.lcm_image_array_viewer import LcmImageArrayViewer


class TestLcmImageArrayViewer(unittest.TestCase):
    def _get_rgba_lcmt_image(self):
        image_message = lcmt_image()
        image_message.pixel_format = lcmt_image.PIXEL_FORMAT_RGBA
        image_message.channel_type = lcmt_image.CHANNEL_TYPE_UINT8
        image_message.width = 1
        image_message.height = 1
        image_message.row_stride = 4
        image_message.size = 4
        image_message.data = [0] * 4
        return image_message

    def _get_depth_lcmt_image(self):
        image_message = lcmt_image()
        image_message.pixel_format = lcmt_image.PIXEL_FORMAT_DEPTH
        image_message.channel_type = lcmt_image.CHANNEL_TYPE_FLOAT32
        image_message.width = 1
        image_message.height = 1
        image_message.row_stride = 4
        image_message.size = 4
        image_message.data = [1.0]
        return image_message

    def _get_label_lcmt_image(self):
        image_message = lcmt_image()
        image_message.pixel_format = lcmt_image.PIXEL_FORMAT_LABEL
        image_message.channel_type = lcmt_image.CHANNEL_TYPE_INT16
        image_message.width = 1
        image_message.height = 1
        image_message.row_stride = 2
        image_message.size = 2
        image_message.data = [127]
        return image_message

    def test_image_processing(self):
        lcm_image_array_viewer = LcmImageArrayViewer(
            host="localhost",
            port=1234,
            channel="does_not_matter",
            unit_test=True,
        )
        lcm = lcm_image_array_viewer._lcm
        builder = DiagramBuilder()
        image_array_publisher = builder.AddSystem(
            LcmPublisherSystem.Make(
                channel="does_not_matter",
                lcm_type=lcmt_image_array,
                lcm=lcm,
                publish_period=1.0,
            )
        )
        diagram = builder.Build()

        # Create an lcmt_image_array containing different types of images.
        array_message = lcmt_image_array()
        array_message.num_images = 3
        array_message.images = [
            # TODO(zachfang): Fix this.
            self._get_rgba_lcmt_image(),
            self._get_rgba_lcmt_image(),
            self._get_rgba_lcmt_image(),
            # self._get_depth_lcmt_image(),
            # self._get_label_lcmt_image(),
        ]

        context = diagram.CreateDefaultContext()
        publisher_context = image_array_publisher.GetMyContextFromRoot(context)
        image_array_publisher.get_input_port().FixValue(
            publisher_context, array_message
        )
        diagram.ForcedPublish(context)
        lcm.HandleSubscriptions(timeout_millis=1)

        # Manually invoke the image proocessing function and read the buffer
        # back to an image for testing.
        image_buffer = lcm_image_array_viewer._process_message()
        pil_image = Image.open(BytesIO(image_buffer))
        # PIL Image returns the size as (width, height).
        self.assertEqual(pil_image.size, (3, 1))

    def test_concatenate_images(self):
        """Checks the dimension of the image after image concatenation."""
        image_width = 2
        image_height = 3
        # Initialize the images with a specific value.
        test_images = [
            ImageRgba8U(image_width, image_height, 11) for i in range(4)
        ]

        test_row_col = [(1, 4), (4, 1), (2, 2)]
        for row, col in test_row_col:
            result = LcmImageArrayViewer._concatenate_images(
                test_images, row, col
            )
            self.assertEqual(
                result.shape, (image_height * row, image_width * col, 4)
            )
            self.assertTrue(np.all(result == 11))
