from io import BytesIO
import unittest

import numpy as np
from PIL import Image

from drake import lcmt_image, lcmt_image_array
from pydrake.systems.sensors import ImageRgba8U
from pydrake.visualization._lcm_image_array_viewer import LcmImageArrayViewer


class TestLcmImageArrayViewer(unittest.TestCase):
    def _get_rgba_lcmt_image(self):
        image_message = lcmt_image()
        image_message.pixel_format = lcmt_image.PIXEL_FORMAT_RGBA
        image_message.channel_type = lcmt_image.CHANNEL_TYPE_UINT8
        image_message.width = 3
        image_message.height = 2
        image_message.row_stride = 12
        image_message.size = 24

        image_data = np.ones((2, 3, 4), dtype=np.uint8)
        image_message.data = image_data.tobytes()
        return image_message

    def _get_depth_lcmt_image(self):
        image_message = lcmt_image()
        image_message.pixel_format = lcmt_image.PIXEL_FORMAT_DEPTH
        image_message.channel_type = lcmt_image.CHANNEL_TYPE_FLOAT32
        image_message.width = 3
        image_message.height = 2
        image_message.row_stride = 12
        image_message.size = 24

        image_data = np.array(
            [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32
        )
        image_message.data = image_data.tobytes()
        return image_message

    def _get_label_lcmt_image(self):
        image_message = lcmt_image()
        image_message.pixel_format = lcmt_image.PIXEL_FORMAT_LABEL
        image_message.channel_type = lcmt_image.CHANNEL_TYPE_INT16
        image_message.width = 3
        image_message.height = 2
        image_message.row_stride = 6
        image_message.size = 12

        image_data = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.int16)
        image_message.data = image_data.tobytes()
        return image_message

    def test_image_processing(self):
        """Publishes a sample image_array message, checks the dimension of the
        returned image and the pixel values.
        """
        lcm_image_array_viewer = LcmImageArrayViewer(
            host="localhost",
            port=1234,
            channel="does_not_matter",
            unit_test=True,
        )

        # Create an lcmt_image_array containing different types of images.
        array_message = lcmt_image_array()
        array_message.num_images = 4
        array_message.images = [
            self._get_rgba_lcmt_image(),
            self._get_depth_lcmt_image(),
            self._get_label_lcmt_image(),
            # Include two images of the same type.
            self._get_rgba_lcmt_image(),
        ]

        lcm = lcm_image_array_viewer._lcm
        lcm.Publish(channel="does_not_matter", buffer=array_message.encode())
        lcm.HandleSubscriptions(timeout_millis=1)

        # Manually invoke the image processing function and read the buffer
        # back to an image for testing.
        image_buffer = lcm_image_array_viewer._process_message()
        pil_image = Image.open(BytesIO(image_buffer))
        # PIL Image returns the size as (width, height).
        self.assertEqual(pil_image.size, (12, 2))

        # Sanity check the pixel values of the images. For RGBA, they should
        # remain exactly the same. For depth and label, we only check they have
        # at least six unique values after the colorization (to rule out
        # failure cases such as all-black pixels).
        np_image_data = np.array(pil_image)
        image_width = 3
        # RGBAs.
        np.testing.assert_equal(np_image_data[:, 0:image_width], 1)
        np.testing.assert_equal(np_image_data[:, -image_width:], 1)
        # Depth and label.
        np_depth_data = np_image_data[:, image_width: image_width * 2]
        np_label_data = np_image_data[:, image_width * 2: image_width * 3]
        depth_pixel_values = set(np_depth_data.flatten())
        label_pixel_values = set(np_label_data.flatten())
        self.assertGreaterEqual(len(depth_pixel_values), 6)
        self.assertGreaterEqual(len(label_pixel_values), 6)

    def test_concatenate_images(self):
        """Checks the pixel values and the dimension of the image after the
        concatenation.
        """
        image_width = 3
        image_height = 2
        pixel_value_base = 10

        # Initialize the images with a distinct value.
        test_images = []
        for index in range(4):
            image = ImageRgba8U(image_width, image_height)
            image.mutable_data[:] = pixel_value_base + index
            test_images.append(image)

        test_rows_cols = [(1, 4), (4, 1), (2, 2)]
        for rows, cols in test_rows_cols:
            result = LcmImageArrayViewer._concatenate_images(
                test_images, rows, cols
            )

            # The overall image dimension should match.
            self.assertEqual(
                result.shape, (image_height * rows, image_width * cols, 4)
            )

            # Each sub-image should remain the same pixel value.
            for r in range(rows):
                for c in range(cols):
                    expected_pixel_value = pixel_value_base + r * cols + c
                    sub_image = result[
                        r * image_height: (r + 1) * image_height,
                        c * image_width: (c + 1) * image_width,
                    ]
                    np.testing.assert_equal(sub_image, expected_pixel_value)
