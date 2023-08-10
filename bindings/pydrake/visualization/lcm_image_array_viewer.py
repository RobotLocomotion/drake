import argparse
from io import BytesIO
import zlib

from flask import Flask, Response
import numpy as np
from PIL import Image

from drake import lcmt_image, lcmt_image_array
from pydrake.lcm import DrakeLcm
from pydrake.systems.sensors import ImageDepth32F, ImageLabel16I, ImageRgba8U
from pydrake.visualization import ColorizeDepthImage, ColorizeLabelImage


class _ImageServer(Flask):
    """Streams images via the HTTP protocol given an image source. The image
    source should be a continuously running generator function that yields
    images sequentially.
    """

    def __init__(self, *, image_generator):
        super().__init__("meldis_lcm_image_viewer")
        self.add_url_rule("/", view_func=self._serve_image)

        self._image_generator = image_generator

    def _serve_image(self):
        return Response(
            self._image_generator(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )


class LcmImageArrayViewer:
    """Displays LCM images to an URL. The program waits for `lcmt_image_arry`
    messages from a particular channel and processes them to image files. It
    contains a flask server, _ImageServer, that grabs images whenever available
    and broadcasts them to an URL for visualization.
    """

    _IMAGE_DATA_TYPE = {
        lcmt_image.CHANNEL_TYPE_UINT8: np.uint8,
        lcmt_image.CHANNEL_TYPE_INT16: np.int16,
        lcmt_image.CHANNEL_TYPE_FLOAT32: np.float32,
    }
    """The mapping from `lcmt_image` channel_type enum to numpy data type."""

    _IMAGE_CHANNEL_NUM = {
        lcmt_image.PIXEL_FORMAT_RGBA: 4,
        lcmt_image.PIXEL_FORMAT_DEPTH: 1,
        lcmt_image.PIXEL_FORMAT_LABEL: 1,
    }
    """The mapping from `lcmt_image` pixel_format enum to the number of
    channels.
    """

    def __init__(self, host, port, channel):
        # Only the latest message from LCM is kept.
        self._latest_message = None

        # Subscribe to the channel.
        self._lcm = DrakeLcm()
        self._lcm.Subscribe(channel=channel, handler=self._update_message)

        # Helpers to convert images to aid visualization.
        self._colorized_label = ColorizeLabelImage()
        self._colorized_depth = ColorizeDepthImage()

        # Instantiate an `_ImageServer` and run it.
        self._image_server = _ImageServer(image_generator=self.image_generator)
        self._image_server.run(
            host=host, port=port, debug=False, threaded=False
        )

    def image_generator(self):
        while True:
            self._lcm.HandleSubscriptions(timeout_millis=1000)
            if self._latest_message is not None:
                new_image = self._process_message()
                self._latest_message = None
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/png\r\n\r\n" + new_image + b"\r\n"
                )

    def _update_message(self, message):
        self._latest_message = message

    def _process_message(self):
        """Processes the latest lcmt_image_array message into a PNG image. If
        the message contains multiple lcmt_image images, they will be
        concatenated together.
        """
        image_array = lcmt_image_array.decode(self._latest_message)
        assert len(image_array.images) > 0

        rgba_images = []
        for image in image_array.images:
            w = image.width
            h = image.height

            # Validate the image.
            data_type = self._IMAGE_DATA_TYPE.get(image.channel_type, None)
            num_channels = self._IMAGE_CHANNEL_NUM.get(
                image.pixel_format, None
            )
            bytes_per_pixel = np.dtype(data_type).itemsize * num_channels
            assert data_type, image.channel_type
            assert num_channels, image.pixel_format
            assert image.row_stride == w * bytes_per_pixel, image.row_stride

            if (
                image.compression_method
                == lcmt_image.COMPRESSION_METHOD_NOT_COMPRESSED
            ):
                data_bytes = image.data
            elif (
                image.compression_method == lcmt_image.COMPRESSION_METHOD_ZLIB
            ):
                # TODO(eric): Consider using `data`s buffer, if possible.
                # Can decompress() somehow use an existing buffer in Python?
                data_bytes = zlib.decompress(image.data)
            else:
                raise RuntimeError(
                    f"Unsupported compression type:{image.compression_method}"
                )

            # Convert bytes to np.array for colorization or concatenation.
            np_image_data = np.frombuffer(data_bytes, dtype=data_type)

            rgba = ImageRgba8U(w, h, 4)
            if image.pixel_format == lcmt_image.PIXEL_FORMAT_RGBA:
                rgba.mutable_data[:] = np_image_data.reshape(h, w, 4)
            elif image.pixel_format == lcmt_image.PIXEL_FORMAT_LABEL:
                label = ImageLabel16I(w, h, 1)
                label.mutable_data[:] = np_image_data.reshape(h, w, 1)
                self._colorized_label._colorize_label_image(label, rgba)
            elif image.pixel_format == lcmt_image.PIXEL_FORMAT_DEPTH:
                depth = ImageDepth32F(w, h, 1)
                depth.mutable_data[:] = np_image_data.reshape(h, w, 1)
                self._colorized_depth._colorize_depth_image(depth, rgba)
            rgba_images.append(rgba)

        # Stack the images vertically.
        display_image = LcmImageArrayViewer.concatenate_images(
            rgba_images, len(rgba_images), 1
        )

        # Save the image in-memory.
        pil_image = Image.fromarray(display_image)
        buffer = BytesIO()
        pil_image.save(buffer, format="png", compress_level=0)
        return buffer.getbuffer()

    @staticmethod
    def concatenate_images(images, row, col):
        """Helper function to concatenate multiple images. `images` is assumed
        to be a list of systems::sensros::Image with the same image size.
        """
        assert len(images) == row * col
        col_images = []
        for i in range(row):
            row_images = []
            for j in range(col):
                image = images[i * col + j]
                row_images.append(image.data)
            row_image = np.hstack(row_images)
            col_images.append(row_image)
        return np.vstack(col_images)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
    )
    parser.add_argument(
        "--host",
        type=str,
        required=False,
        default="127.0.0.1",
        help="URL to host on, default: 127.0.0.1.",
    )
    parser.add_argument(
        "--port",
        type=int,
        required=False,
        default=8000,
        help="Port to host on, default: 8000.",
    )
    parser.add_argument(
        "--channel",
        type=str,
        required=True,
        help="The LCM channel to subscribe to.",
    )
    args = parser.parse_args()

    image_array_viewer = LcmImageArrayViewer(
        args.host, args.port, args.channel
    )


if __name__ == "__main__":
    main()
