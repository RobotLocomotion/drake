import argparse
from io import BytesIO
import zlib

from flask import Flask, Response
import numpy as np
from PIL import Image

from drake import lcmt_image, lcmt_image_array
from pydrake.lcm import DrakeLcm
from pydrake.systems.sensors import (
    ImageDepth16U,
    ImageDepth32F,
    ImageLabel16I,
    ImageRgba8U,
)
from pydrake.visualization import ColorizeDepthImage, ColorizeLabelImage


class _ImageServer(Flask):
    """Streams images via the HTTP protocol given an image source. The image
    source, i.e., `image_generator`, should be a generator function that yields
    a (mime_type, image_data) pair. The `mime-type` is a str, and the
    `image_data` is bytes representing the image.
    """

    def __init__(self, *, image_generator):
        super().__init__("meldis_lcm_image_viewer")
        self.add_url_rule("/", view_func=self._serve_image)

        self._image_generator = image_generator

    def _serve_image(self):
        return Response(
            self._response_generator(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    def _response_generator(self):
        for mime_type, image_data in self._image_generator():
            yield (
                b"--frame\r\nContent-Type: "
                + mime_type.encode("utf-8")
                + b"\r\n\r\n"
                + image_data
                + b"\r\n"
            )


class LcmImageArrayViewer:
    """Displays LCM images to an URL. The program waits for `lcmt_image_array`
    messages from a particular channel and processes them to image files. It
    contains a flask server, _ImageServer, that grabs images whenever available
    and broadcasts them to an URL for visualization.
    """

    _IMAGE_DATA_TYPE = {
        lcmt_image.CHANNEL_TYPE_UINT8: np.uint8,
        lcmt_image.CHANNEL_TYPE_INT16: np.int16,
        lcmt_image.CHANNEL_TYPE_UINT16: np.uint16,
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

    def __init__(self, *, host, port, channel, unit_test=False):
        # Only the latest message from LCM is kept.
        self._latest_message = None

        # Subscribe to the channel.
        self._lcm = DrakeLcm()
        self._lcm.Subscribe(channel=channel, handler=self._update_message)

        # Helpers to convert images to aid visualization.
        self._colorize_label = ColorizeLabelImage()
        self._colorize_depth = ColorizeDepthImage()

        # Instantiate an `_ImageServer` and run it. If `unit_test` is True, the
        # server will not be launched.
        if not unit_test:
            self._image_server = _ImageServer(
                image_generator=self.image_generator
            )
            self._image_server.run(
                host=host, port=port, debug=False, threaded=False
            )

    def image_generator(self):
        mime_type = "image/png"
        while True:
            self._lcm.HandleSubscriptions(timeout_millis=1000)
            if self._latest_message is not None:
                new_image = self._process_message()
                self._latest_message = None
                yield (mime_type, new_image)

    def _update_message(self, message):
        self._latest_message = message

    def _process_message(self):
        """Processes the latest lcmt_image_array message into a single PNG
        image. Depth and label images will be colorized to color images for
        visualization. If the LCM message contains multiple images, they will
        be concatenated together horizontally.
        """
        image_array = lcmt_image_array.decode(self._latest_message)
        assert len(image_array.images) > 0

        rgba_images = []
        for image in image_array.images:
            w = image.width
            h = image.height
            data_type = self._IMAGE_DATA_TYPE[image.channel_type]
            num_channels = self._IMAGE_CHANNEL_NUM[image.pixel_format]

            bytes_per_pixel = np.dtype(data_type).itemsize * num_channels
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
            np_image_data = np.frombuffer(data_bytes, dtype=data_type)

            rgba = ImageRgba8U(w, h)
            if image.pixel_format == lcmt_image.PIXEL_FORMAT_RGBA:
                rgba.mutable_data[:] = np_image_data.reshape(h, w, 4)
            elif image.pixel_format == lcmt_image.PIXEL_FORMAT_LABEL:
                label = ImageLabel16I(w, h)
                label.mutable_data[:] = np_image_data.reshape(h, w, 1)
                self._colorize_label.Calc(label, rgba)
            elif image.pixel_format == lcmt_image.PIXEL_FORMAT_DEPTH:
                if image.channel_type == lcmt_image.CHANNEL_TYPE_UINT16:
                    depth = ImageDepth16U(w, h)
                elif image.channel_type == lcmt_image.CHANNEL_TYPE_FLOAT32:
                    depth = ImageDepth32F(w, h)
                else:
                    raise RuntimeError(
                        f"Unsupported depth pixel format: {image.pixel_format}"
                    )
                depth.mutable_data[:] = np_image_data.reshape(h, w, 1)
                self._colorize_depth.Calc(depth, rgba)
            rgba_images.append(rgba)

        # Stack the images horizontally.
        np_concatenated_image = self._concatenate_images(
            rgba_images, rows=1, cols=len(rgba_images)
        )

        # Save the image in-memory.
        pil_image = Image.fromarray(np_concatenated_image)
        buffer = BytesIO()
        pil_image.save(buffer, format="png", compress_level=0)
        return buffer.getbuffer()

    @staticmethod
    def _concatenate_images(images, rows, cols):
        """Helper function to concatenate multiple images. It is assumed that
        `images` to be a list of systems::sensors::Image with the same size.
        """
        assert len(images) == rows * cols
        col_images = []
        for r in range(rows):
            row_images = []
            for c in range(cols):
                image = images[r * cols + c]
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
        host=args.host, port=args.port, channel=args.channel
    )


if __name__ == "__main__":
    main()
