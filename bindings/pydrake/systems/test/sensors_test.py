import numpy as np
import unittest

import pydrake.systems.sensors as m

# Shorthand aliases, to reduce verbosity.
pt = m.PixelType
pf = m.PixelFormat

# Available image / pixel types.
pixel_types = [
    pt.kRgba8U,
    pt.kDepth32F,
    pt.kLabel16I,
]


class TestSensors(unittest.TestCase):
    def test_image_traits(self):
        t = m.ImageTraits[pt.kRgba8U]
        self.assertEquals(t.kNumChannels, 4)
        self.assertEquals(t.ChannelType, np.uint8)
        self.assertEquals(t.kPixelFormat, pf.kRgba)

        t = m.ImageTraits[pt.kDepth32F]
        self.assertEquals(t.kNumChannels, 1)
        self.assertEquals(t.ChannelType, np.float32)
        self.assertEquals(t.kPixelFormat, pf.kDepth)

        t = m.ImageTraits[pt.kLabel16I]
        self.assertEquals(t.kNumChannels, 1)
        self.assertEquals(t.ChannelType, np.int16)
        self.assertEquals(t.kPixelFormat, pf.kLabel)

    def test_image_shape(self):
        for pixel_type in pixel_types:
            ImageT = m.Image[pixel_type]
            self.assertEquals(ImageT.Traits, m.ImageTraits[pixel_type])

            w = 640
            h = 480
            nc = ImageT.Traits.kNumChannels
            image = ImageT(w, h)
            self.assertEquals(image.width(), w)
            self.assertEquals(image.height(), h)
            self.assertEquals(image.size(), h * w * nc)
            # N.B. Since `shape` is a custom-Python extension, it's defined as
            # a property (not a function).
            self.assertEquals(image.shape, (h, w, nc))
            self.assertEquals(image.data.dtype, ImageT.Traits.ChannelType)

            w /= 2
            h /= 2
            # WARNING: Resizing an image with an existing reference to
            # `image.data` will cause `image.data` + `image.mutable_data` to be
            # invalid.
            image.resize(w, h)
            self.assertEquals(image.shape, (h, w, nc))

    def test_image_data(self):
        for pixel_type in pixel_types:
            # Use a trivial size for ease of debugging.
            w = 8
            h = 6
            channel_default = 1
            ImageT = m.Image[pixel_type]
            image = ImageT(w, h, channel_default)
            nc = ImageT.Traits.kNumChannels

            # Test default initialization.
            self.assertEquals(image.at(0, 0, 0), channel_default)
            self.assertEquals(image.data.shape, image.shape)
            self.assertTrue(np.allclose(image.data, channel_default))

            # Test mutation and pixel / channel access.
            image.set(0, 0, 0, 2)
            self.assertEquals(image.at(0, 0, 0), 2)

            # Test numpy views, access and mutation.
            image.mutable_data[:] = 3
            self.assertEquals(image.at(0, 0, 0), 3)
            self.assertTrue(np.allclose(image.data, 3))
            self.assertTrue(np.allclose(image.mutable_data, 3))

            # Ensure that each dimension of the image array is unique.
            self.assertEquals(len(set(image.shape)), 3)
            # Ensure indices match as expected. Fill each channel at each pixel
            # with unique values, and ensure that pixel / channels map
            # appropriately.
            data = image.mutable_data
            data[:] = np.arange(0, image.size()).reshape(image.shape)
            for iw in range(w):
                for ih in range(h):
                    for ic in range(nc):
                        assert data[ih, iw, ic] == image.at(iw, ih, ic)

    def test_constants(self):
        # Simply ensure we can access the constants.
        values = [
            m.InvalidDepth.kTooFar,
            m.InvalidDepth.kTooClose,
            m.Label.kNoBody,
            m.Label.kFlatTerrain,
        ]
        self.assertTrue(values)


if __name__ == '__main__':
    unittest.main()
