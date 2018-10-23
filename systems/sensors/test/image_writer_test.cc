#include "drake/systems/sensors/image_writer.h"

#include <unistd.h>

#include <fstream>
#include <set>
#include <string>

#include <gtest/gtest.h>
#include <spruce.hh>
#include <vtkImageData.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <vtkSmartPointer.h>
#include <vtkTIFFReader.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/event_collection.h"

// NOTE: This is a limited test of ImageWriter functionality.
namespace drake {
namespace systems {
namespace sensors {

// Friend class to get access to ImageWriter private functions for testing.
class ImageWriterTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageWriterTester)

  explicit ImageWriterTester(const ImageWriter& writer) : writer_(writer) {}

  bool can_write() const { return writer_.can_write_; }

  std::string make_file_name(const std::string& type_name, int i,
                             const std::string& ext) const {
    return writer_.make_file_name(type_name, i, ext);
  }

  int previous_frame_index() const {
    return writer_.previous_frame_index_;
  }

  double start_time() const {
    return writer_.start_time_;
  }

 private:
  const ImageWriter& writer_;
};

namespace {

// Class for testing actual I/O work. This helps manage generated files by
// facilitating temporary file names and registering additional names so that
// they can be cleaned up at the conclusion of the tests.
class ImageWriterTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    ASSERT_TRUE(ImageWriterTester(ImageWriter(temp_dir())).can_write());
  }

  static void TearDownTestCase() {
    for (const auto& file_name : files_) {
      spruce::path file_path(file_name);
      if (file_path.exists()) {
        // We'll consider a failure to delete a temporary file as a test
        // failure.
        unlink(file_path.getStr().c_str());
        EXPECT_FALSE(file_path.exists())
                  << "Failed to delete temporary test file: " << file_name;
      }
    }
  }

  // This assumes that the temp_directory() API will *always* return the same
  // name.
  static std::string temp_dir() { return temp_directory(); }

  // Returns a unique temporary image name - every requested name will be
  // examined at tear down for deletion. When it comes to writing images, all
  // names should come from here.
  static std::string temp_name() {
    spruce::path temp_path;
    do {
      temp_path.setStr(temp_dir());
      temp_path.append("image_writer_test_" + std::to_string(++img_count_) +
          ".png");
    } while (temp_path.exists());
    files_.insert(temp_path.getStr());
    return temp_path.getStr();
  }

  // Arbitrary files that are generated can be added to the set of files that
  // require clean up. This should be invoked for _every_ file generated in this
  // test suite.
  static void add_file_for_cleanup(const std::string& file_name) {
    files_.insert(file_name);
  }

  template <PixelType kPixelType>
  ::testing::AssertionResult ReadImage(const std::string& image_name,
                                       Image<kPixelType>* image) {
    spruce::path image_path(image_name);
    if (image_path.exists()) {
      vtkSmartPointer<vtkImageReader2> reader;
      switch (kPixelType) {
        case PixelType::kRgba8U:
        case PixelType::kLabel16I:
          reader = vtkSmartPointer<vtkPNGReader>::New();
          break;
        case PixelType::kDepth32F:
          reader = vtkSmartPointer<vtkTIFFReader>::New();
          break;
        default:
          return ::testing::AssertionFailure()
              << "Trying to read an unknown image type";
      }
      reader->SetFileName(image_name.c_str());
      vtkNew<vtkImageExport> png_exporter;
      png_exporter->SetInputConnection(reader->GetOutputPort());
      png_exporter->Update();
      vtkImageData *image_data = png_exporter->GetInput();
      // Assumes 1-dimensional data -- the 4x1 image.
      if (image_data->GetDataDimension() == 1) {
        int read_width;
        image_data->GetDimensions(&read_width);
        if (read_width == image->width()) {
          png_exporter->Export(image->at(0, 0));
          return ::testing::AssertionSuccess();
        }
      }
      int dims[3];
      png_exporter->GetDataDimensions(&dims[0]);
      return ::testing::AssertionFailure()
          << "Expected a " << image->width() << "x" << image->height()
          << "image. Read an image of size: " << dims[0] << "x" << dims[1]
          << "x" << dims[2];
    } else {
      return ::testing::AssertionFailure()
          << "The image to be read does not exist: " << image_name;
    }
  }

  // Generates a simple, known color image.
  static ImageRgba8U test_color_image() {
    // Creates a simple 4x1 image consisting of: [red][green][blue][white].
    ImageRgba8U color_image(4, 1);
    auto set_color = [&color_image](int x, int y, uint8_t r, uint8_t g,
                                    uint8_t b) {
      color_image.at(x, y)[0] = r;
      color_image.at(x, y)[1] = g;
      color_image.at(x, y)[2] = b;
      color_image.at(x, y)[3] = 255;
    };
    set_color(0, 0, 255, 0, 0);
    set_color(1, 0, 0, 255, 0);
    set_color(2, 0, 0, 0, 255);
    set_color(3, 0, 255, 255, 255);
    return color_image;
  }

 private:
  // NOTE: These are static so that they are shared across the entire test
  // suite. This allows all tests to have non-conflicting names *and* get
  // cleaned up when the test suite shuts down.

  // This presumes the tests in a single test case do *not* run in parallel.
  static int img_count_;

  // Files that may need to be cleaned up. It _must_ include every file that has
  // been created by these tests but *may* include purely speculative file
  // names.
  static std::set<std::string> files_;
};

int  ImageWriterTest::img_count_{-1};
std::set<std::string> ImageWriterTest::files_;

// Tests the logic for formatting images.
TEST_F(ImageWriterTest, FileNameFormatting) {
  auto test_file_name = [](const ImageWriter& writer,
                           const std::string expected, int frame = 0) {
    const std::string path =
        ImageWriterTester(writer).make_file_name("type", frame, "png");
    EXPECT_EQ(path, expected);
  };

  // Test various folder paths.
  test_file_name(ImageWriter("", "base"), "/base_type_0000.png");
  test_file_name(ImageWriter("/", "base"), "/base_type_0000.png");
  test_file_name(ImageWriter("test", "base"), "test/base_type_0000.png");
  test_file_name(ImageWriter("test/", "base"), "test/base_type_0000.png");
  test_file_name(ImageWriter("/test", "base"), "/test/base_type_0000.png");

  // Test various padding values.
  const double start_time = 0;
  test_file_name(ImageWriter("", "base", start_time, 0), "/base_type_0.png");
  test_file_name(ImageWriter("", "base", start_time, 1), "/base_type_0.png");
  test_file_name(ImageWriter("", "base", start_time, 2), "/base_type_00.png");
  test_file_name(ImageWriter("", "base", start_time, 5),
                 "/base_type_00000.png");

  // Confirm it uses the provided frame (and not a hard-coded zero).
  test_file_name(ImageWriter("", "base"), "/base_type_0001.png", 1);
  test_file_name(ImageWriter("", "base"), "/base_type_0100.png", 100);
  test_file_name(ImageWriter("", "base"), "/base_type_12345.png", 12345);
}

// Test initialization of various non-name related fields.
TEST_F(ImageWriterTest, ConstructorNonNamed) {
  {
    ImageWriterTester png_tester{ImageWriter(temp_dir())};
    EXPECT_EQ(png_tester.previous_frame_index(), -1);
    EXPECT_EQ(png_tester.start_time(), 0.0);  // default value.
  }

  {
    const double start_time = 1.75;
    ImageWriterTester png_tester(ImageWriter(temp_dir(), "name", start_time));
    EXPECT_EQ(png_tester.previous_frame_index(), -1);
    EXPECT_EQ(png_tester.start_time(), start_time);
  }
}

// Tests the writeability of a image writer based on the validity of the
// directory path.
TEST_F(ImageWriterTest, CanWrite) {
  // Case: Non-existent directory.
  {
    ImageWriter writer("this/path/does/not_exist");
    EXPECT_FALSE(ImageWriterTester(writer).can_write());
  }

  // Case: No write permissions (assuming that this isn't run as root).
  {
    ImageWriter writer("/root");
    EXPECT_FALSE(ImageWriterTester(writer).can_write());
  }

  // Case: the path is to a file.
  {
    const std::string file_name = temp_name();
    std::ofstream stream(file_name, std::ios::out);
    ASSERT_FALSE(stream.fail());
    ImageWriter writer(file_name);
    EXPECT_FALSE(ImageWriterTester(writer).can_write());
  }
}

// Confirms publishing semantics.
TEST_F(ImageWriterTest, ConfigurePublishPeriod) {
  // Case: valid working directory.
  {
    ImageWriter writer(temp_dir());

    // Freshly constructed, the writer has no timed events.
    auto events = writer.AllocateCompositeEventCollection();
    auto context = writer.AllocateContext();
    writer.CalcNextUpdateTime(*context, events.get());
    EXPECT_FALSE(events->HasEvents());

    // Invalid period value throws an exception and, god forbid someone catches
    // the exception, it has no timed events.
    DRAKE_EXPECT_THROWS_MESSAGE(writer.set_publish_period(-1), std::logic_error,
                                "ImageWriter requires a positive period value");
    events->Clear();    // Should already be clear; but we're just being safe.
    writer.CalcNextUpdateTime(*context, events.get());
    EXPECT_FALSE(events->HasEvents());

    // With a valid period value, it should report a timed event.
    EXPECT_NO_THROW(writer.set_publish_period(1));
    events->Clear();    // Should already be clear; but we're just being safe.
    writer.CalcNextUpdateTime(*context, events.get());
    EXPECT_TRUE(events->HasEvents());
  }

  // Case: invalid working directory.
  {
    ImageWriter writer("/this/is/garbage");

    // Freshly constructed, the writer has no timed events.
    auto events = writer.AllocateCompositeEventCollection();
    auto context = writer.AllocateContext();

    // A valid period should not throw an exception, but no timed events should
    // be created.
    EXPECT_NO_THROW(writer.set_publish_period(1));
    writer.CalcNextUpdateTime(*context, events.get());
    EXPECT_FALSE(events->HasEvents());
  }
}

// Evaluate the stand-alone test for color images.
TEST_F(ImageWriterTest, SaveToPng_Color) {
  const std::string color_image_name = temp_name();
  ImageRgba8U color_image = test_color_image();
  SaveToPng(color_image_name, color_image);

  ImageRgba8U read_image(color_image.width(), color_image.height());
  ASSERT_TRUE(ReadImage(color_image_name, &read_image));
  for (int u = 0; u < 4; ++u) {
    for (int c = 0; c < 4; ++c) {
      EXPECT_EQ(read_image.at(u, 0)[c], color_image.at(u, 0)[c]);
    }
  }
}

// Evaluate the stand-alone test for depth images.
TEST_F(ImageWriterTest, SaveToTiff_Depth) {
  // Creates a simple 4x1 image consisting of: 0, 0.25, 0.5, 0.75
  ImageDepth32F depth_image(4, 1);
  *depth_image.at(0, 0) = 0.0f;
  *depth_image.at(1, 0) = 0.25f;
  *depth_image.at(2, 0) = 0.5f;
  *depth_image.at(3, 0) = 1.0f;

  const std::string depth_image_name = temp_name();
  SaveToTiff(depth_image_name, depth_image);

  ImageDepth32F read_image(depth_image.width(), depth_image.height());
  ASSERT_TRUE(ReadImage(depth_image_name, &read_image));
  for (int u = 0; u < 4; ++u) {
    EXPECT_EQ(read_image.at(u, 0)[0], depth_image.at(u, 0)[0]);
  }
}

// Evaluate the stand-alone test for label images.
TEST_F(ImageWriterTest, SaveToPng_Label) {
  // Creates a simple 4x1 image consisting of: 0, 1, 2, 3.
  ImageLabel16I label_image(4, 1);
  *label_image.at(0, 0) = 0;
  *label_image.at(1, 0) = 1;
  *label_image.at(2, 0) = 2;
  *label_image.at(3, 0) = 3;

  const std::string label_image_name = temp_name();
  SaveToPng(label_image_name, label_image);

  ImageLabel16I read_image(label_image.width(), label_image.height());
  ASSERT_TRUE(ReadImage(label_image_name, &read_image));
  for (int u = 0; u < 4; ++u) {
    EXPECT_EQ(read_image.at(u, 0)[0], label_image.at(u, 0)[0]);
  }
}

// Confirms that writing increments the counter.
TEST_F(ImageWriterTest, WriteIncrementsCounter) {
  ImageWriter writer(temp_dir(), "increment_test");
  auto context = writer.AllocateContext();
  context->FixInputPort(
      writer.color_image_input_port().get_index(),
      AbstractValue::Make<ImageRgba8U>(test_color_image()));
  ImageWriterTester tester(writer);

  int expected_previous = -1;
  for (int i = 0; i < 2; ++i) {
    // Confirms initial conditions.
    ASSERT_EQ(tester.previous_frame_index(), expected_previous);
    const std::string first_file =
        fmt::format("increment_test_color_{0:04}.png", ++expected_previous);
    spruce::path file_path(temp_dir());
    file_path.append(first_file);
    ASSERT_FALSE(file_path.exists());

    // Publish.
    writer.Publish(*context);
    add_file_for_cleanup(file_path.getStr());

    // Confirm expected state.
    ASSERT_EQ(tester.previous_frame_index(), expected_previous);
    ASSERT_TRUE(file_path.exists());
  }
}

// Confirms that calling publish with contexts before start time do not write
// images (as detected by incrementing the counter).
TEST_F(ImageWriterTest, NoWritingBeforeStartTime) {
  const double start_time = 10;
  ImageWriter writer(temp_dir(), "start_time_test", start_time);
  auto context = writer.AllocateContext();
  context->FixInputPort(
      writer.color_image_input_port().get_index(),
      AbstractValue::Make<ImageRgba8U>(test_color_image()));
  ImageWriterTester tester(writer);

  // Confirms initial conditions.
  ASSERT_EQ(tester.previous_frame_index(), -1);
  const std::string first_file = "start_time_test_color_0000.png";
  spruce::path file_path(temp_dir());
  file_path.append(first_file);
  ASSERT_FALSE(file_path.exists());

  const int kCount = 10;
  for (int i = 0; i < kCount; ++i) {
    // Guarantee that t < start_time. start_time / (kCount + 1)
    const double t = start_time / kCount * i;
    context->set_accuracy(t);
    writer.Publish(*context);
    // If images are *actually* being written, we want to stop this test
    // *immediately*.
    ASSERT_EQ(tester.previous_frame_index(), -1);
    ASSERT_FALSE(file_path.exists());
  }

  context->set_time(start_time);
  writer.Publish(*context);
  add_file_for_cleanup(file_path.getStr());

  ASSERT_EQ(tester.previous_frame_index(), 0);
  ASSERT_TRUE(file_path.exists());
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
