#include "drake/systems/sensors/image_writer.h"

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/event_collection.h"
#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace systems {
namespace sensors {

namespace fs = std::filesystem;

// Friend class to get access to ImageWriter private functions for testing.
class ImageWriterTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImageWriterTester);

  explicit ImageWriterTester(const ImageWriter& writer) : writer_(writer) {}

  std::string DirectoryFromFormat(const std::string& format,
                                  const std::string& port_name,
                                  PixelType pixel_type) const {
    return writer_.DirectoryFromFormat(format, port_name, pixel_type);
  }

  static bool IsDirectoryValid(const std::string& file_path) {
    return ImageWriter::ValidateDirectory(file_path) ==
           ImageWriter::FolderState::kValid;
  }

  static bool DirectoryIsMissing(const std::string& file_path) {
    return ImageWriter::ValidateDirectory(file_path) ==
           ImageWriter::FolderState::kMissing;
  }

  static bool DirectoryIsFile(const std::string& file_path) {
    return ImageWriter::ValidateDirectory(file_path) ==
           ImageWriter::FolderState::kIsFile;
  }

  static bool DirectoryIsUnwritable(const std::string& file_path) {
    return ImageWriter::ValidateDirectory(file_path) ==
           ImageWriter::FolderState::kUnwritable;
  }

  std::string MakeFileName(const std::string& format, PixelType pixel_type,
                           double time, const std::string& port_name,
                           int count) const {
    return writer_.MakeFileName(format, pixel_type, time, port_name, count);
  }

  const std::string& port_format(int port_index) const {
    return writer_.port_info_[port_index].format;
  }

  int port_count(int port_index) const {
    return writer_.port_info_[port_index].count;
  }

  const std::string& label(PixelType pixel_type) const {
    return writer_.labels_.at(pixel_type);
  }

  const std::string& extension(PixelType pixel_type) const {
    return writer_.extensions_.at(pixel_type);
  }

 private:
  const ImageWriter& writer_;
};

namespace {

// Utility functions for creating test, reference images

template <PixelType kPixelType>
static Image<kPixelType> test_image() {
  throw std::logic_error("No default implementation");
}

template <>
Image<PixelType::kRgba8U> test_image() {
  // Creates a simple 4x1 image consisting of: [red][green][blue][white].
  Image<PixelType::kRgba8U> color_image(4, 1);
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

template <>
Image<PixelType::kDepth32F> test_image() {
  // Creates a simple 4x1 image consisting of: 0, 0.25, 0.5, 0.75
  Image<PixelType::kDepth32F> depth_image(4, 1);
  *depth_image.at(0, 0) = 0.0f;
  *depth_image.at(1, 0) = 0.25f;
  *depth_image.at(2, 0) = 0.5f;
  *depth_image.at(3, 0) = 1.0f;
  return depth_image;
}

template <>
Image<PixelType::kLabel16I> test_image() {
  // Creates a simple 4x1 image consisting of: 0, 100, 200, 300.
  Image<PixelType::kLabel16I> label_image(4, 1);
  *label_image.at(0, 0) = 0;
  *label_image.at(1, 0) = 100;
  *label_image.at(2, 0) = 200;
  // Note: value > 255 to make sure that values aren't being truncated/wrapped
  // to 8-bit values.
  *label_image.at(3, 0) = 300;
  return label_image;
}

template <>
Image<PixelType::kDepth16U> test_image() {
  // Creates a simple 4x1 image consisting of: 0, 100, 200, 300.
  Image<PixelType::kDepth16U> depth_image(4, 1);
  *depth_image.at(0, 0) = 0;
  *depth_image.at(1, 0) = 100;
  *depth_image.at(2, 0) = 200;
  // Note: value > 255 to make sure that values aren't being truncated/wrapped
  // to 8-bit values.
  *depth_image.at(3, 0) = 300;
  return depth_image;
}

template <>
Image<PixelType::kGrey8U> test_image() {
  // Creates a simple 4x1 image consisting of: 1, 2, 3, 4.
  Image<PixelType::kGrey8U> grey_imageimage(4, 1);
  *grey_imageimage.at(0, 0) = 1;
  *grey_imageimage.at(1, 0) = 2;
  *grey_imageimage.at(2, 0) = 3;
  *grey_imageimage.at(3, 0) = 4;
  return grey_imageimage;
}

// Class for testing actual I/O work. This helps manage generated files by
// facilitating temporary file names and registering additional names so that
// they can be cleaned up at the conclusion of the tests.
class ImageWriterTest : public ::testing::Test {
 public:
  static void SetUpTestCase() {
    ASSERT_TRUE(ImageWriterTester::IsDirectoryValid(temp_dir()));
  }

  static void TearDownTestCase() {
    for (const auto& file_name : files_) {
      if (fs::exists({file_name})) {
        // We'll consider a failure to delete a temporary file as a test
        // failure.
        unlink(file_name.c_str());
        EXPECT_FALSE(fs::exists({file_name}))
            << "Failed to delete temporary test file: " << file_name;
      }
    }
  }

  // This assumes that the temp_directory() API will *always* return the same
  // name during the execution of this test.
  static std::string temp_dir() { return temp_directory(); }

  // Returns a unique temporary image name - every requested name will be
  // examined at tear down for deletion. When it comes to writing images, all
  // names should come from here.
  static std::string temp_name() {
    fs::path temp_path;
    do {
      temp_path = temp_dir();
      temp_path.append("image_writer_test_" + std::to_string(++img_count_) +
                       ".png");
    } while (fs::exists(temp_path));
    files_.insert(temp_path.string());
    return temp_path.string();
  }

  // Arbitrary files that are generated can be added to the set of files that
  // require clean up. This should be invoked for _every_ file generated in this
  // test suite.
  static void add_file_for_cleanup(const std::string& file_name) {
    files_.insert(file_name);
  }

  template <PixelType kPixelType>
  static void TestWritingImageOnPort() {
    ImageWriter writer;
    ImageWriterTester tester(writer);

    // Values for port declaration.
    const double period = 1 / 10.0;  // 10 Hz.
    const double start_time = 0.25;
    const std::string port_name = "port";
    fs::path path(temp_dir());
    path.append("{image_type}_{time_usec}");

    Image<kPixelType> image = test_image<kPixelType>();
    const auto& port = writer.DeclareImageInputPort<kPixelType>(
        port_name, path.string(), period, start_time);
    auto events = writer.AllocateCompositeEventCollection();
    auto context = writer.AllocateContext();
    port.FixValue(context.get(), image);
    context->SetTime(0.);
    writer.CalcNextUpdateTime(*context, events.get());

    const std::string expected_name = tester.MakeFileName(
        tester.port_format(port.get_index()), kPixelType, context->get_time(),
        port_name, tester.port_count(port.get_index()));
    fs::path expected_file(expected_name);
    EXPECT_FALSE(fs::exists(expected_file));
    const EventStatus status =
        writer.Publish(*context, events->get_publish_events());
    EXPECT_TRUE(status.succeeded());
    EXPECT_TRUE(fs::exists(expected_file));
    EXPECT_EQ(1, tester.port_count(port.get_index()));
    add_file_for_cleanup(expected_file.string());

    Image<kPixelType> readback;
    ASSERT_TRUE(LoadImage(expected_name, &readback));
    EXPECT_EQ(readback, image);
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

int ImageWriterTest::img_count_{-1};
std::set<std::string> ImageWriterTest::files_;

// ImageWriter contains a number of mappings between pixel type and work strings
// (extensions and image_type values for format args, this confirms that they
// are mapped correctly.
TEST_F(ImageWriterTest, ImageToStringMaps) {
  ImageWriter writer;
  ImageWriterTester tester(writer);

  EXPECT_EQ("color", tester.label(PixelType::kRgba8U));
  EXPECT_EQ("label", tester.label(PixelType::kLabel16I));
  EXPECT_EQ("depth", tester.label(PixelType::kDepth32F));
  EXPECT_EQ("depth", tester.label(PixelType::kDepth16U));
  EXPECT_EQ("grey_scale", tester.label(PixelType::kGrey8U));

  EXPECT_EQ(".png", tester.extension(PixelType::kRgba8U));
  EXPECT_EQ(".png", tester.extension(PixelType::kLabel16I));
  EXPECT_EQ(".tiff", tester.extension(PixelType::kDepth32F));
  EXPECT_EQ(".png", tester.extension(PixelType::kDepth16U));
  EXPECT_EQ(".png", tester.extension(PixelType::kGrey8U));
}

// Tests the processing of file format for extracting the directory.
TEST_F(ImageWriterTest, DirectoryFromFormat) {
  ImageWriter writer;
  ImageWriterTester tester{writer};

  DRAKE_EXPECT_THROWS_MESSAGE(
      tester.DirectoryFromFormat("", "port_name", PixelType::kRgba8U),
      ".*empty.*");
  EXPECT_EQ(
      "", tester.DirectoryFromFormat("/root", "port_name", PixelType::kRgba8U));
  DRAKE_EXPECT_THROWS_MESSAGE(
      tester.DirectoryFromFormat("/root/", "port_name", PixelType::kRgba8U),
      ".*cannot end with a '/'");
  EXPECT_EQ("/root", tester.DirectoryFromFormat("/root/file", "port_name",
                                                PixelType::kRgba8U));
  // Don't use all three image types; the FileNameFormatting test already
  // tests those permutations. We just want to make sure it's engaged here.
  EXPECT_EQ("/root/color",
            tester.DirectoryFromFormat("/root/{image_type}/file", "port_name",
                                       PixelType::kRgba8U));
  EXPECT_EQ("/root/my_port",
            tester.DirectoryFromFormat("/root/{port_name}/file", "my_port",
                                       PixelType::kRgba8U));

  // Test against invalid formatting arguments.
  DRAKE_EXPECT_THROWS_MESSAGE(
      tester.DirectoryFromFormat("/root/{count}/file", "port",
                                 PixelType::kRgba8U),
      ".*The directory path cannot include time or image count");
  DRAKE_EXPECT_THROWS_MESSAGE(
      tester.DirectoryFromFormat("/root/{time_double}/file", "port",
                                 PixelType::kRgba8U),
      ".*The directory path cannot include time or image count");
  DRAKE_EXPECT_THROWS_MESSAGE(
      tester.DirectoryFromFormat("/root/{time_usec}/file", "port",
                                 PixelType::kRgba8U),
      ".*The directory path cannot include time or image count");
  DRAKE_EXPECT_THROWS_MESSAGE(
      tester.DirectoryFromFormat("/root/{time_msec}/file", "port",
                                 PixelType::kRgba8U),
      ".*The directory path cannot include time or image count");

  // Make sure it's not fooled by strings that are *almost* format arguments.
  EXPECT_EQ("/root/time_double",
            tester.DirectoryFromFormat("/root/time_double/file", "my_port",
                                       PixelType::kRgba8U));
  EXPECT_EQ("/root/time_usec",
            tester.DirectoryFromFormat("/root/time_usec/file", "my_port",
                                       PixelType::kRgba8U));
  EXPECT_EQ("/root/time_msec",
            tester.DirectoryFromFormat("/root/time_msec/file", "my_port",
                                       PixelType::kRgba8U));
  EXPECT_EQ("/root/count",
            tester.DirectoryFromFormat("/root/count/file", "my_port",
                                       PixelType::kRgba8U));
}

// Tests the logic for formatting images.
TEST_F(ImageWriterTest, FileNameFormatting) {
  auto test_file_name = [](const ImageWriter& writer, const std::string& format,
                           PixelType pixel_type, double time,
                           const std::string& port_name, int count,
                           const std::string expected) {
    const std::string path = ImageWriterTester(writer).MakeFileName(
        format, pixel_type, time, port_name, count);
    EXPECT_EQ(path, expected);
  };

  ImageWriter writer;

  // Completely hard-coded; not dependent on any of ImageWriter's baked values.
  test_file_name(writer, "/hard/coded/file.png", PixelType::kRgba8U, 0, "port",
                 0, "/hard/coded/file.png");

  // Use the port name.
  test_file_name(writer, "/hard/{port_name}/file.png", PixelType::kRgba8U, 0,
                 "port", 0, "/hard/port/file.png");

  // Use the image type.
  test_file_name(writer, "/hard/{image_type}/file.png", PixelType::kRgba8U, 0,
                 "port", 0, "/hard/color/file.png");
  test_file_name(writer, "/hard/{image_type}/file.png", PixelType::kDepth32F, 0,
                 "port", 0, "/hard/depth/file.png");
  test_file_name(writer, "/hard/{image_type}/file.png", PixelType::kLabel16I, 0,
                 "port", 0, "/hard/label/file.png");

  // Use the time values.
  test_file_name(writer, "/hard/{time_double:.2f}.png", PixelType::kRgba8U, 0,
                 "port", 0, "/hard/0.00.png");
  test_file_name(writer, "/hard/{time_double:.2f}.png", PixelType::kRgba8U,
                 1.111, "port", 0, "/hard/1.11.png");
  test_file_name(writer, "/hard/{time_double:.2f}.png", PixelType::kRgba8U,
                 1.116, "port", 0, "/hard/1.12.png");

  test_file_name(writer, "/hard/{time_usec:03}.png", PixelType::kRgba8U, 0,
                 "port", 0, "/hard/000.png");
  test_file_name(writer, "/hard/{time_usec:03}.png", PixelType::kRgba8U, 1.111,
                 "port", 0, "/hard/1111000.png");

  test_file_name(writer, "/hard/{time_msec:03}.png", PixelType::kRgba8U, 0,
                 "port", 0, "/hard/000.png");
  test_file_name(writer, "/hard/{time_msec:03}.png", PixelType::kRgba8U, 1.111,
                 "port", 0, "/hard/1111.png");

  // Use the count value.
  test_file_name(writer, "/hard/{count:03}.png", PixelType::kRgba8U, 1.111,
                 "port", 0, "/hard/000.png");
  test_file_name(writer, "/hard/{count:03}.png", PixelType::kRgba8U, 1.111,
                 "port", 12, "/hard/012.png");

  // Bad place holders.
  EXPECT_THROW(
      test_file_name(writer, "/hard/{port}/file.png", PixelType::kRgba8U, 0,
                     "port", 0, "/hard/{port}/file.png"),
      fmt::format_error);
}

// Tests the write-ability of an image writer based on the validity of the
// directory path.
TEST_F(ImageWriterTest, ValidateDirectory) {
  // Case: Non-existent directory.
  EXPECT_TRUE(
      ImageWriterTester::DirectoryIsMissing("this/path/does/not_exist"));

  // Case: No write permissions (assuming that this isn't run as root).
  fs::path path(temp_dir());
  path.append("unwritable");
  fs::create_directory(path);
  fs::permissions(path, fs::perms::owner_write, fs::perm_options::remove);
  EXPECT_TRUE(ImageWriterTester::DirectoryIsUnwritable(path));

  // Case: the path is to a file.
  const std::string file_name = temp_name();
  std::ofstream stream(file_name, std::ios::out);
  ASSERT_FALSE(stream.fail());
  EXPECT_TRUE(ImageWriterTester::DirectoryIsFile(file_name));
}

// Confirm behavior on documented errors in
TEST_F(ImageWriterTest, ConfigureInputPortErrors) {
  ImageWriter writer;

  // Bad publish period.
  DRAKE_EXPECT_THROWS_MESSAGE(writer.DeclareImageInputPort<PixelType::kRgba8U>(
                                  "port", "format", -0.1, 0),
                              ".* publish period must be positive");

  // Invalid directory -- relies on tested correctness of ValidateDirectory()
  // and simply uses _one_ of the mechanisms for implying an invalid folder.
  DRAKE_EXPECT_THROWS_MESSAGE(
      writer.DeclareImageInputPort<PixelType::kRgba8U>("port", "/root/name",
                                                       0.1, 0),
      ".*The format string .* implied the invalid directory.*");

  // Now test a port with the same name -- can only happen if one port has
  // been successfully declared.
  fs::path path(temp_dir());
  path.append("file_{count:3}");
  const auto& port = writer.DeclareImageInputPort<PixelType::kRgba8U>(
      "port", path.string(), 0.1, 0);
  EXPECT_EQ(0, port.get_index());
  {
    auto events = writer.AllocateCompositeEventCollection();
    auto context = writer.AllocateContext();
    writer.CalcNextUpdateTime(*context, events.get());
    EXPECT_TRUE(events->HasEvents());
  }

  fs::path path2(temp_dir());
  path2.append("alt_file_{count:3}");
  DRAKE_EXPECT_THROWS_MESSAGE(writer.DeclareImageInputPort<PixelType::kRgba8U>(
                                  "port", path2.string(), 0.1, 0),
                              "System .* already has an input port named .*");
}

// Helper function for testing port declaration with a runtime pixel type.
template <PixelType kPixelType>
void TestRuntimePixelType() {
  ImageWriter writer;
  const auto& port =
      writer.DeclareImageInputPort(kPixelType, "in", "/tmp/{time_usec}", 1, 1);
  EXPECT_EQ(port.Allocate()->static_type_info(), typeid(Image<kPixelType>));
}

// This tests that the runtime pixel types are passed through correctly.
TEST_F(ImageWriterTest, RuntimePixelType) {
  TestRuntimePixelType<PixelType::kRgba8U>();
  TestRuntimePixelType<PixelType::kLabel16I>();
  TestRuntimePixelType<PixelType::kDepth32F>();
  TestRuntimePixelType<PixelType::kDepth16U>();
  TestRuntimePixelType<PixelType::kGrey8U>();
}

// Helper function for testing the extension produced for a given pixel type.
template <PixelType kPixelType>
void TestPixelExtension(const std::string& folder, ImageWriter* writer,
                        int* count) {
  using std::to_string;

  ImageWriterTester tester(*writer);

  fs::path format(folder);
  format.append("file");
  const auto& port = writer->DeclareImageInputPort<kPixelType>(
      "port" + to_string(++(*count)), format.string(), 1, 1);
  const std::string& final_format = tester.port_format(port.get_index());
  EXPECT_NE(format.string(), final_format);
  const std::string& ext = tester.extension(kPixelType);
  EXPECT_EQ(ext, final_format.substr(final_format.size() - ext.size()));
}

// This tests that format strings pick up the appropriate extension based on
// image type.
TEST_F(ImageWriterTest, FileExtension) {
  using std::to_string;

  ImageWriter writer;
  int count = 0;

  // Case: each image type applies the right extension to an extension-less
  // format string.
  TestPixelExtension<PixelType::kRgba8U>(temp_dir(), &writer, &count);
  TestPixelExtension<PixelType::kLabel16I>(temp_dir(), &writer, &count);
  TestPixelExtension<PixelType::kDepth32F>(temp_dir(), &writer, &count);
  TestPixelExtension<PixelType::kDepth16U>(temp_dir(), &writer, &count);
  TestPixelExtension<PixelType::kGrey8U>(temp_dir(), &writer, &count);

  ImageWriterTester tester(writer);
  // Case: Format string with correct extension remains unchanged.
  {
    fs::path format(temp_dir());
    format.append("file.png");
    const auto& port = writer.DeclareImageInputPort<PixelType::kRgba8U>(
        "port" + to_string(++count), format.string(), 1, 1);
    const std::string& final_format = tester.port_format(port.get_index());
    EXPECT_EQ(format.string(), final_format);
  }

  // Case: wrong extension gets correct extension appended.
  {
    fs::path format(temp_dir());
    format.append("file.txt");
    const auto& port = writer.DeclareImageInputPort<PixelType::kRgba8U>(
        "port" + to_string(++count), format.string(), 1, 1);
    const std::string& final_format = tester.port_format(port.get_index());
    const std::string& ext = tester.extension(PixelType::kRgba8U);
    EXPECT_EQ(format.string() + ext, final_format);
  }
}

// Probes the correctness of a single declared port.
TEST_F(ImageWriterTest, SingleConfiguredPort) {
  ImageWriter writer;
  ImageWriterTester tester(writer);

  // Freshly constructed, the writer has no timed events.
  {
    auto events = writer.AllocateCompositeEventCollection();
    auto context = writer.AllocateContext();
    writer.CalcNextUpdateTime(*context, events.get());
    EXPECT_FALSE(events->HasEvents());
  }

  // Values for port declaration.
  const double period = 1 / 10.0;  // 10 Hz.
  const double start_time = 0.25;
  const std::string port_name{"single_color_port"};
  const PixelType pixel_type = PixelType::kRgba8U;
  fs::path path(temp_dir());
  path.append("single_port_{time_usec}");

  const auto& port = writer.DeclareImageInputPort<PixelType::kRgba8U>(
      port_name, path.string(), period, start_time);

  // Count gets properly initialized to zero (no images written from this port).
  EXPECT_EQ(0, tester.port_count(port.get_index()));

  // Confirm a reported periodic event and a forced publish event. The
  // configuration parameters above are called out below in commented lines.
  {
    auto events = writer.AllocateCompositeEventCollection();
    auto forced_events = writer.AllocateForcedPublishEventCollection();
    auto context = writer.AllocateContext();
    context->SetTime(0.);
    double next_time = writer.CalcNextUpdateTime(*context, events.get());
    // Confirm start time fed into the periodic event.
    EXPECT_EQ(start_time, next_time);

    EXPECT_TRUE(events->HasEvents());
    EXPECT_FALSE(events->HasDiscreteUpdateEvents());
    EXPECT_FALSE(events->HasUnrestrictedUpdateEvents());
    EXPECT_TRUE(events->HasPublishEvents());
    EXPECT_TRUE(forced_events->HasEvents());

    {
      const auto& publish_events =
          dynamic_cast<const LeafEventCollection<PublishEvent<double>>&>(
              events->get_publish_events())
              .get_events();
      ASSERT_EQ(1u, publish_events.size());
      const auto& event = publish_events.front();
      EXPECT_EQ(TriggerType::kPeriodic, event->get_trigger_type());

      const auto& forced_publish_events =
          dynamic_cast<const LeafEventCollection<PublishEvent<double>>&>(
              *forced_events)
              .get_events();
      // The event collection should have two events: ImageWriter's explicitly
      // declared event and the collection's built-in no-op event.
      ASSERT_EQ(2u, forced_publish_events.size());
      const auto& forced_event = forced_publish_events.front();
      EXPECT_EQ(TriggerType::kForced, forced_event->get_trigger_type());

      // With no connection on the input port, publishing this event will result
      // in an error.
      DRAKE_EXPECT_THROWS_MESSAGE(
          writer.Publish(*context, events->get_publish_events()),
          ".*InputPort.* is not connected");

      // Confirms that a valid publish increments the counter.
      port.FixValue(context.get(), test_image<PixelType::kRgba8U>());

      std::string expected_name = tester.MakeFileName(
          tester.port_format(port.get_index()), pixel_type, context->get_time(),
          port_name, tester.port_count(port.get_index()));
      fs::path expected_file(expected_name);
      EXPECT_FALSE(fs::exists(expected_file));
      const EventStatus status =
          writer.Publish(*context, events->get_publish_events());
      EXPECT_TRUE(status.succeeded());
      EXPECT_TRUE(fs::exists(expected_file));
      EXPECT_EQ(1, tester.port_count(port.get_index()));
      add_file_for_cleanup(expected_file.string());

      // Confirms that a valid forced publish increments the counter and writes
      // an image. We keep the image saved, so that we can immediately test a
      // redundant call to ForcedPublish() with the same context.
      context->SetTime(1e-6);
      expected_name = tester.MakeFileName(
          tester.port_format(port.get_index()), pixel_type, context->get_time(),
          port_name, tester.port_count(port.get_index()));
      expected_file.assign(expected_name);
      EXPECT_FALSE(fs::exists(expected_file));
      writer.ForcedPublish(*context);
      EXPECT_TRUE(fs::exists(expected_file));
      EXPECT_EQ(2, tester.port_count(port.get_index()));

      // Confirms that invoking ForcedPublish() a second time simply redundantly
      // writes the image.
      writer.ForcedPublish(*context);
      EXPECT_TRUE(fs::exists(expected_file));
      EXPECT_EQ(3, tester.port_count(port.get_index()));
      add_file_for_cleanup(expected_file.string());

      // Confirms resetting count to zero works.
      writer.ResetAllImageCounts();
      EXPECT_EQ(0, tester.port_count(port.get_index()));
    }

    // Confirm period is correct.
    context->SetTime(start_time + 0.1 * period);
    events->Clear();
    next_time = writer.CalcNextUpdateTime(*context, events.get());
    EXPECT_EQ(start_time + period, next_time);
  }
}

// This simply confirms that the color image gets written to the right format.
TEST_F(ImageWriterTest, WritesColorImage) {
  TestWritingImageOnPort<PixelType::kRgba8U>();
}

// This confirms that the label image gets written properly.
TEST_F(ImageWriterTest, WritesLabelImage) {
  TestWritingImageOnPort<PixelType::kLabel16I>();
}

// This simply confirms that the depth image gets written to the right format.
TEST_F(ImageWriterTest, WritesDepthImage) {
  TestWritingImageOnPort<PixelType::kDepth32F>();
}

// This simply confirms that the depth image gets written to the right format.
TEST_F(ImageWriterTest, WritesDepthImage16U) {
  TestWritingImageOnPort<PixelType::kDepth16U>();
}

// This simply confirms that the color image gets written to the right format.
TEST_F(ImageWriterTest, WritesGreyImage) {
  TestWritingImageOnPort<PixelType::kGrey8U>();
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
