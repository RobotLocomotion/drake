#include "drake/systems/sensors/vtk_image_reader_writer.h"

#include <functional>
#include <utility>
#include <variant>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkCommand.h>               // vtkCommonCore
#include <vtkJPEGReader.h>            // vtkIOImage
#include <vtkJPEGWriter.h>            // vtkIOImage
#include <vtkMemoryResourceStream.h>  // vtkIOCore
#include <vtkPNGReader.h>             // vtkIOImage
#include <vtkPNGWriter.h>             // vtkIOImage
#include <vtkTIFFReader.h>            // vtkIOImage
#include <vtkTIFFWriter.h>            // vtkIOImage
#include <vtkUnsignedCharArray.h>     // vtkCommonCore

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {
namespace {

namespace fs = std::filesystem;

vtkSmartPointer<vtkImageReader2> MakeReaderObject(ImageFileFormat format) {
  switch (format) {
    case ImageFileFormat::kJpeg:
      return vtkSmartPointer<vtkJPEGReader>::New();
    case ImageFileFormat::kPng:
      return vtkSmartPointer<vtkPNGReader>::New();
    case ImageFileFormat::kTiff: {
      auto result = vtkSmartPointer<vtkTIFFReader>::New();
      // VTK's TIFF reader doesn't use VTK's default coordinate conventions
      // unless we specifically tell it to.
      result->SetOrientationType(4 /* ORIENTATION_BOTLEFT */);
      return result;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

vtkSmartPointer<vtkImageReader2> MakeReader(ImageFileFormat format,
                                            const fs::path& filename) {
  vtkSmartPointer<vtkImageReader2> reader = MakeReaderObject(format);
  reader->SetFileName(filename.c_str());
  return reader;
}

vtkSmartPointer<vtkImageReader2> MakeReader(ImageFileFormat format,
                                            const void* input, size_t size) {
  vtkSmartPointer<vtkImageReader2> reader = MakeReaderObject(format);
  vtkNew<vtkMemoryResourceStream> stream;
  stream->SetBuffer(input, size);
  reader->SetStream(stream);
  return reader;
}

namespace {

vtkSmartPointer<vtkImageWriter> MakeWriterObject(ImageFileFormat format) {
  switch (format) {
    case ImageFileFormat::kJpeg:
      return vtkSmartPointer<vtkJPEGWriter>::New();
    case ImageFileFormat::kPng:
      return vtkSmartPointer<vtkPNGWriter>::New();
    case ImageFileFormat::kTiff:
      return vtkSmartPointer<vtkTIFFWriter>::New();
  }
  DRAKE_UNREACHABLE();
}

/* Trampolines VTK progress events into a Drake-specific callback. */
class VtkProgressObserver final : public vtkCommand {
 public:
  VtkProgressObserver() = default;
  void set_progress_callback(std::function<void(double)> callback) {
    callback_ = std::move(callback);
  }
  // Boilerplate for VTK smart pointers.
  static VtkProgressObserver* New() { return new VtkProgressObserver; }

 private:
  // NOLINTNEXTLINE(runtime/int) To match the VTK signature.
  void Execute(vtkObject*, unsigned long event, void* calldata) final {
    if (event == vtkCommand::ProgressEvent) {
      const double amount = *static_cast<const double*>(calldata);
      if (callback_ != nullptr) {
        callback_(amount);
      }
    }
  }
  std::function<void(double)> callback_;
};

}  // namespace

vtkSmartPointer<vtkImageWriter> MakeWriter(ImageFileFormat format,
                                           const fs::path& filename) {
  vtkSmartPointer<vtkImageWriter> writer = MakeWriterObject(format);
  writer->SetFileName(filename.c_str());
  return writer;
}

vtkSmartPointer<vtkImageWriter> MakeWriter(ImageFileFormat format,
                                           std::vector<uint8_t>* output) {
  DRAKE_DEMAND(output != nullptr);
  if (format == ImageFileFormat::kTiff) {
    throw std::logic_error("Cannot save TIFF images to a memory buffer");
  }
  vtkSmartPointer<vtkImageWriter> writer = MakeWriterObject(format);

  // The "write to memory" API is only available on the concrete subclasses,
  // so we'll always need to call it with a visitor, instead of virtually.
  std::variant<vtkJPEGWriter*, vtkPNGWriter*> writer_variant;
  switch (format) {
    case ImageFileFormat::kJpeg:
      writer_variant = static_cast<vtkJPEGWriter*>(writer.Get());
      break;
    case ImageFileFormat::kPng:
      writer_variant = static_cast<vtkPNGWriter*>(writer.Get());
      break;
    case ImageFileFormat::kTiff:
      DRAKE_UNREACHABLE();
  }
  std::visit(
      [](auto* typed_writer) {
        typed_writer->WriteToMemoryOn();
      },
      writer_variant);

  // Either a FileName or FilePrefix is mandatory.
  writer->SetFilePrefix("drake");

  // Add an observer to the writer that copies the encoded image into the
  // `output` buffer.
  vtkNew<VtkProgressObserver> observer;
  observer->set_progress_callback([output, writer_variant](double amount) {
    // TODO(jwnimmer) Use the `amount` to decide when the writing is finished.
    // At the moment, VTK mistakenly posts 0% progress both when it starts and
    // when it finishes, so we can't use 100% to know when its time to copy.
    unused(amount);
    std::visit(
        [output](auto* typed_writer) {
          vtkUnsignedCharArray* range = typed_writer->GetResult();
          // The result pointer `range` only exists after writing is complete.
          // Use that to know when we're actually done.
          if (range != nullptr) {
            output->clear();
            const size_t size = range->GetNumberOfTuples();
            const uint8_t* const data = range->GetPointer(0);
            output->insert(output->begin(), data, data + size);
          }
        },
        writer_variant);
  });
  writer->AddObserver(vtkCommand::ProgressEvent, observer);

  return writer;
}

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
