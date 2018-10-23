#include "drake/systems/sensors/image_writer.h"

#include <unistd.h>

#include <string>
#include <utility>
#include <vector>

#include <spruce.hh>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>
#include <vtkSmartPointer.h>
#include <vtkTIFFWriter.h>
#include "fmt/ostream.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace systems {
namespace sensors {

using std::move;

template <PixelType kPixelType>
void SaveToFileHelper(const std::string& file_path,
                      const Image<kPixelType>& image) {
  const int width = image.width();
  const int height = image.height();
  const int num_channels = Image<kPixelType>::kNumChannels;

  vtkSmartPointer<vtkImageWriter> writer;
  vtkNew<vtkImageData> vtk_image;
  vtk_image->SetDimensions(width, height, 1);

  switch (kPixelType) {
    case PixelType::kRgba8U:
      vtk_image->AllocateScalars(VTK_UNSIGNED_CHAR, num_channels);
      writer = vtkSmartPointer<vtkPNGWriter>::New();
      break;
    case PixelType::kDepth32F:
      vtk_image->AllocateScalars(VTK_FLOAT, num_channels);
      writer = vtkSmartPointer<vtkTIFFWriter>::New();
      break;
    case PixelType::kLabel16I:
      vtk_image->AllocateScalars(VTK_UNSIGNED_SHORT, num_channels);
      writer = vtkSmartPointer<vtkPNGWriter>::New();
      break;
    default:
      throw std::logic_error(
          "Unsupported image type; cannot be written to file");
  }

  auto image_ptr = reinterpret_cast<typename Image<kPixelType>::T*>(
      vtk_image->GetScalarPointer());
  const int num_scalar_components = vtk_image->GetNumberOfScalarComponents();
  DRAKE_DEMAND(num_scalar_components == num_channels);

  for (int v = height - 1; v >= 0; --v) {
    for (int u = 0; u < width; ++u) {
      for (int c = 0; c < num_channels; ++c) {
        image_ptr[c] =
            static_cast<typename Image<kPixelType>::T>(image.at(u, v)[c]);
      }
      image_ptr += num_scalar_components;
    }
  }

  writer->SetFileName(file_path.c_str());
  writer->SetInputData(vtk_image.GetPointer());
  writer->Write();
}

void SaveToPng(const std::string& file_path, const ImageRgba8U& image) {
  SaveToFileHelper(file_path, image);
}

void SaveToTiff(const std::string& file_path, const ImageDepth32F& image) {
  SaveToFileHelper(file_path, image);
}

void SaveToPng(const std::string& file_path, const ImageLabel16I& image) {
  SaveToFileHelper(file_path, image);
}

ImageWriter::ImageWriter(std::string folder_path, std::string image_name,
                         double start_time, int padding)
    : folder_path_(move(folder_path)),
      image_name_base_(move(image_name)),
      start_time_(start_time),
      padding_(padding),
      can_write_(IsDirectoryValid(folder_path_)) {
  color_image_port_index_ =
      DeclareAbstractInputPort("color_image",
                               systems::Value<ImageRgba8U>()).get_index();
  depth_image_port_index_ =
      DeclareAbstractInputPort("depth_image",
                               systems::Value<ImageDepth32F>()).get_index();
  label_image_port_index_ =
      DeclareAbstractInputPort("label_image",
                               systems::Value<ImageLabel16I>()).get_index();
}

void ImageWriter::set_publish_period(double period) {
  if (period < 0) {
    throw std::logic_error("ImageWriter requires a positive period value");
  }

  if (can_write_) {
    LeafSystem<double>::DeclarePeriodicPublish(period);
  } else {
    drake::log()->warn(
        "ImageWriter: the provided folder cannot be written to: " +
        folder_path_ + "; No files will be written.");
  }
}

const InputPort<double>& ImageWriter::color_image_input_port() const {
  return this->get_input_port(color_image_port_index_);
}

const InputPort<double>& ImageWriter::depth_image_input_port() const {
  return this->get_input_port(depth_image_port_index_);
}

const InputPort<double>& ImageWriter::label_image_input_port() const {
  return this->get_input_port(label_image_port_index_);
}

void ImageWriter::DoPublish(
    const Context<double>& context,
    const std::vector<const PublishEvent<double>*>&) const {
  // Includes can_write_ to guard against forced invocations of Publish (in
  // addition to the events this system may create).
  if (can_write_ && context.get_time() >= start_time_) {
    const ImageRgba8U* color_image =
        this->EvalInputValue<ImageRgba8U>(context, color_image_port_index_);
    const ImageDepth32F* depth_image =
        this->EvalInputValue<ImageDepth32F>(context, depth_image_port_index_);
    const ImageLabel16I* label_image =
        this->EvalInputValue<ImageLabel16I>(context, label_image_port_index_);
    if (color_image || depth_image || label_image) ++previous_frame_index_;

    if (color_image) {
      SaveToPng(make_file_name("color", previous_frame_index_, "png"),
                *color_image);
    }
    if (depth_image) {
      SaveToTiff(make_file_name("depth", previous_frame_index_, "tiff"),
                 *depth_image);
    }
    if (label_image) {
      SaveToPng(make_file_name("label", previous_frame_index_, "png"),
                *label_image);
    }
  }
}

bool ImageWriter::IsDirectoryValid(const std::string& file_path) {
  spruce::path dirpath(file_path);
  if (dirpath.exists()) {
    if (dirpath.isDir()) {
      if (access(dirpath.getStr().c_str(), W_OK) == 0) {
        return true;
      } else {
        drake::log()->warn(
            "ImageWriter: no write permissions for the given folder: " +
            file_path);
      }
    } else {
      drake::log()->warn(
          "ImageWriter: the provided folder isn't a directory: " + file_path);
    }
  } else {
    drake::log()->warn("ImageWriter: the provided folder doesn't exist: " +
                       file_path);
  }
  return false;
}

std::string ImageWriter::make_file_name(const std::string& type_name,
                                        int frame_number,
                                        const std::string& ext) const {
  spruce::path dirpath(folder_path_);
  std::string file_name = fmt::format("{0}_{1}_{2:0{3}d}.{4}", image_name_base_,
                                      type_name, frame_number, padding_, ext);
  dirpath.append(file_name);
  return dirpath.getStr();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
