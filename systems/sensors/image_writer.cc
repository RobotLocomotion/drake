#include "drake/systems/sensors/image_writer.h"

#include <unistd.h>

#include <filesystem>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/systems/sensors/image_io.h"

namespace drake {
namespace systems {
namespace sensors {

void SaveToPng(const ImageRgba8U& image, const std::string& file_path) {
  ImageIo{}.Save(image, file_path, ImageFileFormat::kPng);
}

void SaveToTiff(const ImageDepth32F& image, const std::string& file_path) {
  ImageIo{}.Save(image, file_path, ImageFileFormat::kTiff);
}

void SaveToPng(const ImageDepth16U& image, const std::string& file_path) {
  ImageIo{}.Save(image, file_path, ImageFileFormat::kPng);
}

void SaveToPng(const ImageLabel16I& image, const std::string& file_path) {
  ImageIo{}.Save(image, file_path, ImageFileFormat::kPng);
}

void SaveToPng(const ImageGrey8U& image, const std::string& file_path) {
  ImageIo{}.Save(image, file_path, ImageFileFormat::kPng);
}

ImageWriter::ImageWriter() {
  // NOTE: This excludes *many* of the defined `PixelType` values.
  labels_[PixelType::kRgba8U] = "color";
  extensions_[PixelType::kRgba8U] = ".png";
  labels_[PixelType::kDepth32F] = "depth";
  extensions_[PixelType::kLabel16I] = ".png";
  labels_[PixelType::kLabel16I] = "label";
  extensions_[PixelType::kDepth32F] = ".tiff";
  labels_[PixelType::kDepth16U] = "depth";
  extensions_[PixelType::kDepth16U] = ".png";
  labels_[PixelType::kGrey8U] = "grey_scale";
  extensions_[PixelType::kGrey8U] = ".png";

  // Declares a forced publish event to accommodate non-periodic image saving,
  // e.g., when saving images outside of Simulator::AdvanceTo.
  DeclareForcedPublishEvent(&ImageWriter::WriteAllImages);
}

template <PixelType kPixelType>
const InputPort<double>& ImageWriter::DeclareImageInputPort(
    std::string port_name, std::string file_name_format, double publish_period,
    double start_time) {
  // Test to confirm valid pixel type.
  static_assert(kPixelType == PixelType::kRgba8U ||
                    kPixelType == PixelType::kDepth32F ||
                    kPixelType == PixelType::kDepth16U ||
                    kPixelType == PixelType::kLabel16I ||
                    kPixelType == PixelType::kGrey8U,
                "ImageWriter: the only supported pixel types are: kRgba8U, "
                "kDepth32F, kDepth16U, kGrey8U, and kLabel16I");

  if (publish_period <= 0) {
    throw std::logic_error("ImageWriter: publish period must be positive");
  }

  // Confirms the implied directory is valid.
  const std::string test_dir =
      DirectoryFromFormat(file_name_format, port_name, kPixelType);
  FolderState folder_state = ValidateDirectory(test_dir);
  if (folder_state != FolderState::kValid) {
    const char* const reason = [folder_state]() {
      switch (folder_state) {
        case FolderState::kValid:
          DRAKE_UNREACHABLE();
        case FolderState::kMissing:
          return "the directory does not exist";
        case FolderState::kIsFile:
          return "the directory is actually a file";
        case FolderState::kUnwritable:
          return "no permissions to write the directory";
      }
      DRAKE_UNREACHABLE();
    }();
    throw std::logic_error(
        fmt::format("ImageWriter: The format string `{}` implied the invalid "
                    "directory: '{}'; {}",
                    file_name_format, test_dir, reason));
  }

  // Confirms file has appropriate extension.
  const std::string& extension = extensions_[kPixelType];
  if (file_name_format.substr(file_name_format.size() - extension.size()) !=
      extension) {
    file_name_format += extension;
  }
  // TODO(SeanCurtis-TRI): Handle other issues that may arise with filename:
  //  - invalid symbols
  //  - invalid length
  //  - more?

  // Now configure the system for the valid port declaration.
  const auto& port =
      DeclareAbstractInputPort(port_name, Value<Image<kPixelType>>());

  // There is no DeclarePeriodicPublishEvent that accepts a lambda, so we must
  // use the advanced API to add our event.
  PublishEvent<double> event(
      TriggerType::kPeriodic,
      [port_index = port.get_index()](const System<double>& system,
                                      const Context<double>& context,
                                      const PublishEvent<double>&) {
        const auto& self = dynamic_cast<const ImageWriter&>(system);
        self.WriteImage<kPixelType>(context, port_index);
        return EventStatus::Succeeded();
      });
  DeclarePeriodicEvent<PublishEvent<double>>(publish_period, start_time, event);
  port_info_.emplace_back(std::move(file_name_format), kPixelType);

  return port;
}

const InputPort<double>& ImageWriter::DeclareImageInputPort(
    PixelType pixel_type, std::string port_name, std::string file_name_format,
    double publish_period, double start_time) {
  switch (pixel_type) {
    case PixelType::kRgb8U:
      break;
    case PixelType::kBgr8U:
      break;
    case PixelType::kRgba8U: {
      return this->template DeclareImageInputPort<PixelType::kRgba8U>(
          std::move(port_name), std::move(file_name_format), publish_period,
          start_time);
    }
    case PixelType::kBgra8U:
      break;
    case PixelType::kDepth16U: {
      return this->template DeclareImageInputPort<PixelType::kDepth16U>(
          std::move(port_name), std::move(file_name_format), publish_period,
          start_time);
    }
    case PixelType::kDepth32F: {
      return this->template DeclareImageInputPort<PixelType::kDepth32F>(
          std::move(port_name), std::move(file_name_format), publish_period,
          start_time);
    }
    case PixelType::kLabel16I: {
      return this->template DeclareImageInputPort<PixelType::kLabel16I>(
          std::move(port_name), std::move(file_name_format), publish_period,
          start_time);
    }
    case PixelType::kGrey8U: {
      return this->template DeclareImageInputPort<PixelType::kGrey8U>(
          std::move(port_name), std::move(file_name_format), publish_period,
          start_time);
    }
  }
  throw std::logic_error(fmt::format(
      "ImageWriter::DeclareImageInputPort does not support pixel_type={}",
      static_cast<int>(pixel_type)));
}

void ImageWriter::ResetAllImageCounts() const {
  for (const auto& port_info : port_info_) {
    port_info.count = 0;
  }
}

template <PixelType kPixelType>
void ImageWriter::WriteImage(const Context<double>& context, int index) const {
  const auto& port = get_input_port(index);
  const ImagePortInfo& data = port_info_[index];
  const Image<kPixelType>& image = port.Eval<Image<kPixelType>>(context);
  ImageIo{}.Save(image,
                 MakeFileName(data.format, data.pixel_type, context.get_time(),
                              port.get_name(), data.count++));
}

EventStatus ImageWriter::WriteAllImages(const Context<double>& context) const {
  auto periodic_events = this->AllocateCompositeEventCollection();
  this->GetPeriodicEvents(context, periodic_events.get());
  return this->Publish(context, periodic_events->get_publish_events());
}

std::string ImageWriter::MakeFileName(const std::string& format,
                                      PixelType pixel_type, double time,
                                      const std::string& port_name,
                                      int count) const {
  DRAKE_DEMAND(labels_.contains(pixel_type));

  int64_t u_time = static_cast<int64_t>(time * 1e6 + 0.5);
  int m_time = static_cast<int>(time * 1e3 + 0.5);
  return fmt::format(fmt_runtime(format), fmt::arg("port_name", port_name),
                     fmt::arg("image_type", labels_.at(pixel_type)),
                     fmt::arg("time_double", time),
                     fmt::arg("time_usec", u_time),
                     fmt::arg("time_msec", m_time), fmt::arg("count", count));
}

std::string ImageWriter::DirectoryFromFormat(const std::string& format,
                                             const std::string& port_name,
                                             PixelType pixel_type) const {
  // Extract the directory.  Note that in any error messages to the user, we'll
  // report using the argument name from the public method.
  if (format.empty()) {
    throw std::logic_error("ImageWriter: The file_name_format cannot be empty");
  }
  if (format.back() == '/') {
    throw std::logic_error(fmt::format(
        "ImageWriter: The file_name_format '{}' cannot end with a '/'",
        format));
  }
  size_t index = format.rfind('/');
  std::string dir_format = format.substr(0, index);
  // NOTE: [bcdelmosu] are all the characters in: double, msec, and usec.
  // Technically, this will also key on '{time_mouse}', but if someone is
  // putting that in their file path, they deserve whatever they get.
  std::regex invalid_args("\\{count|time_[bcdelmosu]+\\}");
  std::smatch match;
  std::regex_search(dir_format, match, invalid_args);
  if (!match.empty()) {
    throw std::logic_error(
        "ImageWriter: The directory path cannot include time or image count");
  }
  return MakeFileName(dir_format, pixel_type, 0, port_name, 0);
}

ImageWriter::FolderState ImageWriter::ValidateDirectory(
    const std::string& file_path_str) {
  std::filesystem::path file_path(file_path_str);
  if (std::filesystem::exists(file_path)) {
    if (std::filesystem::is_directory(file_path)) {
      if (::access(file_path.string().c_str(), W_OK) == 0) {
        return FolderState::kValid;
      } else {
        return FolderState::kUnwritable;
      }
    } else {
      return FolderState::kIsFile;
    }
  } else {
    return FolderState::kMissing;
  }
}

template const InputPort<double>& ImageWriter::DeclareImageInputPort<
    PixelType::kRgba8U>(std::string port_name, std::string file_name_format,
                        double publish_period, double start_time);
template const InputPort<double>& ImageWriter::DeclareImageInputPort<
    PixelType::kDepth32F>(std::string port_name, std::string file_name_format,
                          double publish_period, double start_time);
template const InputPort<double>& ImageWriter::DeclareImageInputPort<
    PixelType::kLabel16I>(std::string port_name, std::string file_name_format,
                          double publish_period, double start_time);
template const InputPort<double>& ImageWriter::DeclareImageInputPort<
    PixelType::kDepth16U>(std::string port_name, std::string file_name_format,
                          double publish_period, double start_time);
template const InputPort<double>& ImageWriter::DeclareImageInputPort<
    PixelType::kGrey8U>(std::string port_name, std::string file_name_format,
                        double publish_period, double start_time);

template void ImageWriter::WriteImage<PixelType::kRgba8U>(
    const Context<double>& context, int index) const;
template void ImageWriter::WriteImage<PixelType::kDepth32F>(
    const Context<double>& context, int index) const;
template void ImageWriter::WriteImage<PixelType::kLabel16I>(
    const Context<double>& context, int index) const;
template void ImageWriter::WriteImage<PixelType::kDepth16U>(
    const Context<double>& context, int index) const;
template void ImageWriter::WriteImage<PixelType::kGrey8U>(
    const Context<double>& context, int index) const;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
