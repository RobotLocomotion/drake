#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkImageReader2.h>  // vtkIOImage
#include <vtkImageWriter.h>   // vtkIOImage
#include <vtkSmartPointer.h>  // vtkCommonCore

#include "drake/systems/sensors/image_file_format.h"

namespace drake {
namespace systems {
namespace sensors {
namespace internal {

// This file provides internal helper functions that encapsulate VTK's reader
// and writer classes for the file formats Drake cares about.

/* Constructs a VTK image reader for a specific file format, already connected
to read the data from the given filename. */
vtkSmartPointer<vtkImageReader2> MakeReader(
    ImageFileFormat format, const std::filesystem::path& filename);

/* Constructs a VTK image reader for a specific file format, already connected
to read the data from the given memory buffer of `size` bytes.
@warning The reader retains a pointer to the `input`, so it can read from it
when requested to. The `input` buffer must outlive the returned reader. */
vtkSmartPointer<vtkImageReader2> MakeReader(ImageFileFormat format,
                                            const void* input, size_t size);

/* Constructs a VTK image writer for a specific file format, already connected
to write the data to the given destination filename. */
vtkSmartPointer<vtkImageWriter> MakeWriter(
    ImageFileFormat format, const std::filesystem::path& filename);

/* Constructs a VTK image writer for a specific file format, already connected
to write the data to the given memory
@warning The TIFF file format cannot currently be written to a memory buffer,
rather only to a file. Passing `format == kTiff` will throw an exception.
@warning The writer retains a pointer to the `output`, so it can set it during
the Write() operation.  The `output` buffer must outlive the returned writer. */
vtkSmartPointer<vtkImageWriter> MakeWriter(ImageFileFormat format,
                                           std::vector<uint8_t>* output);

}  // namespace internal
}  // namespace sensors
}  // namespace systems
}  // namespace drake
