#include <dlfcn.h>

#include <filesystem>
#include <string>

#include "drake/common/find_loaded_library.h"
#include "drake/common/text_logging.h"

using drake::LoadedLibraryPath;
using drake::log;

extern "C" {

void* drake_dlopen_mosek(const char* lib_name, bool quiet) {
  // Find libdrake in site-packages/pydrake/lib/libdrake.so.
  std::optional<std::string> libdrake_dir = LoadedLibraryPath("libdrake.so");
  if (!libdrake_dir.has_value()) {
    if (!quiet) log()->error("Cannot load MOSEK: cannot even find libdrake.so");
    return nullptr;
  }
  // Respell from site-packages/pydrake/lib to site-packages/mosek.
  const auto mosek_dir = std::filesystem::path{*libdrake_dir} / "../../mosek";
  // Load libmosek from site-packages.
  const std::string mosek_lib = (mosek_dir / lib_name).string();
  void* handle = dlopen(mosek_lib.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    if (!quiet)
      log()->error("Cannot load MOSEK: dlopen({}) failed: {}", mosek_lib,
                   dlerror());
  }
  return handle;
}

// This is a callack used by Implib.so as part of our ":implib_mosek" library to
// load MOSEK™ lazily, which (currently) is only enabled during our wheel
// builds, not normal packaging builds nor developer builds. It is called once
// the first time Drake needs to call any function in the MOSEK shared library.
void* drake_dlopen_mosek_loud(const char* lib_name) {
  return drake_dlopen_mosek(lib_name, false);
}
}
