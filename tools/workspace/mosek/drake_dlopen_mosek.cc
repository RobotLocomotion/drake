#include <dlfcn.h>
#include <stdio.h>

#include <filesystem>
#include <string>

#include "drake/common/find_loaded_library.h"

using drake::LoadedLibraryPath;

extern "C" void* drake_dlopen_mosek(const char* /* lib_name */) {
  // Find libdrake in site-packages/pydrake/lib/libdrake.so.
  std::optional<std::string> libdrake_dir = LoadedLibraryPath("libdrake.so");
  if (!libdrake_dir.has_value()) {
    fprintf(stderr, "Cannot load MOSEK: cannot even find libdrake.so");
    return nullptr;
  }
  // Respell from site-packages/pydrake/lib to site-packages/mosek.
  const auto mosek_dir = std::filesystem::path{*libdrake_dir} / "../../mosek";
  // Load libmosek from site-packages.
  const std::string mosek_lib = (mosek_dir / "libmosek64.so.11.0").string();
  void* handle = dlopen(mosek_lib.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    fprintf(stderr, "Cannot load MOSEK: dlopen(%s) failed: %s\n",
            mosek_lib.c_str(), dlerror());
  }
  return handle;
}
