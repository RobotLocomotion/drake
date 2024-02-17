/* clang-format off to disable clang-format-includes */
#include "drake/tools/performance/fixture_memory.h"
/* clang-format on */

#include <type_traits>

#include <benchmark/benchmark.h>
#include <malloc.h>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

namespace {

using benchmark::MemoryManager;
using Result = MemoryManager::Result;

/* This class is a singleton (via EnableMemoryManager). */
class GlibcMemoryManager final : public MemoryManager {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GlibcMemoryManager);
  GlibcMemoryManager() = default;

  void Start() final { Tare(); }
  void Stop(Result& result) final;  // NOLINT(runtime/references)

  // Resets all counters back to zero.
  static void Tare() {
    g_num_allocs = 0;
    g_total_allocated_bytes = 0;
    g_mallinfo_tare = ::mallinfo2();
  }

  // Since this class is a singleton, we can use static fields instead of
  // instance fields, for convenience.
  static int64_t g_num_allocs;
  static int64_t g_total_allocated_bytes;
  static struct mallinfo2 g_mallinfo_tare;
};

int64_t GlibcMemoryManager::g_num_allocs = 0;
int64_t GlibcMemoryManager::g_total_allocated_bytes = 0;
static_assert(std::is_trivially_constructible<struct mallinfo2>());
static_assert(std::is_trivially_destructible<struct mallinfo2>());
struct mallinfo2 GlibcMemoryManager::g_mallinfo_tare = {};

void GlibcMemoryManager::Stop(Result& result) {
  struct mallinfo2 report = ::mallinfo2();

  result.num_allocs = g_num_allocs;
  result.total_allocated_bytes = g_total_allocated_bytes;
  result.net_heap_growth = report.uordblks - g_mallinfo_tare.uordblks;

  // There's no easy way to collect this data.
  result.max_bytes_used = 0;
}

}  // namespace

namespace drake {
namespace tools {
namespace performance {

void EnableMemoryManager() {
  // Even though our manager subclass doesn't have any member fields, we don't
  // know if the base class has any. Therefore, we can't use a static global,
  // we need to create the manager on demand (just once).
  static const never_destroyed<GlibcMemoryManager> singleton;
  benchmark::RegisterMemoryManager(
      const_cast<GlibcMemoryManager*>(&singleton.access()));
}

void TareMemoryManager() {
  GlibcMemoryManager::Tare();
}

}  // namespace performance
}  // namespace tools
}  // namespace drake

// https://www.gnu.org/software/libc/manual/html_node/Replacing-malloc.html#Replacing-malloc
extern "C" void* __libc_malloc(size_t);
extern "C" void* __libc_free(void*);
extern "C" void* __libc_calloc(size_t, size_t);
extern "C" void* __libc_realloc(void*, size_t);
void* malloc(size_t size) {
  // Note the disclaimer on EnableMemoryManager() on counting vs threading.
  GlibcMemoryManager::g_num_allocs += 1;
  GlibcMemoryManager::g_total_allocated_bytes += size;
  return __libc_malloc(size);
}
void free(void* ptr) {
  __libc_free(ptr);
}
void* calloc(size_t nmemb, size_t size) {
  // Note the disclaimer on EnableMemoryManager() on counting vs threading.
  GlibcMemoryManager::g_num_allocs += 1;
  GlibcMemoryManager::g_total_allocated_bytes += nmemb * size;
  return __libc_calloc(nmemb, size);
}
void* realloc(void* ptr, size_t size) {
  // Note the disclaimer on EnableMemoryManager() on counting vs threading.
  GlibcMemoryManager::g_num_allocs += 1;
  GlibcMemoryManager::g_total_allocated_bytes += size;
  return __libc_realloc(ptr, size);
}
