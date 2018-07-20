#ifdef DISABLE_SNOPT_CWRAP_EXAMPLE

int main(int argc, char* argv[]) { return 0; }

#else

extern "C" {
#include "snopt_cwrap.h"  // NOLINT(build/include)
}

int main(int argc, char* argv[]) {
  snProblem problem{};
  const auto& function_pointer = &solveA;

  (void)(problem);  // unused
  (void)(function_pointer);  // unused

  return 0;
}

#endif
