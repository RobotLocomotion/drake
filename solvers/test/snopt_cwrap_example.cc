
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
