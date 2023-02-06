#pragma once

namespace drake {
namespace solvers {
namespace internal {
namespace csdp {

// Add CSDP's types into this namespace.
extern "C" {
#include <csdp/blockmat.h>
#include <csdp/index.h>
#include <csdp/parameters.h>
}  // extern C

// Wrap CSDP's C functions with our own C++ functions, for error handling.
int cpp_easy_sdp(const char* params_pathname, int n, int k,
                 struct blockmatrix C, double* a,
                 struct constraintmatrix* constraints, double constant_offset,
                 struct blockmatrix* pX, double** py, struct blockmatrix* pZ,
                 double* ppobj, double* pdobj);
void cpp_free_mat(struct blockmatrix A);
void cpp_free_prob(int n, int k, struct blockmatrix C, double* a,
                   struct constraintmatrix* constraints, struct blockmatrix X,
                   double* y, struct blockmatrix Z);
void cpp_initsoln(int n, int k, struct blockmatrix C, double* a,
                  struct constraintmatrix* constraints, struct blockmatrix* pX0,
                  double** py0, struct blockmatrix* pZ0);

}  // namespace csdp
}  // namespace internal
}  // namespace solvers
}  // namespace drake
