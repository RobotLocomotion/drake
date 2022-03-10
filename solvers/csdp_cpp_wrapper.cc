#include "drake/solvers/csdp_cpp_wrapper.h"

#include <stdexcept>

#include "drake/solvers/csdp_solver_error_handling.h"

// We must call this macro immediately prior to *any* call into CSDP.
#define DRAKE_CSDP_SETJMP()                                             \
  do {                                                                  \
    std::jmp_buf& env = get_per_thread_csdp_jmp_buf();                  \
    if (setjmp(env) > 0) {                                              \
      throw std::runtime_error(                                         \
          "CsdpSolver: the CSDP library exited via a fatal exception"); \
    }                                                                   \
  } while (0)

namespace drake {
namespace solvers {
namespace internal {
namespace csdp {

extern "C" {
#include <csdp/declarations.h>
}  // extern C

int cpp_easy_sdp(const char* params_pathname, int n, int k,
                 struct blockmatrix C, double* a,
                 struct constraintmatrix* constraints, double constant_offset,
                 struct blockmatrix* pX, double** py, struct blockmatrix* pZ,
                 double* ppobj, double* pdobj) {
  DRAKE_CSDP_SETJMP();
  return easy_sdp(params_pathname, n, k, C, a, constraints, constant_offset, pX,
                  py, pZ, ppobj, pdobj);
}

void cpp_free_mat(struct blockmatrix A) {
  DRAKE_CSDP_SETJMP();
  free_mat(A);
}

void cpp_free_prob(int n, int k, struct blockmatrix C, double* a,
                   struct constraintmatrix* constraints, struct blockmatrix X,
                   double* y, struct blockmatrix Z) {
  DRAKE_CSDP_SETJMP();
  free_prob(n, k, C, a, constraints, X, y, Z);
}

void cpp_initsoln(int n, int k, struct blockmatrix C, double* a,
                  struct constraintmatrix* constraints, struct blockmatrix* pX0,
                  double** py0, struct blockmatrix* pZ0) {
  DRAKE_CSDP_SETJMP();
  initsoln(n, k, C, a, constraints, pX0, py0, pZ0);
}

}  // namespace csdp
}  // namespace internal
}  // namespace solvers
}  // namespace drake
