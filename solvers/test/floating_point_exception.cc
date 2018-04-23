#include <mosek.h>

int main() {
  MSKtask_t task = NULL;
  MSKrescodee rescode;

  MSKenv_t env = NULL;

  rescode = MSK_makeenv(&env, NULL);
  rescode = MSK_checkoutlicense(env, MSK_FEATURE_PTS);

  rescode = MSK_maketask(env, 0, 0, &task);
  if (rescode == MSK_RES_OK)
    rescode = MSK_readdata(task, "/home/hongkai/drake-distro/problem.task.gz");
  if (rescode == MSK_RES_OK)
    rescode = MSK_optimize(task);
  MSKsolstae solution_status;
  rescode = MSK_getsolsta(task, MSK_SOL_ITR, &solution_status);
  return 0;
}
