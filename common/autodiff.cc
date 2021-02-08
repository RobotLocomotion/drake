#include "drake/common/autodiff.h"

__thread drake::internal::Pool* drake::internal::PoolVectorXd::pool_{};
