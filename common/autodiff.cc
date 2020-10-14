#include "drake/common/autodiff.h"

drake::internal::Pool drake::internal::PoolVectorXd::pool_(128, 10);
