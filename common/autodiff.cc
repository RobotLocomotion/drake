#include "drake/common/autodiff.h"

drake::internal::Pool& drake::internal::PoolVectorXd::pool_ =
    drake::internal::PoolVectorXd::static_pool();
