#pragma once
#include <Eigen/Core>

Eigen::MatrixXi EnumerateIntegerSolutions(const Eigen::MatrixXd & A,
                                const Eigen::MatrixXd & b,
                                const Eigen::VectorXi & lower_bound,
                                const Eigen::VectorXi & upper_bound);

Eigen::MatrixXi EnumerateIntegerSolutions(const Eigen::MatrixXi & A,
                                const Eigen::MatrixXi & b,
                                const Eigen::VectorXi & lower_bound,
                                const Eigen::VectorXi & upper_bound);
