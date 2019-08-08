#pragma once

#include <memory>
#include <string>
#include <vector>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "drake/common/eigen_types.h"
#include "drake/common/proto/matlab_rpc.pb.h"

// TODO(jamiesnape): Merge the contents of this header and the accompanying
// source file into call_python.

namespace drake {
namespace common {

namespace internal {

void ToMatlabArrayMatrix(
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>>&
        mat,
    MatlabArray* matlab_array, bool is_vector);

void ToMatlabArrayMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                         MatlabArray* matlab_array, bool is_vector);

void ToMatlabArrayMatrix(const Eigen::Ref<const Eigen::MatrixXi>& mat,
                         MatlabArray* matlab_array, bool is_vector);

}  // namespace internal

void ToMatlabArray(double scalar, MatlabArray* matlab_array);

void ToMatlabArray(int scalar, MatlabArray* matlab_array);

void ToMatlabArray(const std::string& str, MatlabArray* matlab_array);

template <typename Derived>
void ToMatlabArray(const Eigen::MatrixBase<Derived>& mat,
                   MatlabArray* matlab_array) {
  const bool is_vector = (Derived::ColsAtCompileTime == 1);
  return internal::ToMatlabArrayMatrix(mat, matlab_array, is_vector);
}

// Helper methods for variadic template call in CallMatlab.
namespace internal {
inline void AssembleCallMatlabMsg(MatlabRPC*) {
  // Intentionally left blank.  Base case for template recursion.
}

template <typename T, typename... Types>
void AssembleCallMatlabMsg(MatlabRPC* msg, T first, Types... args) {
  ToMatlabArray(first, msg->add_rhs());
  AssembleCallMatlabMsg(msg, args...);
}

std::unique_ptr<google::protobuf::io::FileOutputStream>
CreateOutputStream(const std::string& filename);

void PublishCall(
    google::protobuf::io::FileOutputStream* praw_output,
    const MatlabRPC& message);

}  // namespace internal

}  // namespace common
}  // namespace drake
