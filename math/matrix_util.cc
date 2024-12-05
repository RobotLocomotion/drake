#include "drake/math/matrix_util.h"

#include <sstream>

#include <fmt/format.h>
#include <fmt/ranges.h>

namespace drake {
namespace math {
std::string GeneratePythonCsc(const Eigen::SparseMatrix<double>& mat,
                              std::string_view name) {
  std::stringstream python_stream;
  if (mat.nonZeros() == 0) {
    python_stream << fmt::format("{} = sparse.csc_matrix(({}, {}))\n", name,
                                 mat.rows(), mat.cols());
  } else {
    std::vector<double> data;
    std::vector<int> rows;
    std::vector<int> cols;
    data.reserve(mat.nonZeros());
    rows.reserve(mat.nonZeros());
    cols.reserve(mat.nonZeros());
    for (int k = 0; k < mat.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(mat, k); it; ++it) {
        data.push_back(it.value());
        rows.push_back(it.row());
        cols.push_back(it.col());
      }
    }

    python_stream << fmt::format(
        R"""({} = sparse.csc_matrix((np.array([{}], dtype=np.float64), ([{}], [{}])), shape=({}, {}))
)""",
        name, fmt::join(data, ", "), fmt::join(rows, ", "),
        fmt::join(cols, ", "), mat.rows(), mat.cols());
  }
  return python_stream.str();
}
}  // namespace math
}  // namespace drake
