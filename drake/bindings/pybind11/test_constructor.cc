#include <iostream>
#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

typedef Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::VectorXd>, Eigen::Dynamic, 1> VectorXAutoDiffXd;

void init(VectorXAutoDiffXd& self) {
  self.resize(10);
}

int main() {
  VectorXAutoDiffXd x;
  // auto x = VectorXAutoDiffXd();
  init(x);
  std::cout << x.size() << std::endl;
  return 0;
}

