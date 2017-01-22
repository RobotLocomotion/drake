#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {
/**
 * Simple example x = b
 */
class LinearSystemExample1 {
 public:
  LinearSystemExample1() : prog_(std::make_shared<MathematicalProgram>()), x_{}, b_{}, con_{} {
    x_ = prog_->NewContinuousVariables<4>();
    b_ = Eigen::Vector4d::Random();
    con_ = prog_->AddLinearEqualityConstraint(Eigen::Matrix4d::Identity(), b_, x_);
    prog_->SetInitialGuessForAllVariables(Eigen::Vector4d::Zero());
  }

  std::shared_ptr<MathematicalProgram> prog() const {return prog_;}

  const VectorDecisionVariable<4>& x() const {return x_;}

  const Eigen::Vector4d b() const {return b_;}

  std::shared_ptr<LinearEqualityConstraint> con() const {return con_;}

  virtual bool CheckSolution() const {
    auto x_sol = prog_->GetSolution(x_);
    if (!CompareMatrices(x_sol, b_, 1E-10, MatrixCompareType::absolute)) {
      return false;
    }
    for (int i = 0; i < 4; ++i) {
      if (std::abs(x_sol(i) - b_(i)) > 1E-10) {
        return false;
      }
      if (!CompareMatrices(x_sol.head(i), b_.head(i), 1E-10, MatrixCompareType::absolute)) {
        return false;
      }
    }
    return true;
  }

 private:
  std::shared_ptr<MathematicalProgram> prog_;
  VectorDecisionVariable<4> x_;
  Eigen::Vector4d b_;
  std::shared_ptr<LinearEqualityConstraint> con_;
};

/**
 * Simple linear system
 *     x    = b
 * 2 * y(0) = b(0)
 * 2 * y(1) = b(1)
 */
class LinearSystemExample2 : public LinearSystemExample1 {
 public:
  LinearSystemExample2() : LinearSystemExample1(), y_{} {
    y_ = prog()->NewContinuousVariables<2>();
    prog()->AddLinearEqualityConstraint(2 * Eigen::Matrix2d::Identity(), b().topRows<2>(), y_);
  }

  VectorDecisionVariable<2> y() const {return y_;}

  bool CheckSolution() const override {
    if (!LinearSystemExample1::CheckSolution()) {
      return false;
    }
    if (!CompareMatrices(prog()->GetSolution(y_), b().topRows<2>()/2, 1E-10, MatrixCompareType::absolute)) {
      return false;
    }
    return true;
  }

 private:
  VectorDecisionVariable<2> y_;
};

/**
 * Simple linear system
 * 3 * x    = b
 * 2 * y(0) = b(0)
 * 2 * y(1) = b(1)
 */
class LinearSystemExample3 : public LinearSystemExample2 {
 public:
  LinearSystemExample3() : LinearSystemExample2() {
    con()->UpdateConstraint(3 * Eigen::Matrix4d::Identity(), b());
  }

  bool CheckSolution() const override {
    if (!CompareMatrices(prog()->GetSolution(x()), b()/3, 1E-10, MatrixCompareType::absolute)) {
      return false;
    }
    if (!CompareMatrices(prog()->GetSolution(y()), b().topRows<2>() / 2, 1E-10, MatrixCompareType::absolute)) {
      return false;
    }
    return true;
  }
};

/// This test comes from Section 2.2 of "Handbook of Test Problems in
/// Local and Global Optimization."
class NonConvexQPproblem1 {
  /// This is a non-convex quadratic program with inequality constraints.
  /// We choose to add the cost and constraints through different forms,
  /// to test different solvers, and whether MathematicalProgram can parse
  /// constraints in different forms.
 public:
  enum CostForm {
    kCostBegin = 0,
    kGenericCost = 0,
    kQuadraticCost = 1,
    kCostEnd = 1
    // TODO(hongkai.dai) Add quadratic symbolic cost
  };

  enum ConstraintForm {
    kConstraintBegin = 0,
    kSymbolicConstraint = 0,
    kNonSymbolicConstraint = 1,
    kConstraintEnd = 1
  };

 public:
  NonConvexQPproblem1(CostForm cost_form, ConstraintForm constraint_form) : prog_(std::make_shared<MathematicalProgram>()), x_{}, x_expected_{} {
    x_ = prog_->NewContinuousVariables<5>("x");
    prog_->AddBoundingBoxConstraint(0, 1, x_);
    switch (cost_form) {
      case kGenericCost : {
        prog_->AddCost(TestProblem1Cost());
        break;
      }
      case kQuadraticCost : {
        AddQuadraticCost();
        break;
      }
      default:
        throw std::runtime_error("unsupported cost form");
    }
    switch (constraint_form) {
      case kSymbolicConstraint : {
        AddSymbolicConstraint();
        break;
      }
      case kNonSymbolicConstraint : {
        AddConstraint();
        break;
      }
      default:
        throw std::runtime_error("unsupported constraint form");
    }

    x_expected_ << 1, 1, 0, 1, 0;
    prog_->SetInitialGuess(x_, x_expected_ + 0.01 * Eigen::Matrix<double, 5, 1>::Random());
  }

  std::shared_ptr<MathematicalProgram> prog() const {return prog_;}

  bool CheckSolution() const {
    const auto& x_value = prog_->GetSolution(x_);
    return CompareMatrices(x_value, x_expected_, 1E-9, MatrixCompareType::absolute);
  }

 private:
  class TestProblem1Cost {
   public:
    static size_t numInputs() { return 5; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    void eval(detail::VecIn<ScalarType> const& x, detail::VecOut<ScalarType>& y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
      y(0) = (-50.0 * x(0) * x(0)) + (42 * x(0)) - (50.0 * x(1) * x(1)) +
          (44 * x(1)) - (50.0 * x(2) * x(2)) + (45 * x(2)) -
          (50.0 * x(3) * x(3)) + (47 * x(3)) - (50.0 * x(4) * x(4)) +
          (47.5 * x(4));
    }
  };

  void AddConstraint() {
    Eigen::Matrix<double, 1, 5> a;
    a << 20, 12, 11, 7, 4;
    prog_->AddLinearConstraint(a, -std::numeric_limits<double>::infinity(), 40);
  }

  void AddSymbolicConstraint() {
    const auto constraint = 20 * x_(0) + 12 * x_(1) + 11 * x_(2) + 7 * x_(3) + 4 * x_(4);
    prog_->AddLinearConstraint(constraint, -std::numeric_limits<double>::infinity(), 40);
  }

  void AddQuadraticCost() {
    Eigen::Matrix<double, 5, 5> Q = -100 * Eigen::Matrix<double, 5, 5>::Identity();
    Eigen::Matrix<double, 5, 1> c;
    c << 42, 44, 45, 47, 47.5;
    prog_->AddQuadraticCost(Q, c);
  }

  std::shared_ptr<MathematicalProgram> prog_;
  VectorDecisionVariable<5> x_;
  Eigen::Matrix<double, 5, 1> x_expected_;
};

/// This test comes from Section 2.3 of "Handbook of Test Problems in
/// Local and Global Optimization."
class NonConvexQPproblem2 {
 public:
  enum CostForm {
    kCostBegin = 0,
    kGenericCost = 0,
    kQuadraticCost = 1,
    // TODO(hongkai.dai): Add symbolic quadratic cost
    kCostEnd = 1
  };

  enum ConstraintForm {
    kConstraintBegin = 0,
    kNonSymbolicConstraint = 0,
    kSymbolicConstraint = 1,
    kConstraintEnd = 1
  };

 public:
  NonConvexQPproblem2(CostForm cost_form, ConstraintForm cnstr_form) :
      prog_(std::make_shared<MathematicalProgram>()), x_{}, x_expected_{} {
    x_ = prog_->NewContinuousVariables<6>("x");

    prog_->AddBoundingBoxConstraint(0, 1, x_.head<5>());
    prog_->AddBoundingBoxConstraint(0, std::numeric_limits<double>::infinity(), x_(5));

    switch (cost_form) {
      case kGenericCost : {
        prog_->AddCost(TestProblem2Cost());
        break;
      }
      case kQuadraticCost : {
        AddQuadraticCost();
        break;
      }
      default : throw std::runtime_error("Unsupported cost form");
    }

    switch (cnstr_form) {
      case kNonSymbolicConstraint : {
        AddNonSymbolicConstraint();
        break;
      }
      case kSymbolicConstraint : {
        AddSymbolicConstraint();
        break;
      }
      default : throw std::runtime_error("Unsupported constraint form");
    }

    x_expected_ << 0, 1, 0, 1, 1, 20;
    prog_->SetInitialGuess(x_, x_expected_ + 0.01 * Eigen::Matrix<double, 6, 1>::Random());
  }

  bool CheckSolution() const {
    const auto& x_value = prog_->GetSolution(x_);
    return CompareMatrices(x_value, x_expected_, 1E-3, MatrixCompareType::absolute);
  }

  std::shared_ptr<MathematicalProgram> prog() const {return prog_;}

 private:
  class TestProblem2Cost {
   public:
    static size_t numInputs() { return 6; }
    static size_t numOutputs() { return 1; }

    template<typename ScalarType>
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    void eval(detail::VecIn <ScalarType> const &x,
              detail::VecOut <ScalarType> &y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
      y(0) = (-50.0 * x(0) * x(0)) + (-10.5 * x(0)) - (50.0 * x(1) * x(1)) +
          (-7.5 * x(1)) - (50.0 * x(2) * x(2)) + (-3.5 * x(2)) -
          (50.0 * x(3) * x(3)) + (-2.5 * x(3)) - (50.0 * x(4) * x(4)) +
          (-1.5 * x(4)) + (-10.0 * x(5));
    }
  };

  void AddQuadraticCost() {
    Eigen::Matrix<double, 6, 6> Q = -100.0 * Eigen::Matrix<double, 6, 6>::Identity();
    Q(5, 5) = 0.0;
    Eigen::Matrix<double, 6, 1> c{};
    c << -10.5, -7.5, -3.5, -2.5, -1.5, -10.0;

    prog_->AddQuadraticCost(Q, c);
  }

  void AddNonSymbolicConstraint() {
    Eigen::Matrix<double, 1, 6> a1{};
    Eigen::Matrix<double, 1, 6> a2{};
    a1 << 6, 3, 3, 2, 1, 0;
    a2 << 10, 0, 10, 0, 0, 1;
    prog_->AddLinearConstraint(a1, -std::numeric_limits<double>::infinity(), 6.5);
    prog_->AddLinearConstraint(a2, -std::numeric_limits<double>::infinity(), 20);
  }

  void AddSymbolicConstraint() {
    const symbolic::Expression constraint1{6 * x_(0) + 3 * x_(1) + 3 * x_(2) + 2 * x_(3) + x_(4)};
    const symbolic::Expression constraint2{10 * x_(0) + 10 * x_(2) + x_(5)};
    prog_->AddLinearConstraint(constraint1, -std::numeric_limits<double>::infinity(), 6.5);
    prog_->AddLinearConstraint(constraint2, -std::numeric_limits<double>::infinity(), 20);
  }

  std::shared_ptr<MathematicalProgram> prog_;
  Eigen::Matrix<symbolic::Variable, 6, 1> x_;
  Eigen::Matrix<double, 6, 1> x_expected_;
};


/// This test comes from Section 3.4 of "Handbook of Test Problems in
/// Local and Global Optimization.
class LowerBoundedProblem {
 public:
  enum ConstraintForm {
    kConstraintBegin = 0,
    kNonSymbolic = 0,
    kSymbolic = 1,
    kConstraintEnd = 1
  };

 public:
  LowerBoundedProblem(ConstraintForm cnstr_form) : prog_(std::make_shared<MathematicalProgram>()), x_{}, x_expected_{} {
    x_ = prog_->NewContinuousVariables<6>("x");

    Eigen::Matrix<double, 6, 1> lb{};
    Eigen::Matrix<double, 6, 1> ub{};
    lb << 0, 0, 1, 0, 1, 0;
    ub << std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 5, 6, 5, 10;
    prog_->AddBoundingBoxConstraint(lb, ub);

    prog_->AddCost(LowerBoundTestCost());
    std::shared_ptr<Constraint> con1(new LowerBoundTestConstraint(2, 3));
    prog_->AddConstraint(con1);
    std::shared_ptr<Constraint> con2(new LowerBoundTestConstraint(4, 5));
    prog_->AddConstraint(con2);

    switch (cnstr_form) {
      case kNonSymbolic: {
        AddNonSymbolicConstraint();
        break;
      }
      case kSymbolic: {
        AddSymbolicConstraint();
        break;
      }
      default : throw std::runtime_error("Not a supported constraint form");
    }

    x_expected_ << 5, 1, 5, 0, 5, 10;
  }

  bool CheckSolution() const {
    const auto& x_value = prog_->GetSolution(x_);
    return CompareMatrices(x_value, x_expected_, 1E-3, MatrixCompareType::absolute);
  }

  std::shared_ptr<MathematicalProgram> prog() {return prog_;}

  void SetInitialGuess1() {
    std::srand(0);
    Eigen::Matrix<double, 6, 1> delta = 0.05 * Eigen::Matrix<double, 6, 1>::Random();
    prog_->SetInitialGuess(x_, x_expected_ + delta);
  }

  void SetInitialGuess2() {
    std::srand(0);
    Eigen::Matrix<double, 6, 1> delta = 0.05 * Eigen::Matrix<double, 6, 1>::Random();
    prog_->SetInitialGuess(x_, x_expected_ - delta);
  }

 private:
  class LowerBoundTestCost {
   public:
    static size_t numInputs() { return 6; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    void eval(detail::VecIn<ScalarType> const& x, detail::VecOut<ScalarType>& y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
      y(0) = -25 * (x(0) - 2) * (x(0) - 2) + (x(1) - 2) * (x(1) - 2) -
          (x(2) - 1) * (x(2) - 1) - (x(3) - 4) * (x(3) - 4) -
          (x(4) - 1) * (x(4) - 1) - (x(5) - 4) * (x(5) - 4);
    }
  };

  class LowerBoundTestConstraint : public Constraint {
   public:
    LowerBoundTestConstraint(int i1, int i2)
        : Constraint(1, Eigen::Dynamic, Vector1d::Constant(4),
                     Vector1d::Constant(std::numeric_limits<double>::infinity())),
          i1_(i1),
          i2_(i2) {}

   protected:
    // for just these two types, implementing this locally is almost cleaner...
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd& y) const override {
      EvalImpl(x, y);
    }
    void DoEval(const Eigen::Ref<const TaylorVecXd>& x,
                TaylorVecXd& y) const override {
      EvalImpl(x, y);
    }

   private:
    template <typename ScalarType>
    void EvalImpl(
        const Eigen::Ref<const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>>& x,
        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
        Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& y) const {
      y.resize(1);
      y(0) = (x(i1_) - 3) * (x(i1_) - 3) + x(i2_);
    }

    int i1_;
    int i2_;
  };
 private:
  void AddSymbolicConstraint() {
    prog_->AddLinearConstraint(x_(0) - 3 * x_(1), -std::numeric_limits<double>::infinity(), 2);
    prog_->AddLinearConstraint(-x_(0) + x_(1), -std::numeric_limits<double>::infinity(), 2);
    prog_->AddLinearConstraint(x_(0) + x_(1), -std::numeric_limits<double>::infinity(), 6);
  }

  void AddNonSymbolicConstraint() {
    prog_->AddLinearConstraint(Eigen::RowVector2d(1, -3), -std::numeric_limits<double>::infinity(), 2, x_.head<2>());
    prog_->AddLinearConstraint(Eigen::RowVector2d(-1, 1), -std::numeric_limits<double>::infinity(), 2, x_.head<2>());
    prog_->AddLinearConstraint(Eigen::RowVector2d(1, 1), -std::numeric_limits<double>::infinity(), 6, x_.head<2>());
  }

 private:
  std::shared_ptr<MathematicalProgram> prog_;
  Eigen::Matrix<symbolic::Variable, 6, 1> x_;
  Eigen::Matrix<double, 6, 1> x_expected_;
};


/// gloptiPolyConstrainedMinimization
/// @brief from section 5.8.2 of the gloptipoly3 documentation
///
/// Which is from section 3.5 in
///   Handbook of Test Problems in Local and Global Optimization
/// We deliberately duplicate the problem, with the same constraints and
/// costs on decision variables x and y, so as to test out program works
/// correctly with multiple decision variables.
class GloptiPolyConstrainedMinimizationProblem {
 public:
  enum CostForm {
    kCostBegin = 0,
    kGenericCost = 0,
    kNonSymbolicCost = 1,
    // TODO(hongkai.dai): add symbolic linear cost
    kCostEnd = 1
  };

  enum ConstraintForm {
    kConstraintBegin = 0,
    kNonSymbolicConstraint = 0,
    kSymbolicConstraint = 1,
    // TODO(hongkai.dai): add quadratic constraint
    kConstraintEnd = 1
  };

 public:
  GloptiPolyConstrainedMinimizationProblem(CostForm cost_form, ConstraintForm cnstr_form) :
      prog_(std::make_shared<MathematicalProgram>()), x_{}, y_{}, expected_(0.5, 0, 3) {
    x_ = prog_->NewContinuousVariables<3>("x");
    y_ = prog_->NewContinuousVariables<3>("y");

    prog_->AddBoundingBoxConstraint(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, std::numeric_limits<double>::infinity(), 3), x_);
    prog_->AddBoundingBoxConstraint(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(2, std::numeric_limits<double>::infinity(), 3), y_);

    switch (cost_form) {
      case kGenericCost : {
        AddGenericCost();
        break;
      }
      case kNonSymbolicCost : {
        AddSymbolicCost();
        break;
      }
      default : throw std::runtime_error("Not a supported cost form");
    }

    // TODO(hongkai.dai): write this in symbolic form also.
    std::shared_ptr<GloptipolyConstrainedExampleConstraint> qp_con(new GloptipolyConstrainedExampleConstraint());
    prog_->AddConstraint(qp_con, x_);
    prog_->AddConstraint(qp_con, y_);

    switch (cnstr_form) {
      case kNonSymbolicConstraint : {
        AddNonSymbolicConstraint();
        break;
      }
      case kSymbolicConstraint : {
        AddSymbolicConstraint();
        break;
      }
      default : throw std::runtime_error("Not a supported constraint form");
    }

    Eigen::Vector3d initial_guess = expected_ + 0.01 * Eigen::Vector3d::Random();
    prog_->SetInitialGuess(x_, initial_guess);
    prog_->SetInitialGuess(y_, initial_guess);
  }

  std::shared_ptr<MathematicalProgram> prog() const {return prog_;}

  bool CheckSolution() const {
    const auto& x_value = prog_->GetSolution(x_);
    const auto& y_value = prog_->GetSolution(y_);
    return (CompareMatrices(x_value, expected_, 1E-4, MatrixCompareType::absolute)) &&
    (CompareMatrices(y_value, expected_, 1E-4, MatrixCompareType::absolute));
  }

 private:
  class GloptipolyConstrainedExampleCost {
   public:
    static size_t numInputs() { return 3; }
    static size_t numOutputs() { return 1; }

    template <typename ScalarType>
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    void eval(detail::VecIn<ScalarType> const& x, detail::VecOut<ScalarType>& y) const {
      DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
      DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
      y(0) = -2 * x(0) + x(1) - x(2);
    }
  };

  class GloptipolyConstrainedExampleConstraint
      : public Constraint {  // want to also support deriving directly from
    // constraint without going through drake::Function
   public:
    GloptipolyConstrainedExampleConstraint()
        : Constraint(1, 3, Vector1d::Constant(0),
                     Vector1d::Constant(std::numeric_limits<double>::infinity())) {}

   protected:
    // for just these two types, implementing this locally is almost cleaner...
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd& y) const override {
      EvalImpl(x, y);
    }
    void DoEval(const Eigen::Ref<const TaylorVecXd>& x,
                TaylorVecXd& y) const override {
      EvalImpl(x, y);
    }

   private:
    template <typename ScalarType>
    void EvalImpl(const Eigen::Ref<const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>>& x,
        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                  Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& y) const {
      y.resize(1);
      y(0) = 24 - 20 * x(0) + 9 * x(1) - 13 * x(2) + 4 * x(0) * x(0) -
          4 * x(0) * x(1) + 4 * x(0) * x(2) + 2 * x(1) * x(1) -
          2 * x(1) * x(2) + 2 * x(2) * x(2);
    }
  };

 private:
  void AddGenericCost() {
    prog_->AddCost(GloptipolyConstrainedExampleCost(), x_);
    prog_->AddCost(GloptipolyConstrainedExampleCost(), y_);
  }

  void AddSymbolicCost() {
    prog_->AddLinearCost(Eigen::Vector3d(-2, 1, -1), x_);
    prog_->AddLinearCost(Eigen::Vector3d(-2, 1, -1), y_);
  }

  void AddNonSymbolicConstraint() {
    Eigen::Matrix<double, 2, 3> A{};
    // clang-format off
    A << 1, 1, 1,
         0, 3, 1;
    // clang-format on
    prog_->AddLinearConstraint(A, Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity()), Eigen::Vector2d(4, 6), x_);
    prog_->AddLinearConstraint(A, Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity()), Eigen::Vector2d(4, 6), y_);
  }

  void AddSymbolicConstraint() {
    Eigen::Matrix<symbolic::Expression, 2, 1> e1{};
    Eigen::Matrix<symbolic::Expression, 2, 1> e2{};
    // clang-format off
    e1 << x_(0) + x_(1) + x_(2),
              3 * x_(1) + x_(2);
    e2 << y_(0) + y_(1) + y_(2),
              3 * y_(1) + y_(2);
    // clang-format on
    prog_->AddLinearConstraint(e1, Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity()), Eigen::Vector2d(4, 6));
    prog_->AddLinearConstraint(e2, Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity()), Eigen::Vector2d(4, 6));
  }

 private:
  std::shared_ptr<MathematicalProgram> prog_;
  VectorDecisionVariable<3> x_;
  VectorDecisionVariable<3> y_;
  Eigen::Vector3d expected_;
};
}  // namespace test
}  // namespace solvers
}  // namespace drake