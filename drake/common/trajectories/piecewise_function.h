#pragma once

#include <random>
#include <vector>

#include <Eigen/Core>

class PiecewiseFunction {
 protected:
  std::vector<double> breaks;

 public:
  /// Minimum delta quantity used for comparing time.
  static constexpr double kEpsilonTime = 1e-10;

  /// @throws std::runtime_exception if `breaks` increments are
  /// smaller than kEpsilonTime.
  explicit PiecewiseFunction(std::vector<double> const& breaks);

  virtual ~PiecewiseFunction();

  virtual Eigen::Index rows() const = 0;

  virtual Eigen::Index cols() const = 0;

  int getNumberOfSegments() const;

  double getStartTime(int segment_number) const;

  double getEndTime(int segment_number) const;

  double getDuration(int segment_number) const;

  double getStartTime() const;

  double getEndTime() const;

  int getSegmentIndex(double t) const;

  const std::vector<double>& getSegmentTimes() const;

  void segmentNumberRangeCheck(int segment_number) const;

  static std::vector<double> randomSegmentTimes(
      // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references)
      int num_segments, std::default_random_engine& generator);

 protected:
  bool segmentTimesEqual(const PiecewiseFunction& b,
      double tol = kEpsilonTime) const;

  void checkScalarValued() const;

  PiecewiseFunction();

 private:
  int GetSegmentIndexRecursive(double time, int start, int end) const;
};
