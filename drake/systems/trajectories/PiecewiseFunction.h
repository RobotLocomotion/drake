#ifndef DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEFUNCTION_H_
#define DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEFUNCTION_H_

#include <Eigen/Core>
#include <vector>
#include <random>
#include "drake/drakeTrajectories_export.h"

class DRAKETRAJECTORIES_EXPORT PiecewiseFunction {
 protected:
  std::vector<double> segment_times;

 public:
  PiecewiseFunction(std::vector<double> const& segment_times);

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
      int num_segments, std::default_random_engine& generator);

 protected:
  bool segmentTimesEqual(const PiecewiseFunction& b, double tol) const;

  void checkScalarValued() const;

  PiecewiseFunction();
};

#endif /* DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEFUNCTION_H_ */
