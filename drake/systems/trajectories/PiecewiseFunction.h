#ifndef DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEFUNCTION_H_
#define DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEFUNCTION_H_

#include <Eigen/Core>
#include <vector>
#include <random>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeTrajectories_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

class DLLEXPORT PiecewiseFunction
{
protected:
  std::vector<double> segment_times;

public:
  PiecewiseFunction(std::vector<double> const & segment_times);

  virtual ~PiecewiseFunction();

  virtual Eigen::DenseIndex rows() const = 0;

  virtual Eigen::DenseIndex cols() const = 0;

  int getNumberOfSegments() const;

  double getStartTime(int segment_number) const;

  double getEndTime(int segment_number) const;

  double getDuration(int segment_number) const;

  double getStartTime() const;

  double getEndTime() const;

  int getSegmentIndex(double t) const;

  const std::vector<double>& getSegmentTimes() const;

  void segmentNumberRangeCheck(int segment_number) const;

  static std::vector<double> randomSegmentTimes(int num_segments, std::default_random_engine& generator);

protected:
  bool segmentTimesEqual(const PiecewiseFunction& b, double tol) const;

  void checkScalarValued() const;

  PiecewiseFunction();
};

#endif /* DRAKE_SYSTEMS_TRAJECTORIES_PIECEWISEFUNCTION_H_ */
