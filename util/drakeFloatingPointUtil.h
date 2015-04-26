#ifndef DRAKEFLOATINGPOINTUTIL_H_
#define DRAKEFLOATINGPOINTUTIL_H_

#include <limits>

/*
 * substitute for std::isfinite from <limits> in C++11, which is not available on MSVC2010
 */
template<typename T> bool isFinite(T arg)
{
  return arg == arg && 
    arg != std::numeric_limits<T>::infinity() &&
    arg != -std::numeric_limits<T>::infinity();
}

/*
 * substitute for std::isinf from <limits> in C++11, which is not available on MSVC2010
 */
template<typename T> bool isInf(T arg)
{
  return arg == std::numeric_limits<T>::infinity() ||
    arg == -std::numeric_limits<T>::infinity();
}

inline bool isNaN(double x) {
#ifdef WIN32
  return _isnan(x) != 0;
#else
  return std::isnan(x);
#endif
}

/*
 * from http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
 * returns 0 when val is +0 or -0
 */
template <typename T> int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

#endif /* DRAKEFLOATINGPOINTUTIL_H_ */
