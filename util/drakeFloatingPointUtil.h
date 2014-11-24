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
  return arg != std::numeric_limits<T>::infinity() &&
    arg != -std::numeric_limits<T>::infinity();
}

#endif /* DRAKEFLOATINGPOINTUTIL_H_ */