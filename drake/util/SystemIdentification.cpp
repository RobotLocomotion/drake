#include "drake/util/SystemIdentification.h"

namespace drake {
namespace util {

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomial(
    PolyType poly,
    std::set<VarType> vars_of_interest) {
  return SystemIdentification<T>::LumpingMapType();
}

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomials(
    std::set<PolyType> polys,
    std::set<VarType> vars_of_interest) {
  return SystemIdentification<T>::LumpingMapType();
}

template<typename T>
typename SystemIdentification<T>::PolyType
SystemIdentification<T>::RewritePolynomialWithLumpedParameters(
    PolyType poly,
    LumpingMapType lumped_parameters) {
  return PolyType();
}

};
};

template class DRAKEPOLYNOMIAL_EXPORT drake::util::SystemIdentification<double>;
