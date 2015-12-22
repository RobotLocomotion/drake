#ifndef DRAKE_LCMSYSTEM_H
#define DRAKE_LCMSYSTEM_H

#include <unordered_map>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drake/lcmt_drake_signal.hpp"
#include "System.h"
#include "Simulation.h"

namespace Drake {

  /** @defgroup lcm_vector_concept LCMVector<ScalarType> Concept
   * @ingroup vector_concept
   * @brief A specialization of the Vector concept adding the ability to read and publish LCM messages
   *
   * | Valid Expressions (which must be implemented) |  |
   * ------------------|-------------------------------------------------------------|
   * | LCMMessageType  | defined with a typedef                                      |
   * | channel         | defined as a const static std::string                       |
   * | bool encode(const double& t, const Vector<double>& x, LCMMessageType& msg) | define the mapping from your LCM type to your Vector type |
   * | bool decode(const LCMMessageType& msg, double& t, Vector<double>& x)  | define the mapping from your Vector type to your LCM type |
   */


  template <template <typename> class Vector>
  bool encode(const double& t, const Vector<double>& x, drake::lcmt_drake_signal& msg) {
    msg.timestamp = static_cast<int64_t>(t*1000);
    msg.dim = size(x);
    auto xvec = toEigen(x);
    for (int i=0; i<msg.dim; i++) {
      msg.coord.push_back(getCoordinateName(x,i));
      msg.val.push_back(xvec(i));
    }
    return true;
  }

  template <template <typename> class Vector>
  bool decode(const drake::lcmt_drake_signal& msg, double& t, Vector<double>& x) {
    t = double(msg.timestamp)/1000.0;
    std::unordered_map<std::string,double> m;
    for (int i=0; i<msg.dim; i++) {
      m[msg.coord[i]] = msg.val[i];
    }
    Eigen::Matrix<double,Vector<double>::RowsAtCompileTime,1> xvec(msg.dim);
    for (int i=0; i<msg.dim; i++) {
      xvec(i) = m[getCoordinateName(x,i)];
    }
    return true;
  }

  // If Vector implements the LCMPublisher interface, then this system will call the publish command on output
  template <template<typename> class Vector, typename Enable = void>
  class LCMOutputSystem
  {
  public:
    template <typename ScalarType> using StateVector = NullVector<ScalarType>;
    template <typename ScalarType> using InputVector = Vector<ScalarType>;
    template <typename ScalarType> using OutputVector = NullVector<ScalarType>;

    LCMOutputSystem(const std::shared_ptr<lcm::LCM>& lcm) {};

    StateVector<double> dynamics(const double& t, const StateVector<double>& x, const InputVector<double>& u) const
    { return StateVector<double>(); }

    OutputVector<double> output(const double& t, const StateVector<double>& x, const InputVector<double>& u) const
    {
      return OutputVector<double>();
    }
  };

  template <template<typename> class Vector>
  class LCMOutputSystem<Vector,typename std::enable_if<!std::is_void<typename Vector<double>::LCMMessageType>::value>::type>
  {
  public:
    template <typename ScalarType> using StateVector = NullVector<ScalarType>;
    template <typename ScalarType> using InputVector = Vector<ScalarType>;
    template <typename ScalarType> using OutputVector = NullVector<ScalarType>;

    LCMOutputSystem(const std::shared_ptr<lcm::LCM>& lcm) : lcm(lcm) {};

    StateVector<double> dynamics(const double& t, const StateVector<double>& x, const InputVector<double>& u) const
    { return StateVector<double>(); }

    OutputVector<double> output(const double& t, const StateVector<double>& x, const InputVector<double>& u) const
    {
      typename Vector<double>::LCMMessageType msg;
      if (!encode(t,u,msg))
        throw std::runtime_error(std::string("failed to encode")+msg.getTypeName());
      lcm->publish(u.channel(),&msg);
      return OutputVector<double>();
    }
  private:
    std::shared_ptr<lcm::LCM> lcm;
  };

  // todo: template specialization for the CombinedVector case


  /** runLCM
   * @brief Simulates the system with the (exposed) inputs being read from LCM and the output being published to LCM.
   * @ingroup simulation
   *
   * The input and output vector types must overload a publishLCM namespace method; the default for new vectors is to not publish anything.
   */

  template <typename System>
  void runLCM(const std::shared_ptr<System>& sys, const std::shared_ptr<lcm::LCM>& lcm, double t0, double tf, const typename System::template StateVector<double>& x0, const SimulationOptions& options) {
//    typename System::template OutputVector<double> x = 1;  // useful for debugging
    auto lcm_output = std::make_shared<LCMOutputSystem<System::template OutputVector> >(lcm);
    auto lcm_sys = cascade(sys,lcm_output);

    simulate(*lcm_sys,t0,tf,x0,options);
  }


} // end namespace Drake

#endif //DRAKE_LCMSYSTEM_H
