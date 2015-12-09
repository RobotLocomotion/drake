#ifndef DRAKE_SIMULATION_H
#define DRAKE_SIMULATION_H

namespace Drake {

  // simulation options
  struct SimulationOptions {
    double realtime_factor;  // 1 means try to run at realtime speed, < 0 is run as fast as possible
    double initial_step_size;

    SimulationOptions() :
            realtime_factor(-1.0),
            initial_step_size(0.01)
    {};
  };
  const static SimulationOptions default_simulation_options;

  template <typename Derived, template<typename> class StateVector, template<typename> class InputVector, template<typename> class OutputVector, bool isTimeVarying, bool isDirectFeedthrough>
  void simulate(const System<Derived,StateVector,InputVector,OutputVector,isTimeVarying,isDirectFeedthrough>& sys, double t0, double tf, const Eigen::VectorXd& x0, const SimulationOptions& options) {
    double t = t0, dt;
    std::cout << "x0 = " << x0.transpose() << std::endl;
    Eigen::Matrix<double,VectorTraits<StateVector<double> >::RowsAtCompileTime,1> x = x0;
    Eigen::Matrix<double,VectorTraits<StateVector<double> >::RowsAtCompileTime,1> xdot;
    Eigen::Matrix<double,VectorTraits<InputVector<double> >::RowsAtCompileTime,1> u(InputVector<double>::size()); u.setConstant(0);
    Eigen::Matrix<double,VectorTraits<OutputVector<double> >::RowsAtCompileTime,1> y;
    while (t<tf) {
      std::cout << "t=" << t << ", x = " << x.transpose() << std::endl;
      dt = (std::min)(options.initial_step_size,tf-t);
      y = sys.template output<double>(t,x,u);
      xdot = sys.template dynamics<double>(t,x,u);
      x += dt * xdot;
      t += dt;
    }
  }

  template <typename System>
  void simulate(const System& sys, double t0, double tf, const Eigen::VectorXd& x0)  {
    simulate(sys,t0,tf,x0,default_simulation_options);
  }

}  // end namespace Drake

#endif //DRAKE_SIMULATION_H
