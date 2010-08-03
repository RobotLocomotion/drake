classdef ManipulatorDynamics < SecondOrderDynamics
  % An interface class 

  methods (Abstract=true)
    %  H(q)qdd + C(q,qd,f_ext) = Bu
    [H,C,B] = manipulator_dynamics(obj,q,qd);
  end

  methods
    function qdd = sodynamics(obj,t,q,qd,u)
      [H,C,B] = manipulator_dynamics(obj,q,qd);
      qdd = H\(B*u - C);
    end

    % todo: implement gradients
  end
  
  % todo:  add a test function which automatically checks manipulator_dynamics vs. dynamics  
end