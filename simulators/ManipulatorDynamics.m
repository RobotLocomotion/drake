classdef ManipulatorDynamics < SecondOrderDynamics
% An abstract class that wraps H(q)qddot + C(q,qdot) = B(q)u.

  methods (Abstract=true)
    %  H(q)qdd + C(q,qd,f_ext) = Bu
    [H,C,B] = manipulatorDynamics(obj,q,qd);
  end

  methods
    function qdd = sodynamics(obj,t,q,qd,u)
    % Provides the SecondOrderDynamics interface to the manipulatorDynamics.
      [H,C,B] = manipulatorDynamics(obj,q,qd);
      qdd = H\(B*u - C);
    end

    % todo: implement gradients
  end
  
  % todo:  add a test function which automatically checks manipulator_dynamics vs. dynamics  
end