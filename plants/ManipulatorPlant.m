classdef ManipulatorPlant < SecondOrderPlant
% An abstract class that wraps H(q)qddot + C(q,qdot,f_ext) = B(q)u.

  methods
    function obj = ManipulatorPlant(num_q, num_u)
      obj = obj@SecondOrderPlant(num_q,num_u,true);
    end
  end
  
  methods (Abstract=true)
    %  H(q)qddot + C(q,qdot,f_ext) = Bu
    [H,C,B] = manipulatorDynamics(obj,q,qd);
  end

  methods
    function qdd = sodynamics(obj,t,q,qd,u)
    % Provides the SecondOrderDynamics interface to the manipulatorDynamics.
      [H,C,B] = manipulatorDynamics(obj,q,qd);
      qdd = H\(B*u - C);
    end
  end
  
end