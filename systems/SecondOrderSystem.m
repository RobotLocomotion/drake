classdef SecondOrderSystem < RobotLibSystem
% An abstract class that wraps qddot = f(t,q,qdot,u).
%   A specialization of the Dynamics class for systems of second order.  
  
  methods
    function obj = SecondOrderPlant(num_q, num_u,timeInvariantFlag)
    % SecondOrderPlant(num_q, num_u)
    %   Constructs a SecondOrderPlant object with num_q configuration
    %   variables (implying num_q*2 states) and num_u control inputs.

%      if (nargin>0)
        obj = obj@SmoothRobotLibSystem(num_q*2,0,num_u,num_q*2,false,timeInvariantFlag);
        obj = obj.setNumDOF(num_q);
%      end
    end
  end
  
  methods (Abstract=true)
    qdd = sodynamics(obj,t,q,qd,u)  % implements qdd = f(t,q,qd,u)
  end

  methods
    function obj = setNumDOF(obj,num_q)
    % Guards the num_q variable to make sure it stays consistent 
    % with num_x.
      obj = setNumContStates(obj,num_q*2);
    end

    function obj = setNumContStates(obj,num_xc)
    % Guards the num_xc variable to make sure it stays consistent
    % with num_q
      if (num_xc<0 || rem(num_xc,2)) error('num_x must be even for a SecondOrderDynamics'); end 
      obj.num_q = num_xc/2;
      obj = setNumContStates@SmoothRobotLibSystem(obj,num_xc);
      obj = setNumOutputs(obj,num_xc); 
    end
    
    function xdot = dynamics(obj,t,x,u)
    % Provides the dynamics interface for sodynamics
      q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
      qdd = obj.sodynamics(t,q,qd,u);
      xdot = [qd;qdd];
    end
    
    function y = output(obj,t,x,u)
      % default output is the full state
      y = x;
    end
        
  end
  
  properties (SetAccess = private, GetAccess = public)
    num_q  % number of configuration variables
  end
  
  
end
