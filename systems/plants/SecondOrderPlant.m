classdef SecondOrderPlant < SmoothRobotLibSystem
% An abstract class that wraps qddot = f(t,q,qdot,u).
%   A specialization of the Dynamics class for systems of second order.  
  
  methods
    function obj = SecondOrderPlant(num_q, num_u,timeInvariantFlag)
    % SecondOrderPlant(num_q, num_u)
    %   Constructs a SecondOrderPlant object with num_q configuration
    %   variables (implying num_q*2 states) and num_u control inputs.

%      if (nargin>0)
        obj = obj@SmoothRobotLibSystem(num_q*2,0,num_u,num_q*2,false,timeInvariantFlag);
        obj.num_q = num_q;
%      end
    end
  end
  
  methods (Abstract=true)
    qdd = sodynamics(obj,t,q,qd,u)  % implements qdd = f(t,q,qd,u)
  end

  methods
    function obj = setNumQ(obj,num_q)
    % Guards the num_q variable to make sure it stays consistent 
    % with num_x.
      obj = setNumX(obj,num_q*2);
    end

    function obj = setNumX(obj,num_x)
    % Guards the num_x variable to make sure it stays consistent
    % with num_q
      if (num_x<2 || rem(num_x,2)) error('num_x must be even and >= 2 for a SecondOrderDynamics'); end 
      obj.num_q = num_x/2;
      obj = setNumX@Plant(obj,num_x);
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