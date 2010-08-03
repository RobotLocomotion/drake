classdef SecondOrderDynamics < Dynamics
% An abstract class that wraps qddot = f(t,q,qdot,u).
%   A specialization of the Dynamics class for systems of second order.  
  
  methods
    function obj = SecondOrderDynamics(num_q, num_inputs)
    % SecondOrderDynamics(num_q, num_inputs)
    %   Constructs a SecondOrderDynamics object with num_q configuration
    %   variables (implying num_q*2 states) and num_inputs control inputs.

%      if (nargin>0)
        obj = obj@Dynamics(num_q*2,num_inputs);
%      end
    end
  end
  
  methods (Abstract=true)
    qdd = sodynamics(obj,t,q,qd,u);  % implements qdd = f(t,q,qd,u)
  end

  methods
    function df = sodynamicsGradients(obj,t,q,qd,u,order)
    % Provides the Taylor-expansion of the dynamics.
      error('not implemented yet');
    end
    
    function obj = setNumQ(obj,num_q)
    % Guards the num_q variable to make sure it stays consistent 
    % with num_states.
      obj = setNumStates(obj,num_q*2);
    end

    function obj = setNumStates(obj,num_states)
    % Guards the num_states variable to make sure it stays consistent
    % with num_q
      if (num_states<2 || rem(num_states,2)) error('num_states must be even and >= 2 for a SecondOrderDynamics'); end 
      obj.num_q = num_states/2;
      obj = setNumStates@Dynamics(obj,num_states);
    end
    
    function xdot = dynamics(obj,t,x,u);
    % Provides the dynamics interface for sodynamics
      q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
      qdd = obj.sodynamics(t,q,qd,u);
      xdot = [qd;qdd];
    end

    function df = dynamicsGradients(obj,t,x,u,order)
    % Provides the Taylor-expansion of the dynamics
      if (nargin<5) order=1; end
      q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
      df = obj.sodynamicsGradients(t,q,qd,u,order);
      df{1} = [zeros(obj.num_q,1+obj.num_q), eye(obj.num_q), zeros(obj.num_q,obj.num_inputs); df{1}];
      z = zeros(obj.num_q,1+2*obj.num_q+obj.num_inputs);
      for o=2:length(df)
        df{o} = addzeros(df{o});
      end
      
      function d = addzeros(d)
        if (iscell(d))
          for i=1:length(d)
            d{i} = addzeros(d{i});
          end
        else
          d = [z;d];
        end
      end
      
    end
    
  end
  
  properties (SetAccess = private, GetAccess = public)
    num_q  % number of configuration variables
  end
  
  
end