classdef SecondOrderDynamics < Dynamics
% SecondOrderDynamics
%   A specialization of the Dynamics class for systems of second order.  
  
  methods
    function obj = SecondOrderDynamics(num_q, num_inputs)
      obj = obj@Dynamics(num_q*2,num_inputs)
    end
  end
  
  methods (Abstract=true)
    qdd = sodynamics(obj,t,q,qd,u);  % implements qdd = f(t,q,qd,u)
  end

  methods
    function df = sodynamicsGradients(obj,t,q,qd,u,order)
      error('not implemented yet');
    end
    
    function obj = setNumQ(num_q)
      obj.m_num_q = num_q;
      obj = setNumStates@Dynamics(obj,num_q*2);
    end

    function obj = setNumStates(num_states)
      if (num_states<2 || rem(num_states,2)) error('num_states must be even and >= 2 for a SecondOrderDynamics'); end 
      obj.m_num_q = num_states/2;
      obj.setNumStates@Dynamics(obj,num_states);
    end
    
    function xdot = dynamics(obj,t,x,u);
      q=x(1:obj.m_num_q); qd=x((obj.m_num_q+1):end);
      qdd = obj.sodynamics(t,q,qd,u);
      xdot = [qd;qdd];
    end

    function df = dynamicsGradients(obj,t,x,u,order)
      if (nargin<5) order=1; end
      q=x(1:obj.numQ); qd=x((obj.numQ+1):end);
      df = obj.sodynamics_gradients(t,q,qd,u,order);
      df{1} = [zeros(obj.numQ,1+obj.numQ), eye(obj.numQ), zeros(obj.numQ,obj.numInputs); df{1}];
      z = zeros(obj.numQ,1+2*obj.numQ+obj.numInputs);
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
    m_num_q  % number of configuration variables
  end
  
  
end