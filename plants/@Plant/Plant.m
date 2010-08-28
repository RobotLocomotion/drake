classdef Plant
% An abstract class which wraps a dynamics function xdot=f(t,x,u).
%   Provides a common interface and many supporting methods to work with 
%   the dynamics, and implements an S-Function interface (so also 
%   works as a simulink block.) 
  

  % constructor
  methods 
    function obj = Plant(num_x,num_u)
      % Plant class constructor
      %   sets the dimension of the state and input vectors
      if (nargin>0)
        obj = setNumX(obj,num_x);  
        obj = setNumU(obj,num_u);
      end
    end      
  end
  
  % methods that MUST be implemented
  methods (Abstract=true)
    xdot = dynamics(obj,t,x,u);   % implements xdot = f(t,x,u)
  end
  
  % methods that CAN/SHOULD be implemented/overridden
  methods
    function df = dynamicsGradients(obj,t,x,u,order)
      % Computes the Taylor-expansion of the dynamics around a point.
      error('not implemented yet');
    end
  end  
  
  % methods that wrap basic functionality
  methods 
    function obj = setNumX(obj,num_x)
      % Guards the num_states variable
      if (num_x<1) error('num_x must be > 0'); end
      obj.num_x = num_x;
    end
    
    function x = getInitialState(obj)
      % Returns the default initial conditions
      x = zeros(obj.num_x,1);
    end
    
    function obj = setNumU(obj,num_u)
      % Guards the num_u variable.
      %  Also pads umin and umax for any new inputs with [-inf,inf].

      if (num_u<0) error('num_u must be >=0'); end
      obj.num_u = num_u;
      
       % cut umin and umax to the right size, and pad new inputs with
      % [-inf,inf]
      if (length(obj.umin)~=1 && length(obj.umin)~=num_u)
        obj.umin = [obj.umin(1:num_u); -inf*ones(max(num_u-length(obj.umin),0),1)];
      end
      if (length(obj.umax)~=1 && length(obj.umax)~=num_u)
        obj.umax = [obj.umax(1:num_u); inf*ones(max(num_u-obj.length(obj.umax),0),1)];
      end
      
    end
    
    function obj = setInputLimits(obj,umin,umax)
      % Guards the input limits to make sure it stay consistent
      
      if (length(umin)~=1 && length(umin)~=obj.num_u) error('umin is the wrong size'); end
      if (length(umax)~=1 && length(umax)~=obj.num_u) error('umax is the wrong size'); end
      if (any(obj.umax<obj.umin)) error('umin must be less than umax'); end
      obj.umin = umin;
      obj.umax = umax;
    end
    
    function u = getDefaultInput(obj)
      % Define the default initial input so that behavior is well-defined
      % if no controller is specified or if no control messages have been
      % received yet.
      u = zeros(obj.num_u,1);
    end
  end
    
  properties (SetAccess=private, GetAccess = public)
    num_x;  % dimension of x
    num_u;  % dimension of u
    umin=-inf;   % constrains u>=umin (default umin=-inf)
    umax=inf;    % constrains u<=uman (default umax=inf)
  end
  
  properties
    ode_solver=@ode45;  % default is @ode45
    ode_options=odeset; % use odeset to change this
  end
  
end