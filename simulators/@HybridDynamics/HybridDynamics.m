classdef HybridDynamics < Dynamics
% A class that contains mode Dynamics and mode switches.
%   Contains multiple dynamics objects, and rules for transition events and
%   transition dynamics between them.  The state space of the hybrid
%   dynamics has x(1) as the (discrete valued) mode, and x(2:end) as the
%   state of the dynamics in that mode.  Note that this means the dimension
%   of the state vector may not be constant.
  
  properties (Access=public)
    mode_dynamics={};              % cell array - instances of the Dynamics class
  end
  
  methods (Abstract=true)
    phi = collision_detect(obj,t,x,m)
    [xn,mn,status] = collision_dynamics(obj,t,x,m,u)
  end

  
  methods
    function obj = HybridDynamics()
      obj.numStates = NaN;  %since these are poorly defined.
      obj.numInputs = NaN;
    end

    function obj = addDynamics(obj,dynamics)
      if (~iscell(dynamics)) dynamics = {dynamics}; end

      for i=1:length(dynamics)
        typecheck(dynamics{i},'Dynamics'); 
        if (~isempty(odeget(dynamics{i}.odeoptions,'Events')))
          warning('Event property of ODE options will be over-written'); 
        end
        % set event detection function on each subdynamics:
        dynamics{i}.odeoptions = odeset(dynamics{i}.odeoptions,'Events',@(t,x)collision_event(obj,t,x));
      end
      
      obj.mode_dynamics = {obj.mode_dynamics{:},dynamics{:}};
    end
    
    function x = getInitialState(obj)
      mode = 1;
      x = [mode; obj.mode_dynamics{mode}.getInitialState()];
    end
    
    function xdot = dynamics(obj,t,x,u)
      if (mod(x(1),1)~=0) error('x(1) is the mode and should be a positive integer.  Something is wrong.'); end
      if (x(1)<1 || x(1)>length(obj.mode_dynamics)) error('x(1) references an invalid mode.'); end
      
      xdot = [0; obj.mode_dynamics{x(1)}.dynamics(t,x(2:end),u)];
    end
    
    function df = dynamics_gradients(obj,t,x,u,order)
      if (mod(x(1),1)~=0) error('x(1) is the mode and should be a positive integer.  Something is wrong.'); end
      if (x(1)<1 || x(1)>length(obj.mode_dynamics)) error('x(1) references an invalid mode.'); end

      error('not implemented yet'); 
    end      
    
    function xtraj = simulate(obj,control,tspan,x0)
      if (nargin<4) x0 = getInitialState(obj); end
      
      m = x0(1);
      if (obj.mode_dynamics{m}.numInputs>0)
        sol = obj.mode_dynamics{m}.odesolver(@(t,x)closed_loop_dynamics(t,x),tspan,x0,obj.mode_dynamics{m}.odeoptions);
      else
        sol = obj.mode_dynamics{m}.odesolver(@(t,x)obj.dynamics(t,x,[]),tspan,x0,obj.mode_dynamics{m}.odeoptions);
      end
      xtraj{1} = ODESolTrajectory(sol);
      status=0;
      
      while (~status && ~isempty(sol.xe))
        te = sol.xe(end);  xe = sol.ye(:,end);  % annoying, but true
        tspan = [te, tspan(tspan>te)];
        if( obj.mode_dynamics{m}.numInputs>0)
          ue = control.control(te,xe);
          ue = min(max(ue,obj.mode_dynamics{m}.umin),obj.mode_dynamics{m}.umax);
        else
          ue=[];
        end
        [xn,m,status] = obj.collision_dynamics(te,xe(2:end),xe(1),ue);
        x = [m;xn];
        
        if (obj.mode_dynamics{m}.numInputs>0)
          sol = obj.mode_dynamics{m}.odesolver(@(t,x)closed_loop_dynamics(t,x),tspan,x,obj.mode_dynamics{m}.odeoptions);
        else
          sol = obj.mode_dynamics{m}.odesolver(@(t,x)obj.dynamics(t,x,[]),tspan,x,obj.mode_dynamics{m}.odeoptions);
        end
        xtraj = {xtraj{:},ODESolTrajectory(sol)};
      end
      
      xtraj = HybridTrajectory(xtraj);
      
      function xdot = closed_loop_dynamics(t,x)
        u = control.control(t,x(2:end));
        u = min(max(u,obj.mode_dynamics{m}.umin),obj.mode_dynamics{m}.umax);
        xdot = obj.dynamics(t,x,u);
      end
      
    end

    function dphi = collision_detect_gradients(obj,t,x,m)
      error('not implemented yet');
    end
    
    function dxn = collision_dynamics_gradients(obj,t,x,m,u)
      error('not implemented yet');
    end
    
    % wrap the collision detection in a form compatible with the ODE event handling
    function [phi,isterminal,direction] = collision_event(obj,t,x)
      phi = obj.collision_detect(t,x(2:end),x(1));
      isterminal = 1;
      direction = 0;
    end    
    
  end
  
end
