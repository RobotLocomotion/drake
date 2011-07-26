classdef PolynomialSystem < SmoothRobotLibSystem

  properties (SetAccess=private)
    p_t
    p_x
    p_u
    p_dynamics % msspoly representations of dynamics, update,and output
    p_update
    p_output
  end

  methods
%    function x0=getInitialState(obj)
%      x0=zeros(getNumStates(obj),1);
%    end
    
    function obj = PolynomialSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag,p_dynamics,p_update,p_output)
      obj = obj@SmoothRobotLibSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag);

      checkDependency('spot_enabled');
      
      % Now create the msspoly versions of the dynamics,update,and output:
      obj.p_t=msspoly('t',1);
      if (num_xc+num_xd>0)
        obj.p_x=msspoly('x',num_xc+num_xd);
      end
      if (num_u>0)
        obj.p_u=msspoly('u',num_u);
      end
      
      % these will error if the system is not polynomial (should I catch
      % and rethrow the error with more information?)
      if (nargin>6 && num_xc>0)
        typecheck(p_dynamics,'msspoly');
        if any([p_dynamics.m,p_dynamics.n] ~= [num_xc,1]) error('p_dynamics does not match num_xc'); end 
        obj.p_dynamics=p_dynamics;
      elseif (num_xc>0)
        obj.p_dynamics=obj.dynamics(obj.p_t,obj.p_x,obj.p_u);
      end
      
      if (nargin>7 && num_xd>0)
        typecheck(p_update,'msspoly');
        if any([p_update.m,p_update.n] ~= [num_xd,1]) error('p_update does not match num_xd'); end
        obj.p_update = p_update;
      elseif (num_xd>0)
        obj.p_update=obj.update(obj.p_t,obj.p_x,obj.p_u);
      end
      
      if (nargin>8 && num_y>0)
        typecheck(p_output,'msspoly');
        if any([p_output.m,p_output.n] ~= [num_y,1]) error('p_output does not match num_y'); end
        obj.p_output=p_output;
      elseif (num_y>0)
        obj.p_output=obj.output(obj.p_t,obj.p_x,obj.p_u);
      end
    end
    
    % Implement default methods using msspoly vars explicitly
    function xcdot = dynamics(obj,t,x,u)
      % should only get here if constructor specified p_dynamics
      if (isempty(obj.p_dynamics)) error('p_dynamics is not defined.  how did you get here?'); end
      xcdot = double(subs(obj.p_dynamics,[obj.p_t;obj.p_x;obj.p_u],[t;x;u]));
    end
    function xdn = update(obj,t,x,u)
      if (isempty(obj.p_update)) error('p_update is not defined.  how did you get here?'); end
      xdn = double(subs(obj.p_update,[obj.p_t;obj.p_x;obj.p_u],[t;x;u]));
    end
    function y = output(obj,t,x,u)
      if (isempty(obj.p_output)) error('p_output is not defined.  how did you get here?'); end
      y = double(subs(obj.p_output,[obj.p_t;obj.p_x;obj.p_u],[t;x;u]));
    end
    
    % todo: implement gradients (it's trivial)
  end
  
  methods  % for constructing and manipulating polynomial systems
    function polysys = timeReverse(obj)
      if (obj.num_xd>0) error('only for CT systems'); end
      polysys = PolynomialSystem(obj.num_xc,obj.num_xd,obj.num_u,obj.num_u,obj.direct_feedthrough_flag,obj.time_invariant_flag,...
        -obj.p_dynamics,[],obj.p_output);
    end
  end
end
