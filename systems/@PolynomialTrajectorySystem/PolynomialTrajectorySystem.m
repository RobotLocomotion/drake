classdef PolynomialTrajectorySystem < SmoothRobotLibSystem

  properties (SetAccess=private)
    p_x=[]
    p_u=[]
    p_dynamics_traj=[] % msspoly representations of dynamics, update,and output
    p_update_traj=[]
    p_output_traj=[]
  end
    
  methods
    function obj = PolynomialTrajectorySystem(p_dynamics_traj,p_update_traj,p_output_traj,num_u,direct_feedthrough_flag)
      obj = obj@SmoothRobotLibSystem(0,0,0,0,direct_feedthrough_flag,false);

      checkDependency('spot_enabled');

      if (nargin>0 && ~isempty(p_dynamics_traj))
        typecheck(p_dynamics_traj,'PolynomialTrajectory');
        dyn = p_dynamics_traj.eval(p_dynamics_traj.tspan(1));
        num_xc = dyn.m;
        if (dyn.n ~= 1) error('dynamics should be a column vector msspoly'); end 
        obj.p_dynamics_traj=p_dynamics_traj;
      else
        num_xc = 0;
      end
      
      if (nargin>1 && ~isempty(p_update_traj))
        typecheck(p_update_traj,'PolynomialTrajectory'); 
        up = p_update_traj.eval(p_update_traj.tspan(1));
        num_xd = up.m;
        if (up.n ~= 1) error('update should be a column vector msspoly'); end
        obj.p_update_traj = p_update_traj;
      else
        num_xd = 0;
      end
      
      if (nargin>2 && ~isempty(p_output_traj))
        typecheck(p_output_traj,'PolynomialTrajectory'); 
        out = p_output_traj.eval(p_output_traj.tspan(1));
        num_y = out.m;
        if (out.n ~= 1) error('output should be a column vector msspoly'); end
        obj.p_output_traj=p_output_traj;
      else
        num_y = 0;
      end
      
      if (nargin<4)
        num_u = 0;
      end
      
      if (num_xc+num_xd>0)
        obj.p_x=msspoly('x',num_xc+num_xd);
        obj = setNumContStates(obj,num_xc);
        obj = setNumDiscStates(obj,num_xd);
      end
      if (num_u>0)
        obj.p_u=msspoly('u',num_u);
        obj = setNumInputs(obj,num_u);
      end

      obj = setNumOutputs(obj,num_y);
      
    end
    
    % Implement default methods using msspoly vars explicitly
    function xcdot = dynamics(obj,t,x,u)
      % should only get here if constructor specified p_dynamics
      %
      % @param t time to evaluate dyanmics at
      % @param x state to evaluate dynmaics at (as a column vector)
      % @param u control action to evaluate dyanmics at (as a column
      % vector)
      %
      % @retval xcdot approximated dynamics
      
      if (isempty(obj.p_dynamics_traj)) error('dynamics is not defined.  how did you get here?'); end
      xcdot = double(subs(obj.p_dynamics_traj.eval(t),[obj.p_x;obj.p_u],[x;u]));
    end
    
    function xdn = update(obj,t,x,u)
      if (isempty(obj.p_update_traj)) error('update is not defined.  how did you get here?'); end
      xdn = double(subs(obj.p_update_traj.eval(t),[obj.p_x;obj.p_u],[x;u]));
    end
    function y = output(obj,t,x,u)
      if (isempty(obj.p_output_traj)) error('output is not defined.  how did you get here?'); end
      y = double(subs(obj.p_output_traj.eval(t),[obj.p_x;obj.p_u],[x;u]));
    end
    
    % todo: implement gradients (it's trivial)
  end
    
end
