classdef TimeVaryingPolynomialSystem < DrakeSystem
% dynamics, update, output are polynomial in x and u, but not necessarily
% polynomial in t.
  
  properties (SetAccess=private,GetAccess=private)
    p_dynamics_traj=[] % msspoly representations of dynamics, update,and output
    p_update_traj=[]
    p_output_traj=[]

%   todo: implement these   
%    p_state_constraints_traj=[];
%    p_mass_matrix_traj=[]; % msspoly representation for e(x), as in e(x)xdot=f(x).  if empty then the default is e(x)=I.  must be invertible for all x.
  end
    
  methods
    function obj = TimeVaryingPolynomialSystem(varargin)
      % Method 1:
      %   TimeVaryingPolynomialSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag)
      %   and implement the polydynamics, polyupdate, and polyoutput
      %   methods
      %   
      % Method 2: 
      %   TimeVaryingPolynomialSystem(p_dynamics_traj, p_update_traj,num_u, p_output_traj,direct_feedthrough_flag)
      %   where the _traj variables are PolynomialTrajectories (or the
      %   empty matrix)

      obj = obj@DrakeSystem(0,0,0,0,true,false);

      checkDependency('spot_enabled');

      if nargin<5, error('must supply 5 arguments'); end
      
      if (isempty(varargin{1})
        % then do nothing
      elseif (isnumeric(varargin{1}))
        % then it's just num_xc
        obj = setNumConstStates(obj,varargin{1});
        if (varargin{1}>0)  % run a quick sanity check
          typecheck(polydynamics(obj,0),'msspoly');
          sizecheck(polydynamics(obj,0),[getNumContStates(obj),1]);
        end
      else
        % must be a polynomialTrajectory
        p_dynamics_traj = varargin{1};
        typecheck(p_dynamics_traj,'PolynomialTrajectory');
        if ~iscolumn(p_dynamics_traj), error('dynamics must be a column vector'); end
        obj = setNumContStates(obj,size(p_dynamics_traj,1));
        obj.p_dynamics_traj=p_dynamics_traj;
      end
      
      if (isempty(varargin{2}))
        % then do nothing
      elseif isnumeric(varargin{2})
        obj = setNumDiscStates(obj,varargin{2});
        if (varargin{2}>0)  % run a quick sanity check
          typecheck(polyupdate(obj,0),'msspoly');
          sizecheck(polyupdate(obj,0),[getNumDiscStates(obj),1]);
        end
      else
        p_update_traj = varargin{2};
        typecheck(p_update_traj,'PolynomialTrajectory');
        if ~iscolumn(p_update_traj), error('update must be a column vector'); end
        obj = setNumDiscStates(obj,size(p_update_traj,1));
        obj.p_update_traj = p_update_traj;
      end
      
      if ~isnumeric(varargin{3}) || isempty(varargin{3}), error('argument 3 must be the number of inputs'); end
      obj = setNumInputs(obj,varargin{3});
      
      if (isempty(varargin{4})
        % then do nothing
      elseif isnumeric(varargin{4})
        obj = setNumOutputs(obj,varargin{4});
        if (varargin{3}>0)  % run a quick sanity check
          typecheck(polyoutput(obj,0),'msspoly');
          sizecheck(polyoutput(obj,0),[getNumOutputs(obj),1]);
        end
      else
        p_output_traj = varargin{4};
        typecheck(p_output_traj,'PolynomialTrajectory');
        if ~iscolumn(p_output_traj), error('output must be a column vector'); end
        obj = setNumOutput(obj,size(p_output_traj,1));
        obj.p_output_traj=p_output_traj;
      end
      
      obj = setDirectFeedthroughFlag(obj,varargin{5});
      if ~isDirectFeedthrough(obj) && ~isempty(obj.p_output_traj)
        % sanity check, make sure there is no input dependence at tspan(1)
        if (deg(obj.p_output_traj.eval(obj.p_output_traj.tspan(1)),obj.getInputFrame.poly)>0)
          error('found dependence of the output on u in a system that claimed to be not direct feedthrough');
        end
      end
    end
    
    function xcdot = dynamics(obj,t,x,u)
      xcdot = double(subs(polydynamics(obj,t),[obj.getStateFrame.poly;obj.getInputFrame.poly],[x;u]));
    end
    
    function xdn = update(obj,t,x,u)
      xdn = double(subs(polyupdate(obj,t),[obj.getStateFrame.poly;obj.getInputFrame.poly],[x;u]));
    end
    
    function y = output(obj,t,x,u)
      y = double(subs(polyoutput(obj,t),[obj.getStateFrame.poly;obj.getInputFrame.poly],[x;u]));
    end
    
    % todo: implement gradients (it's trivial)
  end
  
  methods  % overload these methods in subclasses (instead of dynamics, update, output)
    function p_dynamics = polydynamics(obj,t)
      if (isempty(obj.p_dynamics_traj)) error('dynamics is not defined.  how did you get here?'); end
      p_dynamics = obj.p_dynamics_traj.eval(t);
    end

    function p_update = polyupdate(obj,t)
      if (isempty(obj.p_update_traj)) error('update is not defined.  how did you get here?'); end
      p_update = obj.p_update_traj.eval(t);
    end

    function p_output = polyoutput(obj,t)
      if (isempty(obj.p_output_traj)) error('output is not defined.  how did you get here?'); end
      p_output = obj.p_output_traj.eval(t);
    end
  end
    
  
  % todo: implement feedback and cascade
end
