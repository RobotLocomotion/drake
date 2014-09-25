classdef PolynomialTrajectorySystem < PolynomialSystem
% dynamics, update, output are polynomial in x and u, but not necessarily
% polynomial in t.

% todo: replace this with a class for polynomials with Time-Varying
% Coefficients (from a trajectory).  get rid of the PolynomialTrajectory
% class completely.  see Bug 1006.
  
  properties (SetAccess=private)
    p_x=[]
    p_u=[]
    p_dynamics_traj=[] % msspoly representations of dynamics, update,and output
    p_update_traj=[]
    p_output_traj=[]
  end
    
  methods
    function obj = PolynomialTrajectorySystem(input_frame,state_frame,output_frame,p_dynamics_traj,p_update_traj,p_output_traj,direct_feedthrough_flag)
      obj = obj@PolynomialSystem(size(p_dynamics_traj,1),size(p_update_traj,1),input_frame.dim,size(p_output_traj,1),direct_feedthrough_flag,false,false);

      if (state_frame.dim~=obj.getNumStates())
        error('state dimension mismatch');
      end
      if (output_frame.dim~=obj.getNumOutputs())
        error('output dimension mismatch');
      end
      obj = setInputFrame(obj,input_frame);
      obj = setStateFrame(obj,state_frame);
      obj = setOutputFrame(obj,output_frame);

      p_u=obj.getInputFrame.getPoly;
      p_x=obj.getStateFrame.getPoly;

      if (~isempty(p_dynamics_traj))
        typecheck(p_dynamics_traj,'PolynomialTrajectory');
        dyn = p_dynamics_traj.eval(p_dynamics_traj.tspan(1));
        sizecheck(dyn,[obj.getNumContStates,1]);
        
        if any(match([p_x;p_u],decomp(dyn))==0)
          error('p_dynamics_traj depends on variables other than x, and u (from the current state and input frames)');
        end
        
        obj.p_dynamics_traj=p_dynamics_traj;
      end
      
      if (~isempty(p_update_traj))
        typecheck(p_update_traj,'PolynomialTrajectory'); 
        up = p_update_traj.eval(p_update_traj.tspan(1));
        sizecheck(up,[obj.getNumDiscStates,1]);
        
        if any(match([p_x;p_u],decomp(up))==0)
          error('p_update_traj depends on variables other than x, and u (from the current state and input frames)');
        end
        
        obj.p_update_traj = p_update_traj;
      end
      
      if (~isempty(p_output_traj))
        typecheck(p_output_traj,'PolynomialTrajectory'); 
        out = p_output_traj.eval(p_output_traj.tspan(1));
        sizecheck(out,[obj.getNumOutputs,1]);

        if any(match([p_x;p_u],decomp(out))==0)
          error('p_output_traj depends on variables other than x, and u (from the current state and input frames)');
        end
        
        obj.p_output_traj=p_output_traj;
      end
      
        
      
    end
    
    % Implement default methods using msspoly vars explicitly
    function xcdot = dynamicsRHS(obj,t,x,u)
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
    
    function [p_dynamics_rhs,p_dynamics_lhs] = getPolyDynamics(obj,t)
      p_dynamics_lhs=[];
      if (isempty(obj.p_dynamics_traj)) p_dynamics_rhs=[];
      else
        p_dynamics_rhs = obj.p_dynamics_traj.eval(t);
      end        
    end
    
    function p_update = getPolyUpdate(obj,t)
      if isempty(obj.p_update_traj) p_update = [];
      else
        p_update = obj.p_update_traj.eval(t);
      end
    end
    
    function p_output = getPolyOutput(obj,t)
      if isempty(obj.p_output_traj) p_output = [];
      else
        p_output = obj.p_output_traj.eval(t);
      end
    end
  end
end
