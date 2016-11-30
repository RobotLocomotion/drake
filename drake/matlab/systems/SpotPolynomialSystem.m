classdef SpotPolynomialSystem < PolynomialSystem
  
  properties (SetAccess=private, GetAccess=private)
    p_dynamics_rhs % msspoly representations of dynamics, update,and output
    p_dynamics_lhs
    p_update
    p_output
    p_state_constraints
    p_t
    p_state_constraint_id
  end
  
  methods
    function obj = SpotPolynomialSystem(input_frame,state_frame,output_frame,p_dynamics_rhs,p_dynamics_lhs,p_update,p_output,p_state_constraints)
      obj = obj@PolynomialSystem(size(p_dynamics_rhs,1),size(p_update,1),input_frame.dim,size(p_output,1),false,true,false);

      if isempty(state_frame) state_frame = CoordinateFrame('state',0); end

      if (state_frame.dim~=obj.getNumStates())
        error('state dimension mismatch');
      end
      if (output_frame.dim~=obj.getNumOutputs())
        error('output dimension mismatch');
      end
      obj.p_t = msspoly('t',1);
      obj = setInputFrame(obj,input_frame);
      obj = setStateFrame(obj,state_frame);
      obj = setOutputFrame(obj,output_frame);
      obj = setPolyDynamics(obj,p_dynamics_rhs,p_dynamics_lhs);
      obj = setPolyUpdate(obj,p_update);
      obj = setPolyOutput(obj,p_output);
      if (nargin>7 && ~isempty(p_state_constraints))
        obj = setPolyStateConstraints(obj,p_state_constraints);
      end
    end
    
    % Implement default methods using msspoly vars explicitly
    function [f,df] = dynamicsRHS(obj,t,x,u)
      p_t=obj.p_t;
      p_x=obj.getStateFrame.getPoly; 
      p_u=obj.getInputFrame.getPoly; 
      f = double(subs(obj.p_dynamics_rhs,[p_t;p_x;p_u],[t;x;u]));
      if (nargout>1)
        df = double(subs(diff(obj.p_dynamics_rhs,[p_t;p_x;p_u]),[p_t;p_x;p_u],[t;x;u]));
      end
    end
    
    function [e,de] = dynamicsLHS(obj,t,x,u)
      p_t=obj.p_t;
      p_x=obj.getStateFrame.getPoly; 
      p_u=obj.getInputFrame.getPoly; 
      e = double(subs(obj.p_dynamics_lhs,[p_t;p_x;p_u],[t;x;u]));
      if (nargout>1)
        de = double(subs(diff(obj.p_dynamics_lhs,[p_t;p_x;p_u]),[p_t;p_x;p_u],[t;x;u]));
      end
    end
    
    function [xdn,df] = update(obj,t,x,u)
      p_t=obj.p_t;
      p_x=obj.getStateFrame.getPoly;
      p_u=obj.getInputFrame.getPoly;
      xdn = double(subs(obj.p_update,[p_t;p_x;p_u],[t;x;u]));
      if (nargout>1)
        df = double(subs(diff(obj.p_update,[p_t;p_x;p_u]),[p_t;p_x;p_u],[t;x;u]));
      end
    end
    
    function [y,dy] = output(obj,t,x,u)
      p_t=obj.p_t;
      p_x=obj.getStateFrame.getPoly;
      if (nargin<4 || obj.num_u<1) p_u=[]; u=[]; else p_u=obj.getInputFrame.getPoly; end  % ok for systems without direct feedthrough
      y = double(subs(obj.p_output,[p_t;p_x;p_u],[t;x;u]));
      if (nargout>1)
        dy = double(subs(diff(obj.p_output,[p_t;p_x;p_u]),[p_t;p_x;p_u],[t;x;u]));
      end
    end    
    
    function obj = setInputFrame(obj,frame)
      if frame.dim>0
        if obj.getNumStates()>0 && any(match(obj.getStateFrame.getPoly,frame.getPoly)~=0)
          error('input frame poly clashes with current state frame poly.  this could lead to massive confusion');
        end

        if ~isempty(obj.p_dynamics_rhs)
          obj.p_dynamics_rhs = subs(obj.p_dynamics_rhs,obj.getInputFrame.getPoly,frame.getPoly);
        end
        if ~isempty(obj.p_dynamics_lhs)
          obj.p_dynamics_lhs = subs(obj.p_dynamics_lhs,obj.getInputFrame.getPoly,frame.getPoly);
        end
        if ~isempty(obj.p_update)
          obj.p_update = subs(obj.p_update,obj.getInputFrame.getPoly,frame.getPoly);
        end
        if ~isempty(obj.p_output) && obj.isDirectFeedthrough()
          obj.p_output = subs(obj.p_output,obj.getInputFrame.getPoly,frame.getPoly);
        end
      end
      
      obj = setInputFrame@DrakeSystem(obj,frame);
    end
    
    function obj = setStateFrame(obj,frame)
      if frame.dim>0
        if obj.getNumInputs()>0 && any(match(obj.getInputFrame.getPoly,frame.getPoly)~=0)
          error('state frame poly clashes with current input frame poly.  this could lead to massive confusion');
        end
        
        if ~isempty(obj.p_dynamics_rhs)
          obj.p_dynamics_rhs = subs(obj.p_dynamics_rhs,obj.getStateFrame.getPoly,frame.getPoly);
        end
        if ~isempty(obj.p_dynamics_lhs)
          obj.p_dynamics_lhs = subs(obj.p_dynamics_lhs,obj.getStateFrame.getPoly,frame.getPoly);
        end
        if ~isempty(obj.p_update)
          obj.p_update = subs(obj.p_update,obj.getStateFrame.getPoly,frame.getPoly);
        end
        if ~isempty(obj.p_output)
          obj.p_output = subs(obj.p_output,obj.getStateFrame.getPoly,frame.getPoly);
        end
        if ~isempty(obj.p_state_constraints)
          obj.p_state_constraints = subs(obj.p_state_constraints,obj.getStateFrame.getPoly,frame.getPoly);
        end
      end
      
      obj = setStateFrame@DrakeSystem(obj,frame);
    end
    
    function obj = setPolyDynamics(obj,p_dynamics_rhs,p_dynamics_lhs)
      if ~isempty(p_dynamics_rhs)
        typecheck(p_dynamics_rhs,'msspoly');
        if ~iscolumn(p_dynamics_rhs) error('p_dynamics_rhs must be a column vector'); end
        obj = setNumContStates(obj,size(p_dynamics_rhs,1));
        p_t=obj.p_t;
        p_x=obj.getStateFrame.getPoly;
        p_u=obj.getInputFrame.getPoly;
        
        if any(match([p_t;p_x;p_u],decomp(p_dynamics_rhs))==0)
          error('p_dynamics_rhs depends on variables other than t,x, and u (from the current state and input frames)');
        end
      end
      
      if ~isempty(p_dynamics_lhs)
        typecheck(p_dynamics_lhs,'msspoly');
        sizecheck(p_dynamics_lhs,[obj.num_xc,obj.num_xc]);
        if ~isempty(decomp(p_dynamics_lhs)) && any(match([p_t;p_x;p_u],decomp(p_dynamics_lhs))==0)
          error('p_dynamics_lhs depends on variables other than t,x, and u (from the current state and input frames)');
        end
      end
      obj = setRationalFlag(obj,~isempty(p_dynamics_lhs));
      obj.p_dynamics_rhs = p_dynamics_rhs;
      obj.p_dynamics_lhs = p_dynamics_lhs;
    end
    
    function [p_dynamics_rhs,p_dynamics_lhs] = getPolyDynamics(obj,t) 
      p_dynamics_rhs = obj.p_dynamics_rhs;
      p_dynamics_lhs = obj.p_dynamics_lhs;
      if (nargin>1 && obj.num_xc>0)
        p_dynamics_rhs = subs(p_dynamics_rhs,obj.p_t,t);
        if (isRational(obj))
          p_dynamics_lhs = subs(p_dynamics_lhs,obj.p_t,t);
        end
      end
    end
    
    function obj = setPolyUpdate(obj,p_update)
      if ~isempty(p_update)
        typecheck(p_update,'msspoly');
        if ~iscolumn(p_update) error('p_update must be a column vector'); end
        obj = setNumDiscStates(obj,size(p_update,1));
        p_t=obj.p_t;
        p_x=obj.getStateFrame.getPoly;
        p_u=obj.getInputFrame.getPoly;
        
        if any(match([p_t;p_x;p_u],decomp(p_update))==0)
          error('p_update depends on variables other than t,x, and u (from the current state and input frames)');
        end
      end
      obj.p_update = p_update;
    end
    
    function p_update = getPolyUpdate(obj,t)
      p_update = obj.p_update;
      if (nargin>1 && obj.num_xd>0)
        p_update = subs(p_update,obj.p_t,t);
      end
    end
    
    function obj = setPolyOutput(obj,p_output)
      if ~isempty(p_output)
        typecheck(p_output,'msspoly');
        if ~iscolumn(p_output) error('p_output must be a column vector'); end
        obj = setNumOutputs(obj,size(p_output,1));
        p_t=obj.p_t;
        p_x=obj.getStateFrame.getPoly;
        p_u=obj.getInputFrame.getPoly;
        
        if any(match([p_t;p_x;p_u],decomp(p_output))==0)
          error('p_output depends on variables other than t,x, and u (from the current state and input frames)');
        end
      end
      obj.p_output = p_output;      
    end
    
    function p_output = getPolyOutput(obj,t)
      p_output = obj.p_output;
      if (nargin>1 && obj.num_y>0)
        p_output = subs(p_output,obj.p_t,t);
      end
    end
    
    function obj = setPolyStateConstraints(obj,p_state_constraints)
      if ~isempty(p_state_constraints)
        typecheck(p_state_constraints,'msspoly');
        if ~iscolumn(p_state_constraints) error('p_state_constraints must be a column vector'); end
        if ~isempty(decomp(p_state_constraints)) && any(match(obj.getStateFrame.getPoly,decomp(p_state_constraints))==0)
          error('p_state_constraints depends on variables other than x (the current state frame)');
        end
        con = SpotPolynomialConstraint(zeros(size(p_state_constraints)),zeros(size(p_state_constraints)),obj.getStateFrame.getPoly,p_state_constraints);
        if isempty(obj.p_state_constraint_id)
          [obj,obj.p_state_constraint_id] = addStateConstraint(obj,con);
        else
          obj = updateStateConstraint(obj,obj.p_constraint_id,con);
        end
      else
        if ~isempty(obj.p_state_constraint_id)
          obj = updateStateConstraint(obj,obj.p_constraint_id,NullConstraint(getNumStates(obj)));
        end
      end
      obj.p_state_constraints = p_state_constraints;
    end
    
    function p_state_constraints = getPolyStateConstraints(obj)
      p_state_constraints = obj.p_state_constraints;
    end
    
    function sys = extractAffineSystem(obj)
      if ~isTI(obj)
        error('time-varying case not implemented yet');
      end
      
      t=0;
      p_x=obj.getStateFrame.getPoly;
      p_u=obj.getInputFrame.getPoly;
      
      Ac = []; Bc = []; xcdot0=[];
      if (obj.num_xc>0)
        if deg(obj.p_dynamics_rhs,[p_x;p_u])>1
          error('Drake:SpotPolynomialSystem:NotAffine','RHS has deg>1');
        end
        Ac = double(diff(obj.p_dynamics_rhs,p_x));
        Bc = double(diff(obj.p_dynamics_rhs,p_u));
        xcdot0 = double(subs(obj.p_dynamics_rhs,[p_x;p_u],zeros(obj.num_x+obj.num_u,1)));
        if (isRational(obj))
          if deg(obj.p_dynamics_lhs,[p_x;p_u])>0
            error('Drake:SpotPolynomialSystem:NotAffine','LHS has deg>0');
          end
          E = double(obj.p_dynamics_lhs);
          Ac = E\Ac;
          Bc = E\Bc;
          xcdot0 = E\xcdot0;
        end
      end
      
      Ad = []; Bd = [];
      if (obj.num_xd>0)
        if deg(obj.p_update,[p_x;p_u])>1
          error('Drake:SpotPolynomialSystem:NotAffine','update has deg>1');
        end
        Ad = double(diff(obj.p_update,p_x));
        Bd = double(diff(obj.p_update,p_u));
        xdn0 = double(subs(obj.p_update,[p_x;p_u],zeros(obj.num_x+obj.num_u,1)));
      end
      
      C = []; D = []; y0 = [];
      if (obj.num_y>0)
        if deg(obj.p_output,[p_x;p_u])>1
          error('Drake:SpotPolynomialSystem:NotAffine','output has deg>1');
        end
        C = double(diff(obj.p_output,p_x));
        D = double(diff(obj.p_output,p_u));
        y0 = double(subs(obj.p_output,[p_x;p_u],zeros(obj.num_x+obj.num_u,1)));
      end
      
      sys = AffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0);
      
      sys = setInputFrame(sys,obj.getInputFrame());
      sys = setStateFrame(sys,obj.getStateFrame());
      sys = setOutputFrame(sys,obj.getOutputFrame());
      
      sys = setSampleTime(sys,obj.getSampleTime);
    end
  end
  
end
