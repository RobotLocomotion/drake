classdef PolynomialSystem < DrakeSystem 
  % A dynamical system described by rational polynomial dynamics, and
  % polynomial outputs.  If the system is time-varying, then it 
  % need not be polynomial in time (but for every time, it is polynomial in x and u). 

  properties (SetAccess=private,GetAccess=private)
    rational_flag=false;
  end
  
  methods
    function obj = PolynomialSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag,rational_flag)
      % initialize PolynomialSystem
      checkDependency('spot_enabled');
      obj = obj@DrakeSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag);
      obj.rational_flag = rational_flag;
    end
  end
  
  methods (Sealed=true)
    function xcdot = dynamics(obj,t,x,u)
      xcdot = dynamicsRHS(obj,t,x,u);
      if (obj.rational_flag)
        lhs = dynamicsLHS(obj,t,x,u);
        xcdot = lhs \ xcdot;
      end
      % todo: add gradient info back in
          % e(x)xdot = f(x) => dexdotdx + e(x)*dxdotdx = dfdx
          %  where the columns of dexdotdx are dedxi*xdot
%          nX = obj.num_xc; cellxcdot=cell(1,nX); [cellxcdot{:}]=deal(xcdot); 
%          dexdotdx = reshape(double(subs(diff(obj.p_mass_matrix(:),p_x),p_x,x)),nX,[])*blkdiag(cellxcdot{:}); 
%          df = e\(df - [zeros(nX,1),dexdotdx,zeros(nX,obj.num_u)]);
    end
  end    

  
  
  methods
    function lhs = dynamicsLHS(obj,t,x,u)
      error('rational polynomial systems with continuous state must implement this');
    end

    function rhs = dynamicsRHS(obj,t,x,u)
      error('polynomial systems with continuous state must implement this');
    end

    function obj = setRationalFlag(obj,tf)
      obj.rational_flag = tf;
    end
    
    function tf = isRational(obj)
      tf=obj.rational_flag;
    end
    
    function [p_dynamics_rhs,p_dynamics_lhs] = getPolyDynamics(obj,t)
      p_dynamics_rhs = [];
      p_dynamics_lhs = [];
      if (obj.num_xc>0)
        if (nargin<2 && ~isTI(obj)) error('you must specify a time'); else t=0; end
        p_x=obj.getStateFrame.poly;
        if (obj.num_u>0) p_u=obj.getInputFrame.poly; else p_u=[]; end
      
        p_dynamics_rhs=obj.dynamicsRHS(t,p_x,p_u);
        if (nargout>1 && obj.rational_flag)
          p_dynamics_lhs=obj.dynamicsLHS(t,p_x,p_u);
        end
      end
    end
    function p_update = getPolyUpdate(obj,t)
      if (obj.num_xd>0)
        if (nargin<2 && ~isTI(obj)) error('you must specify a time'); else t=0; end
        p_x=obj.getStateFrame.poly;
        if (obj.num_u>0) p_u=obj.getInputFrame.poly; else p_u=[]; end

        p_update=obj.update(t,p_x,p_u);
      else
        p_update=[];
      end
    end
    function p_output = getPolyOutput(obj,t)
      if (obj.num_y>0)
        if (nargin<2 && ~isTI(obj)) error('you must specify a time'); else t=0; end
        if (obj.num_x>0) p_x=obj.getStateFrame.poly; else p_x=[]; end
        if (obj.num_u>0) p_u=obj.getInputFrame.poly; else p_u=[]; end
        
        p_output=obj.output(t,p_x,p_u);
      else
        p_output=[];
      end
    end
    function p_state_constraints = getPolyStateConstraints(obj)
      if (obj.num_xcon>0)
        p_x=obj.getStateFrame.poly;
        obj.p_state_constraints=obj.stateConstraints(p_x);
      else
        p_state_constraints = [];
      end
    end
    
    function obj = setNumZeroCrossings(obj,num_zcs)
      if (num_zcs)
        error('PolynomialSystems are not allowed to have zero crossings');
      end
    end
  end
  
  methods  % for constructing and manipulating polynomial systems
    function polysys = timeReverse(obj)
      if (obj.num_xd>0) error('only for CT systems'); end
      if (~isTI(obj)) error('only for time invariant systems'); end
      [p_dynamics_rhs,p_dynamics_lhs]=getPolyDynamics(obj);
      p_output=getPolyOutput(obj);
      p_state_constraints=getPolyStateConstraints(obj);
      polysys = SpotPolynomialSystem(obj.getInputFrame,obj.getStateFrame,obj.getOutputFrame,...
        -p_dynamics_rhs,p_dynamics_lhs,[],p_output,p_state_constraints);
    end
    
    function obj = setInputFrame(obj,frame)
      if frame.dim>0
        if obj.getNumStates()>0 && any(match(obj.getStateFrame.poly,frame.poly)~=0)
          error('input frame poly clashes with current state frame poly.  this could lead to massive confusion');
        end
      end
      
      obj = setInputFrame@DrakeSystem(obj,frame);
    end
    
    function obj = setStateFrame(obj,frame)
      if frame.dim>0
        if obj.getNumInputs()>0 && any(match(obj.getInputFrame.poly,frame.poly)~=0)
          error('state frame poly clashes with current input frame poly.  this could lead to massive confusion');
        end
      end
      
      obj = setStateFrame@DrakeSystem(obj,frame);
    end
    
    function sys = feedback(sys1,sys2)
      % try to keep feedback between polynomial systems polynomial.  else,
      % kick out to DrakeSystem
      %
      
      [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2,false);
      [sys2,sys1] = matchCoordinateFramesForCombination(sys2,sys1,true);

      if ~isa(sys2,'PolynomialSystem') || ~isTI(sys1) || ~isTI(sys2) || any(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax]))
        sys = feedback@DrakeSystem(sys1,sys2);
        return;
      end
        
      if (sys1.isDirectFeedthrough() && sys2.isDirectFeedthrough())
        error('Drake:PolynomalSystem:AlgebraicLoop','algebraic loop');
      end
      
      if (getNumZeroCrossings(sys1)>0 || getNumZeroCrossings(sys2)>0)
        error('polynomialsystems aren''t supposed to have zero crossings'); 
      end
      
      input_frame = sys1.getInputFrame();
      output_frame = sys1.getOutputFrame();
      if (sys1.getNumStates==0) 
        state_frame = sys2.getStateFrame();
      elseif (sys2.getNumStates==0)
        state_frame = sys1.getStateFrame();
      else
        state_frame = CoordinateFrame('FeedbackState',sys1.getNumState()+sys2.getNumStates(),'x');
      end

      p_x = state_frame.poly;
      p_u = input_frame.poly;
      
      [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2);
      p_x1 = p_x(sys1ind);
      p_x2 = p_x(sys2ind);
      
      [p1_dynamics_rhs,p1_dynamics_lhs]=getPolyDynamics(sys1);
      p1_output=getPolyOutput(sys1);
      p1_state_constraints=getPolyStateConstraints(sys1);
      [p2_dynamics_rhs,p2_dynamics_lhs]=getPolyDynamics(sys2);
      p2_output=getPolyOutput(sys2);
      p2_state_constraints=getPolyStateConstraints(sys2);
      
      if (~sys1.isDirectFeedthrough()) % do sys1 first
        p_y1 = subs(p1_output,sys1.getStateFrame.poly,p_x1); % doesn't need u
        p_y2 = subss(p2_output,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1]);
      else % do sys2 first
        p_y2 = subs(p2_output,sys2.getStateFrame.poly,p_x2); % doesn't need u
        p_y1 = subss(p1_output,[sys1.getStateFrame.poly;p_u],[p_x1;p_y2+p_u]);
      end
      
      p_dynamics_rhs=[]; p_dynamics_lhs=[]; p_update=[]; p_output=[]; p_state_constraints=[];
      if (sys1.getNumContStates()>0)
        p_dynamics_rhs=[p_dynamics_rhs;subss(p1_dynamics_rhs,[sys1.getStateFrame.poly;p_u],[p_x1;p_y2+p_u])];
      end
      if (sys2.getNumContStates()>0)
        p_dynamics_rhs=[p_dynamics_rhs;subss(p2_dynamics_rhs,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1])];
      end
      if isRational(sys1)
        if isRational(sys2)
          p_dynamics_lhs=blkdiag(subss(p_dynamics_lhs,sys1.getStateFrame.poly,p_x1),subss(sys2.p_dynamics_lhs,sys2.getStateFrame.poly,p_x2));
        else
          p_dynamics_lhs=blkdiag(subss(p_dynamics_lhs,sys1.getStateFrame.poly,p_x1),eye(getNumContStates(sys2)));
        end
      else
        if isRational(sys2)
          p_dynamics_lhs=blkdiag(eye(getNumContStates(sys1)),subss(sys2.p_dynamics_rhs,sys2.getStateFrame.poly,p_x2));
        else
          p_dynamics_lhs=[];
        end
      end
      if (sys1.getNumDiscStates()>0)
        p_update = [p_update;subss(p1_update,[sys1.getStateFrame.poly;p_u],[p_x1;p_y2+p_u])];
      end
      if (sys2.getNumDiscStates()>0)
        p_update = [p_update; subss(p2_update,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1])];
      end
      if (sys1.getNumOutputs()>0)
        p_output = p_y1;
      end
      
      % handle state constraints
      if (sys1.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(p1_state_constraints,sys1.getStateFrame.poly,p_x1)];
      end
      if (sys2.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(p2_state_constraints,sys2.getStateFrame.poly,p_x2)];
      end
      
      sys = SpotPolynomialSystem(input_frame,state_frame,output_frame,p_dynamics_rhs,p_dynamics_lhs,p_update,p_output,p_state_constraints);
      
      try
        sys = setSampleTime(sys,[sys1.getSampleTime(),sys2.getSampleTime()]);  % todo: if this errors, then kick out to drakesystem?
      catch ex
        if (strcmp(ex.identifier, 'Drake:DrakeSystem:UnsupportedSampleTime'))
          warning('Drake:PolynomialSystem:UnsupportedSampleTime','Aborting polynomial feedback because of incompatible sample times'); 
          sys = feedback@DrakeSystem(sys1,sys2);
          return;
        else
          rethrow(ex)
        end
      end
      
    end
    
    function sys = cascade(sys1,sys2)
      % try to keep cascade between polynomial systems polynomial.  else,
      % kick out to DrakeSystem
      %

      [sys1,sys2] = matchCoordinateFramesForCombination(sys1,sys2);

      if ~isa(sys2,'PolynomialSystem') || ~isTI(sys1) || ~isTI(sys2) || any(~isinf([sys2.umin;sys2.umax]))
        sys = cascade@DrakeSystem(sys1,sys2);
        return;
      end
        
      if (getNumZeroCrossings(sys1)>0 || getNumZeroCrossings(sys2)>0)
        error('polynomialsystems aren''t supposed to have zero crossings'); 
      end
      
      input_frame = sys1.getInputFrame();
      output_frame = sys2.getOutputFrame();
      if (sys1.getNumStates==0) 
        state_frame = sys2.getStateFrame();
      elseif (sys2.getNumStates==0)
        state_frame = sys1.getStateFrame();
      else
        state_frame = CoordinateFrame('FeedbackState',sys1.getNumState()+sys2.getNumStates(),'x');
      end
      
      p_x = state_frame.poly;
      p_u = input_frame.poly;
      
      [sys1ind,sys2ind] = stateIndicesForCombination(sys1,sys2);
      p_x1 = p_x(sys1ind);
      p_x2 = p_x(sys2ind);
      
      [p1_dynamics_rhs,p1_dynamics_lhs]=getPolyDynamics(sys1);
      p1_output=getPolyOutput(sys1);
      p1_state_constraints=getPolyStateConstraints(sys1);
      [p2_dynamics_rhs,p2_dynamics_lhs]=getPolyDynamics(sys2);
      p2_output=getPolyOutput(sys2);
      p2_state_constraints=getPolyStateConstraints(sys2);

      if (sys1.getNumStates()>0)
        p_y1 = subss(p1_output,sys1.getStateFrame.poly,p_x1);
      else
        p_y1 = p1_output;
      end

      p_dynamics_rhs=[]; p_dynamics_lhs=[]; p_update=[]; p_output=[]; p_state_constraints=[];
      if (sys1.getNumContStates()>0)
        p_dynamics_rhs=[p_dynamics_rhs;subss(p1_dynamics,sys1.getStateFrame.poly,p_x1)];
      end
      if (sys2.getNumContStates()>0)
        p_dynamics_rhs=[p_dynamics_rhs;subss(p2_dynamics,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1])];
      end
      if isRational(sys1)
        if isRational(sys2)
          p_dynamics_lhs=blkdiag(subss(p_dynamics_lhs,sys1.getStateFrame.poly,p_x1),subss(sys2.p_dynamics_lhs,sys2.getStateFrame.poly,p_x2));
        else
          p_dynamics_lhs=blkdiag(subss(p_dynamics_lhs,sys1.getStateFrame.poly,p_x1),eye(getNumContStates(sys2)));
        end
      else
        if isRational(sys2)
          p_dynamics_lhs=blkdiag(eye(getNumContStates(sys1)),subss(p_dynamics_lhs,sys2.getStateFrame.poly,p_x2));
        else
          p_dynamics_lhs=[];
        end
      end
      if (sys1.getNumDiscStates()>0)
        p_update = [p_update;subss(p1_update,sys1.getStateFrame.poly,p_x1)];
      end
      if (sys2.getNumDiscStates()>0)
        p_update = [p_update; subss(p2_update,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1])];
      end
      if (sys1.getNumOutputs()>0)
        p_output = subss(p2_output,[sys2.getStateFrame.poly;sys2.getInputFrame.poly],[p_x2;p_y1]);
      end
      if (sys1.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(p1_state_constraints,sys1.getStateFrame.poly,p_x1)];
      end
      if (sys2.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(p2_state_constraints,sys2.getStateFrame.poly,p_x2)];
      end

      sys = SpotPolynomialSystem(input_frame,state_frame,output_frame,p_dynamics_rhs,p_dynamics_lhs,p_update,p_output,p_state_constraints);

      try 
        sys = setSampleTime(sys,[sys1.getSampleTime(),sys2.getSampleTime()]);  % todo: if this errors, then kick out to drakesystem?
      catch ex
        if (strcmp(ex.identifier, 'Drake:DrakeSystem:UnsupportedSampleTime'))
          warning('Drake:PolynomialSystem:UnsupportedSampleTime','Aborting polynomial cascade because of incompatible sample times'); 
          sys = cascade@DrakeSystem(sys1,sys2);
        else
          rethrow(ex)
        end
      end
      
    end
  end
    
end
