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
      obj = obj@DrakeSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag);
      obj.rational_flag = rational_flag;
    end
  end

  methods (Sealed=true)
    function [xcdot,dxcdot] = dynamics(obj,t,x,u)
      if (nargout<2)
        xcdot = dynamicsRHS(obj,t,x,u);
        if (obj.rational_flag)
          lhs = dynamicsLHS(obj,t,x,u);
          xcdot = lhs \ xcdot;
        end
      else
        if ~obj.rational_flag
          try
            [xcdot,dxcdot] = dynamicsRHS(obj,t,x,u);
          catch
            if ~isTI(obj) error('not implemented yet'); end  % todo: geval equivalent for t, but poly computations for x,u?

            p_x=obj.getStateFrame.getPoly;
            if (obj.num_u>0) p_u=obj.getInputFrame.getPoly; else p_u=[]; end

            f = getPolyDynamics(obj,0);
            xcdot = double(subs(f,[p_x;p_u],[x;u]));
            dxcdot = double([0*xcdot,subs(diff(f,[p_x;p_u]),[p_x;p_u],[x;u])]);
          end
        else
          if ~isTI(obj) error('not implemented yet'); end  % todo: geval equivalent for t, but poly computations for x,u?

          p_x=obj.getStateFrame.getPoly;
          if (obj.num_u>0) p_u=obj.getInputFrame.getPoly; else p_u=[]; end

          [f,e] = getPolyDynamics(obj,0);
          xcdot = double(subs(e,p_x,x))\double(subs(f,[p_x;p_u],[x;u]));
          df = double([0*xcdot,subs(diff(f,[p_x;p_u]),[p_x;p_u],[x;u])]);

          % e(x)xdot = f(x) => dexdotdx + e(x)*dxdotdx = dfdx
          %  where the columns of dexdotdx are dedxi*xdot
          nX = obj.num_xc;
%          cellxcdot=cell(1,nX); [cellxcdot{:}]=deal(xcdot);
%          dexdotdx = reshape(double(subs(diff(e(:),p_x),p_x,x)),nX,[])*blkdiag(cellxcdot{:});
          dexdotdx = double(subs(diff(e*xcdot,p_x),p_x,x));
          dxcdot = double(subs(e,p_x,x))\(df - [zeros(nX,1),dexdotdx,zeros(nX,obj.num_u)]);
        end
      end
    end
  end



  methods
    function [e,de] = dynamicsLHS(obj,t,x,u)
      error('rational polynomial systems with continuous state must implement this');
    end

    function [f,df] = dynamicsRHS(obj,t,x,u)
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
        p_x=obj.getStateFrame.getPoly;
        if (obj.num_u>0) p_u=obj.getInputFrame.getPoly; else p_u=[]; end

        p_dynamics_rhs=obj.dynamicsRHS(t,p_x,p_u);
        if (nargout>1 && obj.rational_flag)
          p_dynamics_lhs=obj.dynamicsLHS(t,p_x,p_u);
        end
      end
    end
    function p_update = getPolyUpdate(obj,t)
      if (obj.num_xd>0)
        if (nargin<2 && ~isTI(obj)) error('you must specify a time'); else t=0; end
        p_x=obj.getStateFrame.getPoly;
        if (obj.num_u>0) p_u=obj.getInputFrame.getPoly; else p_u=[]; end

        p_update=obj.update(t,p_x,p_u);
      else
        p_update=[];
      end
    end
    function p_output = getPolyOutput(obj,t)
      if (obj.num_y>0)
        if (nargin<2 && ~isTI(obj)) error('you must specify a time'); else t=0; end
        if (obj.num_x>0) p_x=obj.getStateFrame.getPoly; else p_x=[]; end
        if (obj.num_u>0) p_u=obj.getInputFrame.getPoly; else p_u=[]; end

        p_output=obj.output(t,p_x,p_u);
      else
        p_output=[];
      end
    end
    function p_state_constraints = getPolyStateConstraints(obj)
      if (obj.num_xcon_eq>0)
        p_x=obj.getStateFrame.getPoly;
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
        if obj.getNumStates()>0 && hasSamePrefix(obj.getStateFrame,frame)
          error('input frame poly clashes with current state frame poly.  this could lead to massive confusion');
        end
      end

      obj = setInputFrame@DrakeSystem(obj,frame);
    end

    function obj = setStateFrame(obj,frame)
      if frame.dim>0
        if obj.getNumInputs()>0 && hasSamePrefix(obj.getInputFrame,frame)
          error('state frame poly clashes with current input frame poly.  this could lead to massive confusion');
        end
      end

      obj = setStateFrame@DrakeSystem(obj,frame);
    end

    function sys = inStateFrame(sys,frame)
      if (sys.getStateFrame == frame) return; end

      if (~isCT(sys) || isRational(sys) || ~isempty(sys.state_constraints)), error('not implemented yet'); end   % though some of them would be easy to implement

      ctf = findTransform(getStateFrame(sys),frame,struct('throw_error_if_fail',true));
      dtf = findTransform(frame,getStateFrame(sys),struct('throw_error_if_fail',true));
      if ~isa(ctf,'PolynomialSystem') || ~isa(dtf,'PolynomialSystem') || getNumStates(ctf)>0 || getNumStates(dtf)>0
        error('unsupported transform');
      end

      % from xdot = f(t,x,u) to zdot = g(t,z,u) via z=c(t,x) and x=d(t,z)
      % zdot = dcdt(t,d(t,z)) + dcdx(t,d(t,z)*f(t,d(t,z),u)

      x = sys.getStateFrame.getPoly; z=frame.getPoly;
      if isTI(sys) && isTI(ctf) && isTI(dtf)
        c = subs(ctf.getPolyOutput,ctf.getInputFrame.getPoly,x);
        d = subs(dtf.getPolyOutput,dtf.getInputFrame.getPoly,z);
        g = diff(c,x)*sys.getPolyDynamics;
        g = subss(g,x,d);
        y = subss(sys.getPolyOutput,x,d);
        sys = SpotPolynomialSystem(sys.getInputFrame,frame,sys.getOutputFrame,g,[],[],y);
      else % time-varying case
        y = @(t) subss(sys.getPolyOutput(t),x,subs(dtf.getPolyOutput(t),dtf.getInputFrame.getPoly,z));
        % todo: note the breaks in the polynomialtrajectory systems below are a hack. need to handle that better.
        sys = PolynomialTrajectorySystem(sys.getInputFrame,frame,sys.getOutputFrame,PolynomialTrajectory(@(t) newdyn(t,sys,frame,ctf,dtf),[0 inf]),[],PolynomialTrajectory(y,[0 inf]),sys.isDirectFeedthrough);
      end

        function g = newdyn(t,sys,frame,ctf,dtf)
          x = sys.getStateFrame.getPoly; z=frame.getPoly;
          p_t = msspoly('t',1);
          [c,dc]=ctf.output(t,[],x);
          d = dtf.output(t,[],z);
          dcdt = dc(:,1); dcdx = dc(:,2:end);
          g = dcdt + dcdx*sys.getPolyDynamics(t);
          g = subss(g,x,d);
        end


    end

    function sys = feedback(sys1,sys2)
      % try to keep feedback between polynomial systems polynomial.  else,
      % kick out to DrakeSystem
      %

      sys2 = sys2.inInputFrame(sys1.getOutputFrame);
      sys2 = sys2.inOutputFrame(sys1.getInputFrame);

      if ~isa(sys2,'PolynomialSystem') || ~isTI(sys1) || ~isTI(sys2) || any(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax])) || getNumUnilateralConstraints(sys1)>0 || getNumUnilateralConstraints(sys2)>0
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

      p_x = state_frame.getPoly;
      p_u = input_frame.getPoly;

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
        p_y1 = subs(p1_output,sys1.getStateFrame.getPoly,p_x1); % doesn't need u
        p_y2 = subss(p2_output,[sys2.getStateFrame.getPoly;sys2.getInputFrame.getPoly],[p_x2;p_y1]);
      else % do sys2 first
        p_y2 = subs(p2_output,sys2.getStateFrame.getPoly,p_x2); % doesn't need u
        p_y1 = subss(p1_output,[sys1.getStateFrame.getPoly;sys1.getInputFrame.getPoly],[p_x1;p_y2+p_u]);
      end

      p_dynamics_rhs=[]; p_dynamics_lhs=[]; p_update=[]; p_output=[]; p_state_constraints=[];
      if (sys1.getNumContStates()>0)
        p_dynamics_rhs=[p_dynamics_rhs;subss(p1_dynamics_rhs,[sys1.getStateFrame.getPoly;sys1.getInputFrame.getPoly],[p_x1;p_y2+p_u])];
      end
      if (sys2.getNumContStates()>0)
        p_dynamics_rhs=[p_dynamics_rhs;subss(p2_dynamics_rhs,[sys2.getStateFrame.getPoly;sys2.getInputFrame.getPoly],[p_x2;p_y1])];
      end
      if isRational(sys1)
        if isRational(sys2)
          p_dynamics_lhs=blkdiag(subss(p_dynamics_lhs,sys1.getStateFrame.getPoly,p_x1),subss(sys2.p_dynamics_lhs,sys2.getStateFrame.getPoly,p_x2));
        else
          p_dynamics_lhs=blkdiag(subss(p_dynamics_lhs,sys1.getStateFrame.getPoly,p_x1),eye(getNumContStates(sys2)));
        end
      else
        if isRational(sys2)
          p_dynamics_lhs=blkdiag(eye(getNumContStates(sys1)),subss(sys2.p_dynamics_rhs,sys2.getStateFrame.getPoly,p_x2));
        else
          p_dynamics_lhs=[];
        end
      end
      if (sys1.getNumDiscStates()>0)
        p_update = [p_update;subss(p1_update,[sys1.getStateFrame.getPoly;sys1.getInputFrame.getPoly],[p_x1;p_y2+p_u])];
      end
      if (sys2.getNumDiscStates()>0)
        p_update = [p_update; subss(p2_update,[sys2.getStateFrame.getPoly;sys2.getInputFrame.getPoly],[p_x2;p_y1])];
      end
      if (sys1.getNumOutputs()>0)
        p_output = p_y1;
      end

      % handle state constraints
      if (sys1.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(p1_state_constraints,sys1.getStateFrame.getPoly,p_x1)];
      end
      if (sys2.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(p2_state_constraints,sys2.getStateFrame.getPoly,p_x2)];
      end
      if (sys1.getNumUnilateralConstraints()>0 || sys2.getNumUnilateralConstraints()>0)
        error('not implemented yet')
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

      sys2 = sys2.inInputFrame(sys1.getOutputFrame);

      if ~isa(sys2,'PolynomialSystem') || ~isTI(sys1) || ~isTI(sys2) || any(~isinf([sys2.umin;sys2.umax])) || getNumUnilateralConstraints(sys1)>0 || getNumUnilateralConstraints(sys2)>0
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

      p_x = state_frame.getPoly;
      p_u = input_frame.getPoly;

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
        p_y1 = subss(p1_output,sys1.getStateFrame.getPoly,p_x1);
      else
        p_y1 = p1_output;
      end

      p_dynamics_rhs=[]; p_dynamics_lhs=[]; p_update=[]; p_output=[]; p_state_constraints=[];
      if (sys1.getNumContStates()>0)
        p_dynamics_rhs=[p_dynamics_rhs;subss(p1_dynamics,sys1.getStateFrame.getPoly,p_x1)];
      end
      if (sys2.getNumContStates()>0)
        p_dynamics_rhs=[p_dynamics_rhs;subss(p2_dynamics,[sys2.getStateFrame.getPoly;sys2.getInputFrame.getPoly],[p_x2;p_y1])];
      end
      if isRational(sys1)
        if isRational(sys2)
          p_dynamics_lhs=blkdiag(subss(p_dynamics_lhs,sys1.getStateFrame.getPoly,p_x1),subss(sys2.p_dynamics_lhs,sys2.getStateFrame.getPoly,p_x2));
        else
          p_dynamics_lhs=blkdiag(subss(p_dynamics_lhs,sys1.getStateFrame.getPoly,p_x1),eye(getNumContStates(sys2)));
        end
      else
        if isRational(sys2)
          p_dynamics_lhs=blkdiag(eye(getNumContStates(sys1)),subss(p_dynamics_lhs,sys2.getStateFrame.getPoly,p_x2));
        else
          p_dynamics_lhs=[];
        end
      end
      if (sys1.getNumDiscStates()>0)
        p_update = [p_update;subss(p1_update,sys1.getStateFrame.getPoly,p_x1)];
      end
      if (sys2.getNumDiscStates()>0)
        p_update = [p_update; subss(p2_update,[sys2.getStateFrame.getPoly;sys2.getInputFrame.getPoly],[p_x2;p_y1])];
      end
      if (sys1.getNumOutputs()>0)
        p_output = subss(p2_output,[sys2.getStateFrame.getPoly;sys2.getInputFrame.getPoly],[p_x2;p_y1]);
      end
      if (sys1.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(p1_state_constraints,sys1.getStateFrame.getPoly,p_x1)];
      end
      if (sys2.getNumStateConstraints()>0)
        p_state_constraints=[p_state_constraints;subss(p2_state_constraints,sys2.getStateFrame.getPoly,p_x2)];
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

    function sys = extractAffineSystem(obj)
      spsys = extractPolynomialSystem(obj);  % first get the spot version
      sys = extractAffineSystem(spsys);
    end
  end

end
