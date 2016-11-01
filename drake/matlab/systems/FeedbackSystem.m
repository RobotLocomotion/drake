classdef FeedbackSystem < DrakeSystem

  properties
    sys1
    sys2
    sys1ind=[]
    sys2ind=[]
  end

  methods
    function obj = FeedbackSystem(sys1,sys2)
      obj = obj@DrakeSystem(sys1.getNumContStates()+sys2.getNumContStates(),...
        sys1.getNumDiscStates()+sys2.getNumDiscStates(),...
        sys1.getNumInputs(), sys1.getNumOutputs(), sys1.isDirectFeedthrough(), sys1.isTI() & sys2.isTI());
      typecheck(sys1,'DrakeSystem');
      typecheck(sys2,'DrakeSystem');

      if (isa(sys1,'HybridDrakeSystem') || isa(sys2,'HybridDrakeSystem'))
        error('Drake:FeedbackSystem:NoHybridSupport','feedback combinations with hybrid systems should be created using the hybrid system feedback method.');
      end
      if (isa(sys1,'StochasticDrakeSystem') || isa(sys2,'StochasticDrakeSystem'))
        error('Drake:FeedbackSystem:NoStochasticSupport','feedback combinations with stochastic systems not implemented yet.');
      end

      if any(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax]))
        error('Drake:FeedbackSystem:InternalInputLimits','feedback combinations with saturations were causing problems in simulation.  See https://github.com/RobotLocomotion/drake/issues/494');
      end
      
      sys2 = sys2.inInputFrame(sys1.getOutputFrame);
      sys2 = sys2.inOutputFrame(sys1.getInputFrame);

      if (sys1.isDirectFeedthrough() && sys2.isDirectFeedthrough())
        error('Drake:FeedbackSystem:AlgebraicLoop','algebraic loop');
      end

      [obj.sys1ind,obj.sys2ind] = stateIndicesForCombination(sys1,sys2);

      obj = setSampleTime(obj,[sys1.getSampleTime(),sys2.getSampleTime()]);
      obj = setNumZeroCrossings(obj,sys1.getNumZeroCrossings()+sys2.getNumZeroCrossings()+sum(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax])));

      for i=1:numel(sys1.state_constraints)
        obj = addStateConstraint(obj,sys1.state_constraints{i},obj.sys1ind(sys1.state_constraint_xind{i}));
      end
      for i=1:numel(sys2.state_constraints)
        obj = addStateConstraint(obj,sys2.state_constraints{i},obj.sys2ind(sys2.state_constraint_xind{i}));
      end

      obj = setInputFrame(obj,sys1.getInputFrame());
      obj = setOutputFrame(obj,sys1.getOutputFrame());

      obj = setStateFrame(obj,MultiCoordinateFrame.constructFrame( ...
        { getStateFrame(sys1),getStateFrame(sys2) }, ...
        [ ones(getNumContStates(sys1),1); ...
          2*ones(getNumContStates(sys2),1); ...
          ones(getNumDiscStates(sys1),1); ...
          2*ones(getNumDiscStates(sys2),1) ], ...
          true) );  % zap empty frames

      if ~isempty(sys1.state_constraints) || ~isempty(sys2.state_constraints)
        obj.warning_manager.warnOnce('Drake:FeedbackSystem:Todo','todo: still need to add state constriants for the feedback system');
      end

      obj.sys1=sys1;
      obj.sys2=sys2;
    end

    function xdn = update(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      [y1,y2]=getOutputs(obj,t,x,u);
      xdn=[];
      if (obj.sys1.getNumDiscStates()) xdn=[xdn;update(obj.sys1,t,x1,sat1(obj,y2+u))]; end
      if (obj.sys2.getNumDiscStates()) xdn=[xdn;update(obj.sys2,t,x2,sat2(obj,y1))]; end
    end
    function xcdot = dynamics(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      [y1,y2]=getOutputs(obj,t,x,u);
      xcdot=[];
      if (obj.sys1.getNumContStates()) xcdot=[xcdot;dynamics(obj.sys1,t,x1,sat1(obj,y2+u))]; end
      if (obj.sys2.getNumContStates()) xcdot=[xcdot;dynamics(obj.sys2,t,x2,sat2(obj,y1))]; end
    end
    function y = output(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      if (~obj.sys1.isDirectFeedthrough()) % do sys1 first
        % note: subsystems of sys1 could still be direct feedthrough.  but
        % their outputs will not effect the final y1.  I'll just put in
        % something random that's the right size for u here.
        y=output(obj.sys1,t,x1,u);  % output shouldn't depend on u
      else % do sys2 first
        y2=output(obj.sys2,t,x2,zeros(obj.sys2.num_u,1));  % output shouldn't depend on this u
        y=output(obj.sys1,t,x1,sat1(obj,y2+u));
      end
    end

    function x0=getInitialState(obj)
      if ~isempty(obj.initial_state)
        x0 = obj.initial_state;
        return;
      end

      x0=encodeX(obj,getInitialState(obj.sys1),getInitialState(obj.sys2));
    end

    function x0=getInitialStateWInput(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      [y1,y2]=getOutputs(obj,t,x,u);
      x1=getInitialStateWInput(obj.sys1,t,x1,sat1(obj,y2+u));
      x2=getInitialStateWInput(obj.sys2,t,x2,sat2(obj,y1));
      x0=encodeX(obj,x1,x2);
      % note: this is not perfect (y2 could change after updating x1).  how
      % does simulink do it on the backend?
    end

    function zcs = zeroCrossings(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      
      % same as getOutputs, but *without* the input saturations
      if (~obj.sys1.isDirectFeedthrough()) % do sys1 first
        y1=output(obj.sys1,t,x1,u);  % output shouldn't depend on u
        y2=output(obj.sys2,t,x2,y1);
      else % do sys2 first
        y2=output(obj.sys2,t,x2,zeros(obj.sys2.num_u,1));  % output shouldn't depend on this u
        y1=output(obj.sys1,t,x1,y2+u);
      end
      
      if (getNumZeroCrossings(obj.sys1)>0)
        zcs=zeroCrossings(obj.sys1,t,x1,sat1(obj,y2+u));
      else
        zcs=[];
      end
      if (getNumZeroCrossings(obj.sys2)>0)
        zcs=[zcs;zeroCrossings(obj.sys2,t,x2,sat2(obj,y1))];
      end

      return;  % the following code is disabled by throwing an error in the constructor
      
      % sys1 umin
      ind=find(~isinf(obj.sys1.umin));
      if (~isempty(ind)) zcs=[zcs;y2(ind)+u(ind) - obj.sys1.umin(ind)]; end

      % sys1 umax
      ind=find(~isinf(obj.sys1.umax));
      if (~isempty(ind)) zcs=[zcs;obj.sys1.umax(ind) - y2(ind)-u(ind)]; end

      % sys2 umin
      ind=find(~isinf(obj.sys2.umin));
      if (~isempty(ind)) zcs=[zcs;y1(ind) - obj.sys2.umin(ind)]; end

      % sys2 umax
      ind=find(~isinf(obj.sys2.umax));
      if (~isempty(ind)) zcs=[zcs;obj.sys2.umax(ind) - y1(ind)]; end
    end

  end

  methods (Access=private)
    function [x1,x2] = decodeX(obj,x)
      x1=x(obj.sys1ind);
      x2=x(obj.sys2ind);
    end
    function x = encodeX(obj,x1,x2)
      x(obj.sys2ind,1)=x2;  % x2 first so it only allocates once
      x(obj.sys1ind,1)=x1;
    end
    function u1=sat1(obj,u1)
      u1=min(max(u1,obj.sys1.umin),obj.sys1.umax);
    end
    function u2=sat2(obj,u2)
      u2=min(max(u2,obj.sys2.umin),obj.sys2.umax);
    end
    function [y1,y2] = getOutputs(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      if (~obj.sys1.isDirectFeedthrough()) % do sys1 first
        % note: subsystems of sys1 could still be direct feedthrough.  but
        % their outputs will not effect the final y1.  I'll just put in
        % something random that's the right size for u here.
        y1=output(obj.sys1,t,x1,u);  % output shouldn't depend on u
        y2=output(obj.sys2,t,x2,sat2(obj,y1));
      else % do sys2 first
        y2=output(obj.sys2,t,x2,zeros(obj.sys2.num_u,1));  % output shouldn't depend on this u
        y1=output(obj.sys1,t,x1,sat1(obj,y2+u));
      end
    end
  end

end
