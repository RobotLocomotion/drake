classdef CascadeSystem < DrakeSystem

  properties
    sys1
    sys2
    sys1ind=[]
    sys2ind=[]
  end

  methods
    function obj = CascadeSystem(sys1,sys2)
      obj = obj@DrakeSystem(sys1.getNumContStates()+sys2.getNumContStates(),...
        sys1.getNumDiscStates()+sys2.getNumDiscStates(),...
        sys1.getNumInputs(), sys2.getNumOutputs(), sys1.isDirectFeedthrough() & sys2.isDirectFeedthrough(), sys1.isTI() & sys2.isTI());
      typecheck(sys1,'DrakeSystem');
      typecheck(sys2,'DrakeSystem');

      if (isa(sys1,'HybridDrakeSystem') || isa(sys2,'HybridDrakeSystem'))
        error('Drake:CascadeSystem:NoHybridSupport','cascade combinations with hybrid systems not implemented yet.');
      end
      if (isa(sys1,'StochasticDrakeSystem') || isa(sys2,'StochasticDrakeSystem'))
        error('Drake:CascadeSystem:NoStochasticSupport','cascade combinations with stochastic systems not implemented yet.');
      end

      sys2 = sys2.inInputFrame(sys1.getOutputFrame);

      obj = obj.setInputLimits(sys1.umin,sys1.umax);

      [obj.sys1ind,obj.sys2ind] = stateIndicesForCombination(sys1,sys2);

      obj = setNumZeroCrossings(obj,sys1.getNumZeroCrossings()+sys2.getNumZeroCrossings()+sum(~isinf([sys2.umin;sys2.umax])));
      
      for i=1:numel(sys1.state_constraints)
        obj = addStateConstraint(obj,sys1.state_constraints{i},obj.sys1ind(sys1.state_constraint_xind{i}));
      end
      for i=1:numel(sys2.state_constraints)
        obj = addStateConstraint(obj,sys2.state_constraints{i},obj.sys2ind(sys2.state_constraint_xind{i}));
      end
      
      obj = setSampleTime(obj,[sys1.getSampleTime(),sys2.getSampleTime()]);

      obj = setInputFrame(obj,sys1.getInputFrame());
      obj = setOutputFrame(obj,sys2.getOutputFrame());
      obj = setStateFrame(obj,MultiCoordinateFrame.constructFrame( ...
        { getStateFrame(sys1),getStateFrame(sys2) }, ...
        [ ones(getNumContStates(sys1),1); ...
          2*ones(getNumContStates(sys2),1); ...
          ones(getNumDiscStates(sys1),1); ...
          2*ones(getNumDiscStates(sys2),1) ], ...
          true) );  % zap empty frames

      obj.sys1=sys1;
      obj.sys2=sys2;
    end

    function [x1,x2] = decodeX(obj,x)
      x1=x(obj.sys1ind);
      x2=x(obj.sys2ind);
    end
    function x = encodeX(obj,x1,x2)
      x(obj.sys2ind,1)=x2;  % x2 first so it only allocates once
      x(obj.sys1ind,1)=x1;
    end

    function xdn = update(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      xdn=[];
      if (obj.sys1.getNumDiscStates()) xdn=[xdn;update(obj.sys1,t,x1,u)]; end
      if (obj.sys2.getNumDiscStates()) xdn=[xdn;update(obj.sys2,t,x2,sat2(obj,output(obj.sys1,t,x1,u)))]; end
    end
    function xcdot = dynamics(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      xcdot=[];
      if (obj.sys1.getNumContStates()) xcdot=[xcdot;dynamics(obj.sys1,t,x1,u)]; end
      if (obj.sys2.getNumContStates()) xcdot=[xcdot;dynamics(obj.sys2,t,x2,sat2(obj,output(obj.sys1,t,x1,u)))]; end
    end
    function y = output(obj,t,x,u)
      if (nargin<4) u=[]; end  % easiest way to handle the non-direct feedthrough case
      [x1,x2]=decodeX(obj,x);
      y = obj.sys2.output(t,x2,sat2(obj,output(obj.sys1,t,x1,u)));
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
      x1=getInitialStateWInput(obj.sys1,t,x1,u);
      x2=getInitialStateWInput(obj.sys2,t,x2,sat2(obj,output(obj.sys1,t,x1,u)));
      x0=encodeX(obj,x1,x2);
    end

    function zcs = zeroCrossings(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      y1=output(obj.sys1,t,x1,u);
      if (getNumZeroCrossings(obj.sys1)>0)
        zcs=zeroCrossings(obj.sys1,t,x1,u);
      else
        zcs=[];
      end
      if (getNumZeroCrossings(obj.sys2)>0)
        zcs=[zcs;zeroCrossings(obj.sys2,t,x2,sat2(obj,y1))];
      end

      % sys2 umin
      ind=find(~isinf(obj.sys2.umin));
      if (~isempty(ind)) zcs=[zcs;y1(ind) - obj.sys2.umin(ind)]; end

      % sys2 umax
      ind=find(~isinf(obj.sys2.umax));
      if (~isempty(ind)) zcs=[zcs;obj.sys2.umax(ind) - y1(ind)]; end
    end
    
    % todo: implement cascade, and if sys1 or sys2 can be cascaded more
    % efficient (e.g. two affinesystems), then do it internally instead of
    % adding a nested cascade system.  Will require getting precendence over
    % the polynomial class hierarchy, so that this method is called even
    % when the *second* argument is a cascade system.
    % http://www.mathworks.com/help/techdoc/matlab_oop/f1-6987.html
    % also note that:
    % "Subclasses do not inherit a superclass InferiorClasses attribute.
    % Only instances of the classes specified in the subclass InferiorClasses
    % attribute are inferior to subclass objects."
  end

  methods (Access=private)
    function u2=sat2(obj,u2)
%      u2=min(max(u2,obj.sys2.umin),obj.sys2.umax);
      if any(~isinf(obj.sys2.umin)) % writing it out with if's helps msspoly get through
        u2 = max(u2,obj.sys2.umin);
      end
      if any(~isinf(obj.sys2.umax))
        u2 = min(u2,obj.sys2.umax);
      end
    end
  end
end
