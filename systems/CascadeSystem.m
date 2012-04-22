classdef CascadeSystem < SmoothRobotLibSystem
  
  properties
    sys1
    sys2
    sys1ind=[]
    sys2ind=[]
  end
  
  methods
    function obj = CascadeSystem(sys1,sys2)
      obj = obj@SmoothRobotLibSystem(sys1.getNumContStates()+sys2.getNumContStates(),...
        sys1.getNumDiscStates()+sys2.getNumDiscStates(),...
        sys1.getNumInputs(), sys2.getNumOutputs(), sys1.isDirectFeedthrough() & sys2.isDirectFeedthrough(), sys1.isTI() & sys2.isTI());
      typecheck(sys1,'SmoothRobotLibSystem');
      typecheck(sys2,'SmoothRobotLibSystem');
      obj.sys1=sys1;
      obj.sys2=sys2;

      if (any(~isinf([sys2.umin;sys2.umax])))
        error('saturations on system 2 are not supported yet');
      end
      obj = obj.setInputLimits(sys1.umin,sys2.umax);
      
      ind=0;
      n=obj.sys1.getNumDiscStates();
      obj.sys1ind = ind+(1:n)';
      ind=ind+n;
      n=obj.sys2.getNumDiscStates();
      obj.sys2ind=  ind+(1:n)'; ind=ind+n;

      n=obj.sys1.getNumContStates();
      obj.sys1ind = [obj.sys1ind; ind+(1:n)'];  ind=ind+n;
      n=obj.sys2.getNumContStates();
      obj.sys2ind = [obj.sys2ind; ind+(1:n)'];  
    end
    
    function [x1,x2] = decodeX(obj,x)
      x1=x(obj.sys1ind);
      x2=x(obj.sys2ind);
    end
    function x = encodeX(obj,x1,x2)
      x(obj.sys2ind)=x2;  % x2 first so it only allocates once
      x(obj.sys1ind)=x1;
    end
    
    function xdn = update(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      xdn=[];
      if (obj.sys1.getNumDiscStates()) xdn=[xdn;update(obj.sys1,t,x1,u)]; end
      if (obj.sys2.getNumDiscStates()) xdn=[xdn;update(obj.sys2,t,x2,output(obj.sys1,t,x1,u))]; end
    end
    function xcdot = dynamics(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      xcdot=[];
      if (obj.sys1.getNumContStates()) xcdot=[xcdot;dynamics(obj.sys1,t,x1,u)]; end
      if (obj.sys2.getNumContStates()) xcdot=[xcdot;dynamics(obj.sys2,t,x2,output(obj.sys1,t,x1,u))]; end
    end
    function y = output(obj,t,x,u)
      if (nargin<4) u=[]; end  % easiest way to handle the non-direct feedthrough case
      [x1,x2]=decodeX(obj,x);
      y = obj.sys2.output(t,x2,obj.sys1.output(t,x1,u));
    end
    
    function x0=getInitialState(obj)
      x0=encodeX(obj,getInitialState(obj.sys1),getInitialState(obj.sys2));
    end

    function x0=getInitialStateWInput(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      x1=getInitialStateWInput(obj.sys1,t,x1,u);
      x2=getInitialStateWInput(obj.sys2,t,x2,obj.sys1.output(t,x1,u));
      x0=encodeX(obj,x1,x2);
    end
  end
  
end
    
