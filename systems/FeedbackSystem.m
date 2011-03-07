classdef FeedbackSystem < SmoothRobotLibSystem
  
  properties
    sys1
    sys2
    sys1ind=[]
    sys2ind=[]
  end
  
  methods
    function obj = FeedbackSystem(sys1,sys2)
      obj = obj@SmoothRobotLibSystem(sys1.getNumContStates()+sys2.getNumContStates(),...
        sys1.getNumDiscStates()+sys2.getNumDiscStates(),...
        0, sys1.getNumOutputs(), false, sys1.isTI() & sys2.isTI());
      typecheck(sys1,'SmoothRobotLibSystem');
      typecheck(sys2,'SmoothRobotLibSystem');
      obj.sys1=sys1;
      obj.sys2=sys2;

      if (sys1.isDirectFeedthrough() && sys2.isDirectFeedthrough())
        error('algebraic loop');
      end
      if (any(~isinf([sys1.umin;sys1.umax;sys2.umin;sys2.umax])))
        error('saturations not supported yet');
      end
      
      ind=0;
      n=obj.sys1.getNumDiscStates();
      obj.sys1ind = ind+(1:n);
      ind=ind+n;
      n=obj.sys2.getNumDiscStates();
      obj.sys2ind=  ind+(1:n); ind=ind+n;

      n=obj.sys1.getNumContStates();
      obj.sys1ind = [obj.sys1ind, ind+(1:n)];  ind=ind+n;
      n=obj.sys2.getNumContStates();
      obj.sys2ind = [obj.sys2ind, ind+(1:n)];  
    end
    
    function [x1,x2] = decodeX(obj,x)
      x1=x(obj.sys1ind);
      x2=x(obj.sys2ind);
    end
    function x = encodeX(obj,x1,x2)
      x(obj.sys2ind)=x2;  % x2 first so it only allocates once
      x(obj.sys1ind)=x1;
    end
    function [y1,y2] = getOutputs(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      if (~obj.sys1.isDirectFeedthrough()) % do sys1 first
        y1=output(obj.sys1,t,x1);  % doesn't need u
        y2=output(obj.sys2,t,x2,y1);
      else % do sys2 first
        y2=output(obj.sys2,t,x2);  % doesn't need u
        y1=output(obj.sys1,t,x1,y2);
      end
    end
    
    function xdn = update(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      [y1,y2]=getOutputs(obj,t,x,u);
      xdn=[];
      if (obj.sys1.getNumDiscStates()) xdn=[xdn;update(obj.sys1,t,x1,y2)]; end
      if (obj.sys2.getNumDiscStates()) xdn=[xdn;update(obj.sys2,t,x2,y1)]; end
    end
    function xcdot = dynamics(obj,t,x,u)
      [x1,x2]=decodeX(obj,x);
      [y1,y2]=getOutputs(obj,t,x,u);
      xcdot=[];
      if (obj.sys1.getNumContStates()) xcdot=[xcdot;dynamics(obj.sys1,t,x1,y2)]; end
      if (obj.sys2.getNumContStates()) xcdot=[xcdot;dynamics(obj.sys2,t,x2,y1)]; end
    end
    function y = output(obj,t,x,u)
      [y1,y2] = getOutputs(obj,t,x,u);
      y=y1;
    end
    
  end
  
end
    