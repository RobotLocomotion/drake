classdef SmoothRobotLibSystem < RobotLibSystem

  % constructor
  methods
    function obj = SmoothRobotLibSystem(num_xc,num_xd,num_u,num_y,direct_feedthrough_flag,time_invariant_flag)
      obj = obj@RobotLibSystem(num_xc,num_xd,num_u,num_y);
      if (nargin>=4) obj = setNumOutputs(obj,num_y); end
      if (nargin>=5) obj = setDirectFeedthrough(obj,direct_feedthrough_flag); end
      if (nargin>=6) obj = setTIFlag(obj,time_invariant_flag); end
    end      
  end
      
  % default methods - these should be implemented or overwritten
  % 
  methods
    function x0 = getInitialState(obj)
      x0 = zeros(obj.num_xd+obj.num_xc,1);
    end
    
    function xcdot = dynamics(obj,t,x,u)
      error('systems with continuous states must implement Derivatives');
    end
    
    function xdn = update(obj,t,x,u)
      error('systems with discrete states must implement Update');
    end
    
    function y = output(obj,t,x,u)
      error('default is intentionally not implemented');
    end
    
  end      

  % utility methods
  methods
    function [A,B,C,D,x0dot,y0] = linearize(obj,t,x0,u0)
      if (~isCT(obj) || getNumDiscStates(obj)>0)  % boot if it's not the simple case
        [A,B,C,D,x0dot,y0] = linearize@DynamicalSystem(obj,t,x0,u0);
      end
      
      nX = getNumContStates(obj);
      nU = getNumInputs(obj);
      [f,df] = geval(@obj.dynamics,t,x0,u0);
      A = df(:,1+(1:nX));
      B = df(:,nX+1+(1:nU));
      
      if (nargout>2)
        [y,dy] = geval(@obj.output,t,x0,u0);
        C = dy(:,1+(1:nX));
        D = dy(:,nX+1+(1:nU));
        if (nargout>4)
          x0dot = dynamics(obj,t,x0,u0);
          if (nargout>5)
            y0 = output(obj,t,x0,u0);
          end
        end
      end
    end

    function sys=feedback(sys1,sys2)
      try 
        sys=FeedbackSystem(sys1,sys2);  % try to keep it a smoothrobotlibsystem
      catch
        sys=feedback@DynamicalSystem(sys1,sys2);
      end
    end

  end
  

end
