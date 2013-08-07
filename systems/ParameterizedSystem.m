classdef ParameterizedSystem < DrakeSystem
  
  % A drake system should publish this interface iff 
  % it can provide the methods:
  %   setParams
  %   getParams
  % and optionally 
  %   paramConstraints
  %
  % Alternatively, if the system calls setParamFrame with the coordinates
  % names as property names, then this class provides default setParam and
  % getParam methods which will populate/read from the named properties.
  
  methods
    function obj = setParams(obj,p)
      if isa(p,'Point')
        p = double(inFrame(p,obj.param_frame));
      else
        sizecheck(p,obj.param_frame.dim);
      end
      c = obj.param_frame.coordinates;
      for i=1:length(c)
        obj.(c{i}) = p(i);
      end
    end
    
    function p = getParams(obj)
      c = obj.param_frame.coordinates;
      p=zeros(obj.param_frame.dim,1);
      for i=1:length(c)
        p(i) = obj.(c{i});
      end
      p = Point(obj.param_frame,p);
    end
    
    function [phi_ineq,phi_eq] = paramConstraints(obj,p)
      % adds constraints that phi_ineq<=0, phi_eq==0 for param I.D.
      if (obj.num_param_con_ineq>0 || obj.num_param_con_eq>0)
        error('systems with parameter constraints must overload this method');
      end
    end
  end
  
  methods (Access=protected)
    function obj = setNumParamConstraints(obj,num_ineq,num_eq)
      sizecheck(num_ineq,1);
      integervaluedcheck(num_ineq);
      rangecheck(num_ineq,0,inf);
      obj.num_param_con_ineq = num_ineq;

      sizecheck(num_eq,1);
      integervaluedcheck(num_eq);
      rangecheck(num_eq,0,inf);
      obj.num_param_con_eq = num_eq;
    end
  end
  
  methods
    function [num_ineq,num_eq] = getNumParamConstraints(obj)
      num_ineq = obj.num_param_con_ineq;
      num_eq = obj.num_param_con_eq;
    end
    
    function obj = setParamFrame(obj,fr)
      typecheck(fr,'CoordinateFrame');
      obj.param_frame = fr;
    end
    function fr = getParamFrame(obj)
      fr = obj.param_frame;
    end
  end
  
  methods % actual worker methods
    function phat = parameterEstimation(obj,data,options)
      % @param data an instance of iddata (see 'help iddata' for more info)
      error('todo: implement a generic nonlinear optimization approach here');
    end
  end
  
  properties
    num_param_con_ineq=0;
    num_param_con_eq=0;
    param_frame;
  end
end  