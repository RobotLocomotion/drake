classdef ParameterizedSystem < DrakeSystem
  
  % A drake system should publish this interface iff 
  % it can provide the methods:
  %   setParams
  %   getParams
  %
  % Alternatively, if the system calls setParamFrame with the coordinates
  % names as property names, then this class provides default setParam and
  % getParam methods which will populate/read from the named properties.
  
  methods
    function obj = setParams(obj,p)
      % This default setParams method attempts to set class properties of the
      % system according to the coordinate names in the parameter frame.

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
      % This default getParams method attempts to get class properties of the
      % system according to the coordinate names in the parameter frame.

      c = obj.param_frame.coordinates;
      p=zeros(obj.param_frame.dim,1);
      for i=1:length(c)
        p(i) = obj.(c{i});
      end
      p = Point(obj.param_frame,p);
    end
    
  end
  
  methods
    function obj = setParamFrame(obj,fr)
      % Set the CoordinateFrame object which describes any system
      % parameters
      typecheck(fr,'CoordinateFrame');
      obj.param_frame = fr;
    end
    function fr = getParamFrame(obj)
      % Returns the CoordinateFrame object which describes any system
      % parameters
      fr = obj.param_frame;
    end
    
    function obj = setParamLimits(obj,pmin,pmax)
      % Set lower and upper bounds on the system parameters
      d = prod(obj.param_frame.dim);
      sizecheck(pmin,[d 1]);
      obj.pmin = pmin;
      if nargin>2,
        sizecheck(pmax,[d 1]);
        obj.pmax = pmax; 
      end
    end
    
    function [pmin,pmax] = getParamLimits(obj)
      % Returns the current lower and upper bounds on the system parameters
      pmin = obj.pmin;
      pmax = obj.pmax;
    end
  end
  
  methods % actual worker methods
    function phat = parameterEstimation(obj,data,options)
      % Estimate parameter values from input-output data
      %
      % @param data an instance of iddata (see 'help iddata' for more info)
      
      error('todo: implement a generic nonlinear optimization approach here');
    end
  end
  
  properties (SetAccess=private, GetAccess=private)
    param_frame;
    pmin;
    pmax;
  end
end  