classdef ScopeSystem < ConstOrPassthroughSystem
% A simple system that will pass the input through to the output,
% but publish it to the lcmscope on the way.  

  properties
    options;
  end

  methods
    function obj = ScopeSystem(fr,options)
      typecheck(fr,'CoordinateFrame');
      obj = obj@ConstOrPassthroughSystem(nan(fr.dim,1));
      obj = setInputFrame(obj,fr);
      obj = setOutputFrame(obj,fr);
      obj.options = options;
    end
    
    function y=output(obj,t,x,u)
      y=output@ConstOrPassthroughSystem(obj,t,x,u);
      scope(obj.getOutputFrame,t,y,obj.options);
    end
  end
  
end