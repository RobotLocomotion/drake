classdef FunctionWrapper
  %FUNCTIONWRAPPER 
  % A simple class to wrap a function handle. It seems like this should not
  % be necessary, but somehow when using class methods this avoids a large
  % amount of overhead
  
  properties (Access = private)
    function_handle
  end
  
  methods
    function obj = FunctionWrapper(function_handle)
      obj.function_handle = function_handle;
    end
    
    function x = eval(obj, varargin)
      x = obj.function_handle(varargin{:});
    end
  end
  
end

